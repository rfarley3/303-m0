/*  Create a 303-ish synth with potentiometer controllers.
 *  (c) Ryan Farley ryanjfarley@gmail.com rfarley3@github
 *  
 *  when you load the project in Arduino
 *  * install Mozzi, MIDI
 *  * open adc.ino, dotstar.ino

    Mozzi documentation/API
    https://sensorium.github.io/Mozzi/doc/html/index.html
    Many ideas and circuits taken from this series:
    https://diyelectromusic.wordpress.com/2021/03/20/trinket-fm-synthesis-with-mozzi/
    https://diyelectromusic.wordpress.com/2021/02/15/midi-in-for-3-3v-microcontrollers/
    https://diyelectromusic.wordpress.com/2021/05/29/midi-connections-cheat-sheet/
    https://github.com/diyelectromusic/sdemp/blob/main/src/SDEMP/TrinketUSBMIDIMultiPotMozziSynth3/TrinketUSBMIDIMultiPotMozziSynth3.ino
    ^^ shows USB host mode
    
    Circuit with Adafruit Trinket M0
    * pin 1 (DAC 10b) audio output 
      * 1k Ohm with (1k Ohm voltage divider (halving 3.3 -> 1.625) & 10 uF coupling capacitor (biasing as AC += .8v))
      * neg of cap to jack tip
    * pin 0 to I2C SDA
    * pin 2 to I2C SCL
    * pin 3 (RX) from MIDI
    * pin 4 (ADC 12b) from 10k pot for filter cut freq
    * ADC (ADS7830 8 chan 8b) addr x4b
      * Y A0 res
      * Y A1 wave
      * Y A2 env mod
      * Y A3 decay
      *   A4 accent
      *   A5 trig (temporary for not needing keyboard)
      *   A6 TBD
      *   A7 TBD
   
    TODO
    * add Serial.read to send notes over TTY and not need keyboard for testing
    * SAMD21 ADC bug
    * zombie osc_playing, is this an issue?
    * test accent on everyother, add math for VENV boost
    * test accent to dcy shortening
    * work math for env_mod to cut
    *   w/o accent: reduce cut by env_mod, then pluck it up to the amount it was reduced to (cut is lower and pluck is centered on actual cut)
    *   w   accent: reduce cut by env_mod, make a second dcy that is reduced by accent minus a diode and reduced?smoothed?what? by res

    Version road map:
      * Y verify turns on, use dotstar
      * Y dac: play tone and test speaker/line out
      * Y cut pot: above but respond to cut adjust
      * Y res pot on adc: above but also response to res adjust
      * Y osc waveform selector: add pot to adc and test changing waveforms
      * Y midi input for note on/off (gate and pitch)
      * Y fix multiple notes held, etc
      * Y fix env
      * add env shaping over cc
      * Y determine default 303 env shape
      * Y add env knob
      * add drone at max env
      * Y add decay knob
      * add vel over cc
      * add acc knob
      * sub-osc: play constant sub-osc square -12
      * sub-osc waveform selector and level
      * add cc or knob to select lfo waveform, rate
      * add cc or knob to do lfo send intensity/amount to pitch/OSC, cut, res, trem/VCA
      * ...

per some random internet post:
There is an 'official' tuning for the VCF.
According to the Roland TB-303 manual, when playing C1 (65.4Hz) with the cutoff frequency knob set to 50%, saw waveform, and resonance knob at 100%, the resonance frequency should be 500Hz (give or take 100Hz depending on how it sounds to you).
You can perform this tuning by either applying 3.0VDC to the VCO or finishing the entire assembly and using something like Keyboard mode to trigger the note. "
*/
#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
// ~/Arduino/libraries/Mozzi/tables 
# define TABLES_SIZE 2048
// #include <tables/cos2048_int8.h> // for the modulation oscillators
#include <tables/sin2048_int8.h> // sine table for oscillator
#include <tables/saw2048_int8.h> // saw table for oscillator
#include <tables/triangle2048_int8.h> // triangle table for oscillator
#include <tables/square_no_alias_2048_int8.h> // square table for oscillator
// #include <StateVariable.h>
#include <ResonantFilter.h>
#include <mozzi_rand.h> // for rand()
#include <mozzi_midi.h>
#include <ADSR.h>

/* adc defines */
#define RES_PIN 0
#define OSC0WAVT_PIN 1
#define DCY_PIN 2
#define ENVMOD_PIN 3
#define ACC_PIN 4
#define TBD_PIN 5
// 6
// 7
// will be redirected to on-board_adc or pin 4
#define CUT_PIN 8

/* midi defines */
// #define MIDI_CHANNEL MIDI_CHANNEL_OMNI  // means all channels
#define MIDI_CHANNEL 1


// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 64  // Hz, powers of 2 are most reliable, lower numbers are "better" bc less load on the system
#define CONTROL_SUBRATE 4  // how often you want to check controls that don't need to be checked everytime
int control_cnt = 0;


#define NUM_OSCILS 1
Oscil<TABLES_SIZE, AUDIO_RATE> oscils[NUM_OSCILS]; // is this template right? should it be bit width and am I missing updates anywhere?
int oscils_note[NUM_OSCILS];
int oscils_freq[NUM_OSCILS];
bool oscils_playing[NUM_OSCILS];
int oscils_wavt[NUM_OSCILS];
int osc0_wavt_cnt = 4;  // count of the switch statement options


// use: Oscil <table_size, update_rate> oscilName (wavetable), look in .h file of table #included above
// Oscil <COS2048_NUM_CELLS, CONTROL_RATE> kFilterMod(COS2048_DATA);
//Oscil <, AUDIO_RATE> WAVEFORM0 = [
//  Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aSin(SIN2048_DATA),
//  Oscil <WHITENOISE8192_NUM_CELLS, AUDIO_RATE> aNoise(WHITENOISE8192_DATA)  // noise always at end
//];

LowPassFilter lpf;  // can be changed to HighPassFilter, BandPassFilter or NotchFilter
// cut 0-255 to represent 0-8192 Hz
// res 0-255, with 255 as max res

// StateVariable <LOWPASS> svf; // can be LOWPASS, BANDPASS, HIGHPASS or NOTCH
// svf freq range is 20 Hz to AUDIO_RATE/4 (32k/4 = 8192)
#define CUT_MIN 0
const int CUT_MAX = 255;
int cutoff = CUT_MAX;
#define RES_MIN 0
const int RES_MAX = 255;
int resonance = RES_MIN;


// with accent off, these are the time for exponential decay to 10%
// accent on it is always 200, with off this is the min
#define DCY_MIN 200
// with accent off
#define DCY_MAX 2500
int accent = 0;
int env_mod = 127;
int decay = 0;
int tbd = 0;


// envelope generator
// ADSR <CONTROL_RATE, AUDIO_RATE> venv;
// <x, y> where x is how often update() will be called and y is how often next()
// so for this, put update into controlHool and next in audioHook
ADSR <CONTROL_RATE, AUDIO_RATE> venv[NUM_OSCILS];
ADSR <CONTROL_RATE, CONTROL_RATE> fenv[NUM_OSCILS];
// for now normal is 62.5%, accent base is 80%
#define LVL_MIN 0
// #define LVL_NORM 160
// #define LVL_ACC 208
#define LVL_NORM 208
#define LVL_MAX 255
bool accent_on = false;
int accent_level = LVL_NORM;


#define DEBUG 1
void debug_setup () {
  /* if debug off, we don't need serial */
  if (!DEBUG) {
    return;
  }
  Serial.begin(115200);
  Serial.println("Hello world");
}


void setup () {
  debug_setup();
  adc_setup();
  dotstar_setup();
  midi_setup();
  mozzi_setup();
}


void loop(){
  audioHook(); // required here
  rainbowHook();
  midiHook();
}


void mozzi_setup () {
  oscils_note[0] = 81;  // A4
  oscils_freq[0] = 440;  // could also use (int)mtof
  oscils_playing[0] = false;
  oscils_wavt[0] = 0;
  set_wavetables();
  oscils[0].setFreq(oscils_freq[0]);
  // svf.setResonance(resonance);
  // svf.setCentreFreq(cutoff);
  lpf.setCutoffFreqAndResonance(cutoff, resonance);
  venv[0].setADLevels(accent_level, 0);  // att, dcy; 0-255. 
  //venv[0].setTimes(50,200,10000,200); // testing long note
  venv[0].setTimes(3, 10000, 0, 0); // 303 VENV is constant, accent changes levels
  fenv[0].setADLevels(255, 0);  // 303 FENV level changed by env mod, or if accent, then fn of accent, res, and env mod
  fenv[0].setTimes(3, 2500, 0, 0); // 303 FENV delay changes with knob, levels change as function of env mod or accent, res
  startMozzi(CONTROL_RATE); // :)
}


void set_wavetables () {
  // for each osc, call set wavetable, which will look at global oscils_wavt */
  for (int i = 0; i < NUM_OSCILS; i++) {
    set_wavetable(i);
  }
}


void set_wavetable (int oscil_idx) {
  /* look at global storage of what wavetable this osc should use, map it to the actual lib table, and set it */
  int wavt = oscils_wavt[oscil_idx];
  // consider oscil specific order of tables (ex: LFO vs OSC)
  switch (wavt) {
    case 1:
      oscils[oscil_idx].setTable(SQUARE_NO_ALIAS_2048_DATA);
      break;
    case 2:
      oscils[oscil_idx].setTable(TRIANGLE2048_DATA);
      break;
    case 3:
      oscils[oscil_idx].setTable(SIN2048_DATA);
      break;
    default: // case 0
      oscils[oscil_idx].setTable(SAW2048_DATA);
  }
}


void note_change (int osc_idx, int note) {
  /* the note changed, update globals, convert to freq, and set the osc */
  if (note == oscils_note[osc_idx]) {
    return;
  }
  int freq = (int)mtof(note);
  if (DEBUG) { Serial.print("Freq "); Serial.print(oscils_freq[osc_idx]); Serial.print(" -> "); Serial.println(freq); }
  oscils_freq[osc_idx] = freq;
  oscils_note[osc_idx] = note;
  oscils[osc_idx].setFreq(oscils_freq[osc_idx]);  // +- bend
  // this would be where the subosc offset could be calc'ed and set
  // note_change(1, note - suboscoffset, false);
}


bool trigger_env (int osc_idx) {
  /* trigger both envelopes, probably a new note (not a glide) happened */
    // if venv is in ADS, then let it finish
  if (oscils_playing[osc_idx]) {
    // if there was a freq change in the caller, this will mean the orig/existing env will just continue
    // TODO check the status of the venv (always longer than fenv) and if idle/complete
    //      then call stop_env and then trigger the env
    return false;
  }
  if (DEBUG) { Serial.println("Triggering ENV ADSR"); }  
  oscils_playing[osc_idx] = true;
  venv[osc_idx].noteOn();
  fenv[osc_idx].noteOn();
  return true;
}


void stop_env (int osc_idx) {
  /* when no note pressed, switch to rel. with 0 sustain you get here if decay not finished, 303 rel is 0 anyways */
  if (DEBUG) { Serial.println("Triggering VENV Rel (is currently playing but stopped)"); }
  // if (venv[0].playing()) {
  oscils_playing[osc_idx] = false;
  venv[osc_idx].noteOff();
  fenv[osc_idx].noteOff();
}


/* 
import numpy as np                                                                                                                                                                  
y = np.linspace(0,255,256)
y = (np.e**(np.log(255)/255))**y 
y.round(0)
 */
const int lin_to_exp[256] = {
         0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,  //  16
         1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,  //  32
         2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,  //  48
         3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   4,  //  64
         4,   4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   5,   5,   5,   5,   6,  //  80
         6,   6,   6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   8,   8,   8,  //  96
         8,   8,   8,   9,   9,   9,   9,   9,  10,  10,  10,  10,  10,  11,  11,  11,  // 112
        11,  12,  12,  12,  12,  13,  13,  13,  14,  14,  14,  14,  15,  15,  15,  16,  // 128
        16,  16,  17,  17,  18,  18,  18,  19,  19,  20,  20,  21,  21,  21,  22,  22,  // 144
        23,  23,  24,  24,  25,  25,  26,  27,  27,  28,  28,  29,  30,  30,  31,  32,  // 160  // [50] == 26 == 10% point
        32,  33,  34,  35,  35,  36,  37,  38,  39,  39,  40,  41,  42,  43,  44,  45,  // 176
        46,  47,  48,  49,  50,  51,  52,  53,  55,  56,  57,  58,  59,  61,  62,  63,  // 192
        65,  66,  68,  69,  71,  72,  74,  76,  77,  79,  81,  82,  84,  86,  88,  90,  // 208
        92,  94,  96,  98, 100, 102, 105, 107, 109, 112, 114, 117, 119, 122, 124, 127,  // 224
       130, 133, 136, 139, 142, 145, 148, 151, 155, 158, 162, 165, 169, 172, 176, 180,  // 240
       184, 188, 192, 196, 201, 205, 210, 214, 219, 224, 229, 234, 239, 244, 250, 255   // 256
};


void updateControl () {
  /* Mozzi calls this every CONTROL_RATE, keep as fast as possible as it will hold up AUDIO_RATE calls
   *  for wave forms (like oscil, env shapes), call .update() per CONTROL_RATE and .next() per AUDIO_RATE in audioHook()
   *    update calcs the actual value of that wave
   *    next extrapolates between actual and next
   *  aka this is where you read knobs and set/update any numbers extrapolated/used within audioHook
   */
  // bool update_lpf = false;
  control_cnt++;
  int tbd_val = 0;
  int waveform = 0;
  int dcy_val = 0;
  int dcy_ms = DCY_MIN;
  if (control_cnt > CONTROL_SUBRATE) {
    // these are controls that don't need to be responsive, reduce the I2C waits
    control_cnt = 0;
    tbd_val = adc_read(TBD_PIN);
    tbd_val = map(tbd_val, 0, 255, 0, 8);  
    tbd_val = constrain(tbd_val, 0, 7); 
    byte notes[] = { 48, 50, 51, 53, 55, 56, 58, 60 };
    if (tbd_val != tbd) {
      // for testing, this is c harmonic minor
      HandleNoteOff(MIDI_CHANNEL, notes[tbd], LVL_NORM);
      // for testing alternate accent
      accent_on = !accent_on;
      if (DEBUG) { Serial.print("Tbd "); Serial.print(tbd); Serial.print(" -> "); Serial.print(tbd_val); Serial.print(" a? "); Serial.println(accent_on); } 
      tbd = tbd_val;
      HandleNoteOn(MIDI_CHANNEL, notes[tbd_val], LVL_NORM);
    }
    else if (venv[0].playing()) {
      HandleNoteOff(MIDI_CHANNEL, notes[tbd], LVL_NORM);      
    }
    // freq = fn(note, tuning offset, glide, midi pitch bend)
    // wave = ratio of knob to number of options
    waveform = adc_read(OSC0WAVT_PIN);
    waveform = map(waveform, 0, 255, 0, osc0_wavt_cnt);  
    waveform = constrain(waveform, 0, osc0_wavt_cnt - 1); // else 255 is osc0_wavt_cnt which would be an index error
    if (waveform != oscils_wavt[0]) {
      if (DEBUG) { Serial.print("Osc0-wave "); Serial.print(oscils_wavt[0]); Serial.print(" -> "); Serial.println(waveform); }
      oscils_wavt[0] = waveform;
      set_wavetable(0);
    }
    // if !accent_on: fenv(atk=3 msec, dcy=200 msec +(2500-200)*ratio of knob)
    // if  accent_on: fenv(atk=3 msec, dcy=200 msec)
    if (!accent_on) {
      dcy_val = adc_read(DCY_PIN);
      dcy_ms = map(dcy_val, 0, 255, DCY_MIN, DCY_MAX);
    }
    if (dcy_ms != decay) {
      if (DEBUG) { Serial.print("Dcy "); Serial.print(decay); Serial.print(" -> "); Serial.println(dcy_ms); }
      decay = dcy_ms;    
      // see if you don't need to set setADLevel based on accent if you don't need to, use the boost ratio in control
      fenv[0].setTimes(3, decay, 0, 0);
    }
  }
  // acc = accent_on ? ratio of knob : 0
  // accent_on affects others
  //   * fenv = fenv(dcy=.2)  // dcy looses effect; per schem
  //   * cut = cut - env_mod_bias + fenv*env_mod% + smooth_via_c13(res, fenv*acc%)
  //   * venv = venv + acc%*(LVL_MAX-LVL_NORM)
  int acc = adc_read(ACC_PIN);
  if (acc != accent) {
    if (DEBUG) { Serial.print("Acc "); Serial.print(accent); Serial.print(" -> "); Serial.println(acc); }
    accent = acc;
    // update_lpf = true;
  }
  // res = ratio of knob
  //   if accent_on, res has an effect on fenv
  //     the higher it is, the smoother the curve (more voltage from Accent knob availble to charge C13 in schem)
  //     the lower it is, the accented decay cv is summed with the decay cv that went through the env mod
  //   on the 303 it is a double/stacked pot, 1 controls res, 2 compresses/smooths acc
  //   see https://www.firstpr.com.au/rwi/dfish/303-unique.html
  //   called the "Accent Sweep Circuit" and the smoother curve adds a wah/wow to the note
  int res = adc_read(RES_PIN);
  //res = map(res, 0, 255, RES_MIN, RES_MAX);
  // if (DEBUG) { Serial.println(res); }
  if (res != resonance) {
    if (DEBUG) { Serial.print("Res "); Serial.print(resonance); Serial.print(" -> "); Serial.println(res); }
    resonance = res;
    // update_lpf = true;
  }
  // env_mod = ratio of knob
  //   env_mod has an effect on fenv and cut
  //   called the gimmick circuit in manual, increasing env_mod directly reduces cut
  //   this makes the center of the sweep the cut off keeping more in performance range
  //   env_mod directly reduces fenv, and is summed with the accent fenv that comes through res c9
  // int env = ctrl_env_mod();  // nothing to directly set, only reads, ret 0..255
  int env = adc_read(ENVMOD_PIN);
  // env = map(env, 0, 255, x_MIN, x_MAX);
  if (env != env_mod) {
    if (DEBUG) { Serial.print("Env "); Serial.print(env_mod); Serial.print(" -> "); Serial.println(env); }
    env_mod = env;
    // update_lpf = true;
  }
  // if !accent_on: cut = cut + fn(fenv(dcy=knob)*env_mod%)
  // if  accent_on: cut = cut + smooth_via_c13(res, fenv*acc%)
  //                res 0% is fenv/acc. res 100% smooth(fenv*acc%)
  //                see https://www.firstpr.com.au/rwi/dfish/303-unique.html
  int cut_value = lin_to_exp[adc_read(CUT_PIN)];  // convert to exp (less change per step at lower values; more at higher; for more intuitive knob turning)
  // int cut_freq = map(cut_value, 0, 255, CUT_MIN, CUT_MAX);
  // mozziAnalogRead value is 0-1023 AVR, 0-4095 on STM32; set with analogReadResolution in setup
  // note that this has been reduced to 8b until testing complete, so cut_value=0..255
  if (cut_value != cutoff) {
    if (DEBUG) { Serial.print("Cut "); Serial.print(cutoff); Serial.print(" -> "); Serial.println(cut_value); }
    cutoff = cut_value;
    // update_lpf = true;
  }
  // if anything is playing...
  // now we adjust the cut
  // for now, just give it the spike to know it works
  fenv[0].update();  // does this need to happen every ctrl or can it be skipped if nothing is playing?
  int fenv_level = (fenv[0].next() * env_mod) >> 8;  // use env_mod as a % on fenv and preserve 8b
  fenv_level = lin_to_exp[fenv_level];  // scale to be exponential decay
  // make a temp cut reduced by env_mod
  // find the headroom above the temp cut, then use fenv as a % against that
  fenv_level = ((CUT_MAX - cutoff) * fenv_level) >> 8;  // and preserve 8b
  // avoid aliasing due to integer overflow, this is like hard clipping, could benefit from compression-like alg, or assurances against int overflow
  int tmp_cutoff = constrain(cutoff + fenv_level, 0, 255);
  // if update_lpf { and if nothing is playing then don't do the cutoff calcs
  // might as well call this if anything changes, so there isn't the risk of a cut/res jump if a ctrl lags after a noteOn
  lpf.setCutoffFreqAndResonance(tmp_cutoff, resonance);
  // float venv_accent_boost = acc_pot_to_gain_val[acc];  // ret 1..(LVL_MAX/LVL_NORM)
  // venv(atk=3 msec, dcy=5000) * acc%*(LVL_MAX/LVL_NORM)
  // in audio: venv.next()*venv_accent_boost
  //           scale_8b_by_perc(venv.next(), venv_accent_boost)
  venv[0].update();  // does this need to happen every ctrl or can it be skipped if nothing is playing?
}


AudioOutput_t updateAudio () {
  /* Mozzi calls this every AUDIO_RATE, keep fast, returns 8b int, centered around 0, that gets written to a DAC
   *  DAC will output 0v - 3.3v, so "centered around 0" means centered around 1.15v
   *  put DAC through a cap to remove DC and have 1.15 peak VAC. Put that through a voltage divider to reach your preferred line level.
   *  
   *  can access all the controls from updateControl, but ideally those are only set once per CONTROL_RATE, so do them there
   *  for wave forms (like oscil, env shapes), call .update() per CONTROL_RATE in updateControl() and .next() here
   *    update calcs the actual value of that wave
   *    next extrapolates between actual and next
   *  
   * SAMD21 also in Feather M0
   *  
   *  updateAudio should return roughly 9b, with range of 488: -244 to 243
   *  https://sensorium.github.io/Mozzi/doc/html/_mozzi_guts_8h_source.html
   *  STANDARD_PLUS is default; nearly 9 bit sound (-244 to 243) at 16384 Hz sample rate (AUDIO_RATE)
   *  However: https://sensorium.github.io/Mozzi/doc/html/_audio_config_s_a_m_d21_8h_source.html
   *   and https://github.com/sensorium/Mozzi/issues/75
   *   #define AUDIO_BITS 10 "make sure updateAudio returns values from -512 to 511
   *   #define AUDIO_BIAS ((uint16_t) 1 << (AUDIO_BITS - 1)) aka 512
   *     
   *  venv[0].next returns 0..255, Q8n0 is same as unsigned char (uint8_t)
   *   https://sensorium.github.io/Mozzi/doc/html/_a_d_s_r_8h_source.html#l00165
   * 
   *  osc.next reads the wave table, which is int8_t (signed 8b int) -128-127
   *  https://sensorium.github.io/Mozzi/doc/html/_audio_output_8h_source.html ::46
   *   #define AudioOutputStorage_t int
   *  lpf reads osc multiplies feedback. also see note on line 154 about overflow https://sensorium.github.io/Mozzi/doc/html/_resonant_filter_8h_source.html
   *    but that should only be by 1b: TODO calc max buf0/buf1 with f=1, fb=1, in={-128,127}
   *    buf0 = buf0 + f * (in - buf0 + fb * (buf0 - buf1));
   *    buf1 = buf1 + f * (buf0 - buf1)
   *      127,127 = 127+(127-127) = 127
   *      -128,-128 = -128+(-128+128)=-128
   *      127,-128 = -128+(127+128)=127
   *      -128,127 = 127+(-128-127)=-128
   *       out = buf1; // LowPass
   *       out = in - buf0; // HighPass
   *       out = buf0 - buf1; // BandPass
   *       out = in - buf0 + buf1; // Notch
   */
  int32_t vca_exp = lin_to_exp[venv[0].next()];  // 0..255 -> exp(0..255)
  // TODO boost if accent_on
  // int8_t osc.next -128-127
  // int9   lpf.next -256-255  // TODO verify
  int32_t audio_out = lpf.next(oscils[0].next());  // AudioOutputStorage_t is an int
  // trusting a comment in MultiResonantFilter example::86 to allow 1 bit for resonance
  // filtered_osc should be <=9b
  // now scale for vca, use x*y>>8 which will need 8+9=17b
  audio_out = (vca_exp * audio_out) >> 8;
  if (audio_out < -512) {  // (-(AudioOutputStorage_t) AUDIO_BIAS)
    if (DEBUG) { Serial.print("Clipping - "); Serial.println(audio_out); }
    audio_out = -512;
  }
  else if (audio_out > 511) {  // (AudioOutputStorage_t) AUDIO_BIAS-1
    if (DEBUG) { Serial.print("Clipping + "); Serial.println(audio_out); }
    audio_out = 511;
  }
  // For reasons, allow 1 bit of headroom to bring us to 10 bits, which is perfect for the SAMD21 DAC
  // a bit of a hack, but let's clip this thing at 10b. see ::55 at https://sensorium.github.io/Mozzi/doc/html/_audio_output_8h_source.html
  // #define CLIP_AUDIO(x) constrain((x), (-(AudioOutputStorage_t) AUDIO_BIAS), (AudioOutputStorage_t) AUDIO_BIAS-1)
  return MonoOutput::fromNBit(10, audio_out);
}
