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
      *   A2 env mod
      *   A3 decay
      *   A4 accent
      *   A5 trig (temporary for not needing keyboard)
      *   A6 TBD
      *   A7 TBD
   
    TODO
    * adc_expontential and apply to cut
    * send i2c req for value after read to prep for next read
    * swap out filter for something better
    * add max/min for filter and res
    * if res is still reverse (1 max) then switch to adc_lin_inv
    * add Serial.read to send notes over TTY and not need keyboard for testing
    * exp_mappings, and use for cut
    * SAMD21 ADC bug
    * zombie osc_playing

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
      * add env knob
      * add drone at max env
      * add decay knob
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
#include <tables/cos2048_int8.h> // for the modulation oscillators
#include <tables/sin2048_int8.h> // sine table for oscillator
#include <tables/saw2048_int8.h> // saw table for oscillator
#include <tables/triangle2048_int8.h> // triangle table for oscillator
#include <tables/square_no_alias_2048_int8.h> // square table for oscillator
#include <StateVariable.h>
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
#define CONTROL_RATE 64  // Hz, powers of 2 are most reliable
#define CONTROL_SUBRATE 4  // how often 
int control_cnt = 0;


#define NUM_OSCILS 1
Oscil<COS2048_NUM_CELLS, AUDIO_RATE> oscils[NUM_OSCILS];
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

// LowPassFilter lpf;  // can be changed to HighPassFilter, BandPassFilter or NotchFilter

StateVariable <LOWPASS> svf; // can be LOWPASS, BANDPASS, HIGHPASS or NOTCH
// svf freq range is 20 Hz to AUDIO_RATE/4 (32k/4 = 8192)
#define CUT_MIN 20
const int CUT_MAX = AUDIO_RATE >> 2;  // AUDIO_RATE/4;
// const int cut_max = 20000;  // max moog filter is 20k, prev 16k, what is 303?
int cutoff = CUT_MAX;
#define RES_MIN 1  // min value, but maximum resonance
const int RES_MAX = 255;  // 180; // observed, but technically 255, max value, but minimum res
int resonance = 1; //RES_MAX;


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


#define DEBUG 0
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
  svf.setResonance(resonance);
  svf.setCentreFreq(cutoff);
  venv[0].setADLevels(accent_level, 0);  // att, dcy; 0-255. 
  //venv[0].setTimes(50,200,10000,200); // testing long note
  venv[0].setTimes(3, 10000, 0, 0); // 303 VENV is constant, accent changes levels
  fenv[0].setADLevels(LVL_NORM, 0);
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


void updateControl () {
  /* Mozzi calls this every CONTROL_RATE, keep as fast as possible as it will hold up AUDIO_RATE calls
   *  for wave forms (like oscil, env shapes), call .update() per CONTROL_RATE and .next() per AUDIO_RATE in audioHook()
   *    update calcs the actual value of that wave
   *    next extrapolates between actual and next
   *  aka this is where you read knobs and set/update any numbers extrapolated/used within audioHook
   */
  control_cnt++;
  int tbd_val = 0;
  int waveform = 0;
  int dcy_val = 0;
  int dcy_ms = 200;
  if (control_cnt > CONTROL_SUBRATE) {
    // these are controls that don't need to be responsive, reduce the I2C waits
    control_cnt = 0;
    tbd_val = adc_read(TBD_PIN);
    tbd_val = map(tbd_val, 0, 255, 0, 8);  
    tbd_val = constrain(tbd_val, 0, 7); 
    if (tbd_val != tbd) {
      // for testing, this is c harmonic minor
      byte notes[] = { 48, 50, 51, 53, 55, 56, 58, 60 };
      if (DEBUG) { Serial.print("Tbd "); Serial.print(tbd); Serial.print(" -> "); Serial.println(tbd_val); } 
      HandleNoteOff(MIDI_CHANNEL, notes[tbd], LVL_NORM);
      tbd = tbd_val;
      HandleNoteOn(MIDI_CHANNEL, notes[tbd_val], LVL_NORM);
    }
    // freq = fn(note, tuning offset, glide)
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
    dcy_val = adc_read(DCY_PIN);
    if (!accent_on) {
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
  }
  // res = ratio of knob
  //   res has an effect on fenv, the higher it is, the smoother the curve (C9 in schem)
  //   on the 303 it is a double/stacked pot, 1 controls res, 2 compresses/smooths acc
  //   see https://www.firstpr.com.au/rwi/dfish/303-unique.html
  //   called the "Accent Sweep Circuit"
  int res = adc_read(RES_PIN);
  //res = map(res, 0, 255, RES_MIN, RES_MAX);
  // 1 is max res, 255 is min, so use inverted map
  //res = RES_MIN + 255 - res;
  res = constrain(res, RES_MIN, RES_MAX);  // 1..255per StateVariableFilter
  // TODO find reasonable range, prev notes indicate 180..1
  // lower res value means more resonance, valid values are 0-255
  // pot wired to return lower ADC when turned clockwise
  // if (DEBUG) { Serial.println(res); }
  if (res != resonance) {
    if (DEBUG) { Serial.print("Res "); Serial.print(resonance); Serial.print(" -> "); Serial.println(res); }
    resonance = res;
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
  }
  fenv[0].update();  // TODO try removing this to see if speed up happens
  // if !accent_on: cut = cut + fn(fenv(dcy=knob)*env_mod%)
  // if  accent_on: cut = cut + smooth_via_c13(res, fenv*acc%)
  //                res 0% is fenv/acc. res 100% smooth(fenv*acc%)
  //                see https://www.firstpr.com.au/rwi/dfish/303-unique.html
  // int cut = ctrl_cut();  // only reads, ret 20..8192
  int cut_value = adc_read(CUT_PIN);
  // TODO cut_value = adc_exponential[cut_value]; // convert to exp (less change per step at lower values; more at higher)
  int cut_freq = map(cut_value, 0, 255, CUT_MIN, CUT_MAX);
  // mozziAnalogRead value is 0-1023 AVR, 0-4095 on STM32; set with analogReadResolution in setup
  // note that this has been reduced to 8b until testing complete, so cut_value=0..255
  // int cut_value = mozziAnalogRead(CUT_PIN);
  // int cut_freq = cut_pot_to_val[cut_value];
  // pot wired to return higher ADC when turned clockwise
  // if (DEBUG) { Serial.print("Cut v "); Serial.println(cut_value); }
  // if (DEBUG) { Serial.print("Cut f "); Serial.println(cut_freq); }
  if (cut_freq != cutoff) {
    if (DEBUG) { Serial.print("Cut "); Serial.print(cutoff); Serial.print(" -> "); Serial.println(cut_freq); }
    cutoff = cut_freq;
  }
  int fenv_boost = 0;
  // use shifts instead of float/perc multipliers
//  if (false /*TODO*/ && env_mod && !accent_on) {
//    int fenv_level = fenv[0].next();
//    // float fenv_boost_perc = acc_pot_to_gain_val[env_mod];  // >= 1.0
//    float fenv_attenuate_perc = (float) fenv_level * ((float) env_mod / 255.0);  // 0..255
//    fenv_attenuate_perc = fenv_attenuate_perc / 255.0;  // 0..1
//    float gain = ((float)LVL_MAX - (float)LVL_NORM) / (float)LVL_NORM;  // >= 1.0
//    float attenuated_gain = 1.0 + ((gain - 1.0) * fenv_attenuate_perc);  // >= 1.0
//    fenv_boost = (int)((float)cut + ((float)(CUT_MAX - cut) * attenuated_gain));
//    // fenv_boost = calc_cut(cut, fenv[0], env_mod);
//  }
//  else if (accent_on) {
//    // TODO
//    // fenv_boost = calc_cut_acc(cut, fenv[0], acc, res);
//  }
  svf.setResonance(resonance);  // resonance is the global, res is local
  svf.setCentreFreq(cutoff + fenv_boost);  // cutoff is the global, cut_freq is local
  // seems like need to svf.update() here! but SVF doesn't have that member?!
  // float venv_accent_boost = acc_pot_to_gain_val[acc];  // ret 1..(LVL_MAX/LVL_NORM)
  // venv(atk=3 msec, dcy=5000) * acc%*(LVL_MAX/LVL_NORM)
  // in audio: venv.next()*venv_accent_boost
  //           scale_8b_by_perc(venv.next(), venv_accent_boost)
  venv[0].update();
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
   */
  // return MonoOutput::from8Bit(aSin.next()); // return an int signal centred around 0
  // 'from' means to convert from 'Nbit' to audio output bit width (hardware specific)
  // return MonoOutput::fromAlmostNBit(12, svf.next(oscils[0].next()));
  return MonoOutput::from16Bit((int) (venv[0].next() * svf.next(oscils[0].next())));  // why (int)?? why not better faster math?
}
