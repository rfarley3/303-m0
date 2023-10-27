#ifndef synth303_ino
#define synth303_ino
/*  Create a 303-ish synth with potentiometer controllers.
 *  (c) Ryan Farley rfarley3@github
 *  
 *  when you load the project in Arduino
 *  * install Mozzi, MIDI
 *  * open adc.ino, dotstar.ino, midi_in.ino, EnvADExp.h

    Mozzi documentation/API
    https://sensorium.github.io/Mozzi/doc/html/index.html
    https://sensorium.github.io/Mozzi/learn/under-the-hood/
    https://diyelectromusic.wordpress.com/2021/03/20/trinket-fm-synthesis-with-mozzi/
    https://diyelectromusic.wordpress.com/2021/02/15/midi-in-for-3-3v-microcontrollers/
    https://diyelectromusic.wordpress.com/2021/05/29/midi-connections-cheat-sheet/
    https://github.com/diyelectromusic/sdemp/blob/main/src/SDEMP/TrinketUSBMIDIMultiPotMozziSynth3/TrinketUSBMIDIMultiPotMozziSynth3.ino
    ^^ shows USB host mode

    303 Circuit and manual
    * http://machines.hyperreal.org/manufacturers/Roland/TB-303/schematics/roland.TB-303.schem-5.gif
    * https://www.firstpr.com.au/rwi/dfish/303-unique.html
    * http://machines.hyperreal.org/manufacturers/Roland/TB-303/schematics/roland.TB-303.schem-8.gif
    * https://olney.ai/ct-modular-book/tb-303.html
 
    Circuit with Adafruit Trinket M0
    * pin 1 (DAC 10b) audio output 
      * 1k Ohm with (1k Ohm voltage divider (halving 3.3 -> 1.625) & 10 uF coupling capacitor (biasing as AC += .8v))
      * pos of cap to jack tip
    * pin 0 to I2C SDA
    * pin 2 to I2C SCL
    * pin 3 (RX) from MIDI
    * pin 4 momentary NO switch for input
    * ADC (ADS7830 8 chan 8b) addr x4b
      * Y A0 cut
      * Y A1 res
      * Y A2 env_mod
      * Y A3 decay
      * Y A4 accent
      * Y A5 wave form
      *   A6 (temporary for not needing keyboard)
          *  Eventually something highly playable, like 1) FENV & VENV Attack, 2) Accented Decay % of unaccented decay, 3) Sub-OSC
      *   A7 ping-4 Button selectable, Candidates in README
*/
#include <MozziGuts.h>
#include <Oscil.h>
// ~/Arduino/libraries/Mozzi/tables 
# define TABLES_SIZE 2048
#include <tables/saw2048_int8.h> // saw table for oscillator
#include <tables/square_no_alias_2048_int8.h> // square table for oscillator
#include <ResonantFilter.h>
#include <mozzi_rand.h>
#include <mozzi_midi.h>
#include <Smooth.h>
#include "EnvADExp.h"  // copy of ~/Arduino/libraries/Mozzi/ADSR.h with exponential decay

/* adc defines */
#define CUT_PIN 5
#define RES_PIN 4
#define ENVMOD_PIN 3
#define DCY_PIN 6
#define ACC_PIN 7
#define OSC0WAVT_PIN 0
#define TESTING_PIN 1  // for testing, uses tbd and tbd_knob
#define SELECTABLE 2 // on-board pin 4 momentary button changes which variable and commiting values
#define MOMENTARY_PIN 4  // TODO double clicking this rotates through what a knob can set, hold to set that setting

/* midi defines */
// #define MIDI_CHANNEL MIDI_CHANNEL_OMNI  // means all channels
#define MIDI_CHANNEL 1


// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 64  // Hz, powers of 2 are most reliable, lower numbers are "better" bc less load on the system
#define CONTROL_SUBRATE 4  // how often you want to check controls that don't need to be checked everytime
int control_cnt = 0;


#define NUM_OSCILS 1
Oscil<TABLES_SIZE, AUDIO_RATE> oscils[NUM_OSCILS];
int oscils_note[NUM_OSCILS];
int oscils_freq[NUM_OSCILS];
bool oscils_playing[NUM_OSCILS];
int oscils_wavt[NUM_OSCILS];
int osc0_wavt_cnt = 2;  // TODO add low-passed sq/saw per https://olney.ai/ct-modular-book/tb-303.html


LowPassFilter lpf;  // same as StateVariable <LOWPASS>
ResonantFilter <HIGHPASS> hpf;
// cut 0-255 to represent freq range is 20 Hz to AUDIO_RATE/4 (32k/4 = 8192) Hz
// res 0-255, with 255 as max res
#define CUT_MIN 3
const int CUT_MAX = 255;
#define RES_MIN 0
const int RES_MAX = 255; // 240; // 255;
int FIXED_LOW_CUT = 2;  // of whatever 10 Hz may be for a res peak in the low end


int resonance = RES_MIN;
int cutoff = CUT_MAX;
int accent = 0;
int env_mod = 0;
// TODO env_mod divider so it's "anti-log"
int decay = 0;
int testing = 0;
int selectable = 0;


// envelope generator
// ADSR <CONTROL_RATE, AUDIO_RATE> venv;
// <x, y> where x is how often update() will be called and y is how often next()
// so for this, put update into controlHool and next in audioHook
EnvelopeExponentialDecay <AUDIO_RATE, AUDIO_RATE> venv[NUM_OSCILS];  // needs audio rate updates to declick atk/rel
EnvelopeExponentialDecay <CONTROL_RATE, CONTROL_RATE> fenv[NUM_OSCILS];
#define ATK_MSEC 3  // per manual for both VENV and FENV (VEG & MEG)
#define REL_MSEC 3  // declicker
// with accent off, these are the time for exponential decay to 10%
// accent on it is always 200, with off this is the min
#define DCY_FENV_MIN 200
// with accent off, decay can be between 200 and 2500 to reach 10%
#define DCY_FENV_MAX 2500
#define DCY_VENV 10000
// for now normal is 62.5%, accent base is 80%
#define LVL_MIN 0
#define LVL_NORM 127  // unaccented
#define LVL_MAX 255
bool accent_on = false;
int accent_level = LVL_NORM;  // see audioUpdate for true default value when !accent_on
#define VEL_TO_ACC_THRESHOLD 64
int ao_min = 0;
int ao_max = 0;
float smoothness_max = 0.9975f;
//bool smoothness_on = false;
Smooth <long> aSmoothGain(smoothness_max);


#define DEBUG 1
#define DEBUG_TBD_KNOB_NOTES 0  // turning TBD creates note events
#define DEBUG_TBD_KNOB_ENVM 0 // turning TBD modifies the rate env_mod changes cutoff and the fenv sweep range
#define DEBUG_NOTE_EVENTS 1 // print out note events
#define DEBUG_DISABLE_FENV 0  // same as disable LPF & HPF
#define DEBUG_DISABLE_LPF 0
#define DEBUG_DISABLE_HPF 1
#define DEBUG_DISABLE_VENV 0
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
  lpf.setCutoffFreqAndResonance(cutoff, resonance);
  if (!DEBUG_DISABLE_HPF) {
    hpf.setCutoffFreqAndResonance(FIXED_LOW_CUT, resonance);
  }
  // VENV
  venv[0].setADLevels(255, 0);//accent_level, 0);  // att, dcy; 0-255. 
  // FENV has 3 msec attack per manual, too clicky for VENV or the LERP alg
  // turns out it was on exp attack, so fixed that, but still too clicky
  // tried 0 (did not fix it), 3 (orig), 10 (better, but not for sine/tri), 100 (good for sine/triage, but very long)
  venv[0].setTimes(ATK_MSEC, DCY_VENV, REL_MSEC); // 303 VENV is constant, accent changes levels
  // FENV
  fenv[0].setADLevels(255, 0);  // 303 FENV level changed by env mod, or if accent, then fn of accent, res, and env mod
  fenv[0].setTimes(ATK_MSEC, DCY_FENV_MAX, REL_MSEC); // 303 FENV delay changes with knob, levels change as function of env mod or accent, res
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
    // TODO add HPF versions of sq and saw
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
  if (DEBUG && DEBUG_NOTE_EVENTS) { Serial.print("Freq "); Serial.print(oscils_freq[osc_idx]); Serial.print(" -> "); Serial.println(freq); }
  oscils_freq[osc_idx] = freq;
  oscils_note[osc_idx] = note;
  oscils[osc_idx].setFreq(oscils_freq[osc_idx]);  // +- bend
  // this would be where the subosc offset could be calc'ed and set
  // note_change(1, note - suboscoffset, false);
  // this is where keyboard tracking would be modify cutoff freq
}


bool gate_on (int osc_idx, int velocity) {
  /* trigger both envelopes, probably a new note (not a glide) happened */
    // if venv is in ADS, then let it finish
  if (oscils_playing[osc_idx]) {
    // if there was a freq change in the caller, this will mean the orig/existing env will just continue
    // TODO check the status of the venv (always longer than fenv) and if idle/complete
    //      then call gate_off and then trigger the env
    return false;
  }
  if (velocity >= VEL_TO_ACC_THRESHOLD) {
    accent_on = true;
  }
  else {
    accent_on = false;
  }
  if (DEBUG && DEBUG_NOTE_EVENTS) { Serial.print("Gate on. a? "); Serial.println(accent_on); }
  oscils_playing[osc_idx] = true;
  venv[osc_idx].noteOn();
  fenv[osc_idx].noteOn();
  return true;
}


void gate_off (int osc_idx) {
  /* when no note pressed, switch to rel. with 0 sustain you get here if decay not finished, 303 rel is 0 anyways */
  // if (DEBUG && DEBUG_NOTE_EVENTS) { Serial.println("Triggering VENV Rel (is currently playing but stopped)"); }
  // if (venv[0].playing()) {
  oscils_playing[osc_idx] = false;
  venv[osc_idx].noteOff();
  fenv[osc_idx].noteOff();  // TODO should something be done here to continue the FENV value as base into next noteOn if still on? (like don't release)
}


/* 
import numpy as np                                                                                                                                                                  
y = np.linspace(0,255,256)
y = (np.e**(np.log(255)/255))**y 
y.round(0)

Exponentially increasing look up table
         *
         *
        **
      ***
    ***
*****
log(x) = 255-lin_to_exp[255-x]
     *****
   ***
 ***
**
*
*
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


int lin_to_log(int lin_val) {
  // assumes lin is 0..255
  int log_val = 255 - lin_to_exp[255 - lin_val];
  return log_val;
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
  int dcy_ms = DCY_FENV_MIN;
  if (control_cnt > CONTROL_SUBRATE) {
    // these are controls that don't need to be responsive, reduce the I2C waits
    control_cnt = 0;
    int testing_val = adc_read(TESTING_PIN);
    // testing_val = map(testing_val, 0, 255, 0, 255);
    // testing_val = constrain(testing_val, 0, 255);
    // if (testing_val != testing) {
    //   testing = testing_val;
    // }
    if (DEBUG_TBD_KNOB_NOTES) {
      //tbd_val = adc_read(TBD_PIN);
      testing_val = map(testing_val, 0, 255, 0, 8);  
      testing_val = constrain(testing_val, 0, 7); 
      byte notes[] = { 48, 50, 51, 53, 55, 56, 58, 60 };
      if (testing_val != testing) {
        // for testing, this is c harmonic minor
        int velocity = 40;
        // for testing accent, even ones are accented
        if (notes[testing_val] % 2 == 0) {
          velocity = 127;
        }
        HandleNoteOff(MIDI_CHANNEL, notes[testing], velocity);
        HandleNoteOn(MIDI_CHANNEL, notes[testing_val], velocity);
        if (DEBUG) { Serial.print("Tbd "); Serial.print(testing_val); Serial.print(" -> "); Serial.print(testing_val); Serial.print(" a? "); Serial.println(accent_on); } 
        testing = testing_val;
      }
    }
    else if (DEBUG_TBD_KNOB_ENVM) {
      if (testing_val != testing) {
        // TODO change the dividing factor for envm to cut
        testing = testing_val;
      }
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
    // Y if !accent_on: fenv(atk=3 msec, dcy=200 msec +(2500-200)*ratio of knob)
    // Y if  accent_on: fenv(atk=3 msec, dcy=200 msec)
    if (!accent_on) {
      dcy_val = lin_to_log(adc_read(DCY_PIN));
      dcy_ms = map(dcy_val, 0, 255, DCY_FENV_MIN, DCY_FENV_MAX);
    }
    if (dcy_ms != decay) {
      if (DEBUG) { Serial.print("Dcy "); Serial.print(decay); Serial.print(" -> "); Serial.println(dcy_ms); }
      decay = dcy_ms;    
      // see if you don't need to set setADLevel based on accent if you don't need to, use the boost ratio in control
      fenv[0].setTimes(ATK_MSEC, decay, REL_MSEC);
    }
  }  // end if (control_cnt > CONTROL_SUBRATE)
  
  // The following are checked every CONTROL_RATE
  // acc = accent_on ? ratio of knob : 0
  // accent_on affects others
  //   * Y fenv = fenv(dcy=.2)  // dcy looses effect; per schem
  //   * cut = cut - env_mod_bias + fenv*env_mod% + smooth_via_c13(res, fenv*acc%)
  //   * Y venv = venv + acc%*(LVL_MAX-LVL_NORM)
  int acc = adc_read(ACC_PIN);
  if (acc != accent) {
    if (DEBUG) { Serial.print("Acc "); Serial.print(accent); Serial.print(" -> "); Serial.println(acc); }
    accent = acc;
    // TODO mod fenv curve
    // mod venv amount
  }
  accent_level = LVL_NORM + (((LVL_MAX - LVL_NORM) * accent) >> 8);
  // res = ratio of knob
  //   if accent_on, res has an effect on fenv
  //     the higher it is, the smoother the curve (more voltage from Accent knob availble to charge C13 in schem)
  //     the lower it is, the accented decay cv is summed with the decay cv that went through the env mod
  //   on the 303 it is a double/stacked pot, 1 controls res, 2 compresses/smooths acc
  //   see https://www.firstpr.com.au/rwi/dfish/303-unique.html
  //   called the "Accent Sweep Circuit" or "Wow Circuit" and the smoother curve adds a wah/wow to the note
  int res = adc_read(RES_PIN);
  res = map(res, 0, 255, RES_MIN, RES_MAX);
  // if (DEBUG) { Serial.println(res); }
  if (res != resonance) {
    if (DEBUG) { Serial.print("Res "); Serial.print(resonance); Serial.print(" -> "); Serial.println(res); }
    resonance = res;
    float smoothness_a = float(resonance)/256.0;
    if (smoothness_a > smoothness_max) {
      smoothness_a = smoothness_max;
    }
    aSmoothGain.setSmoothness(smoothness_a);
  }
  // env_mod = ratio of knob
  //   env_mod has an effect on fenv and cut
  //   called the gimmick circuit in manual, increasing env_mod directly reduces cut
  //   this makes the center of the sweep the cut off keeping more in performance range
  //   env_mod directly reduces fenv, and is summed with the accent fenv that comes through res c13
  int env = adc_read(ENVMOD_PIN);
  // env = map(env, 0, 255, ENVMOD_MIN, ENVMOD_MAX);
  if (env != env_mod) {
    if (DEBUG) { Serial.print("Env "); Serial.print(env_mod); Serial.print(" -> "); Serial.println(env); }
    env_mod = env;
  }
  // if !accent_on: cut = cut + fn(fenv(dcy=knob)*env_mod%)
  
  // if  accent_on: cut = cut + smooth_via_c13(res, fenv*acc%)
  //                res 0% is fenv/acc. res 100% smooth(fenv*acc%)
  //                see https://www.firstpr.com.au/rwi/dfish/303-unique.html
  //  HERE     * Make formula to make a duplicate (aka dup-fenv, accented fenv) of fenv (primary, from env_mod)
  //            * Reduced by accent knob
  //            * Constant value reduction from a diode
  //            * Smooth it more as res increases
  //   if accent_on, res has an effect on fenv
  //     the higher it is, the smoother the curve (more voltage from Accent knob availble to charge C13 in schem)
  //     the lower it is, the accented decay cv is summed with the decay cv that went through the env mod
  //   this makes the center of the sweep the cut off keeping more in performance range
  //   env_mod directly reduces fenv, and is summed with the accent fenv that comes through res c13

  // int cut_value = lin_to_exp[adc_read(CUT_PIN)];  // convert to exp (less change per step at lower values; more at higher; for more intuitive knob turning)
  int cut_value = adc_read(CUT_PIN);
  cut_value = map(cut_value, 0, 255, CUT_MIN, CUT_MAX);
  // mozziAnalogRead value is 0-1023 AVR, 0-4095 on STM32; set with analogReadResolution in setup
  // note that this has been reduced to 8b until testing complete, so cut_value=0..255
  if (cut_value != cutoff) {
    if (DEBUG) { Serial.print("Cut "); Serial.print(cutoff); Serial.print(" -> "); Serial.println(cut_value); }
    cutoff = cut_value;
  }
  // now we adjust the cut
  fenv[0].update();  // consider moving to audio_rate if alg is fast enough
  int fenv_level = fenv[0].next();
  int tmp_cutoff = calc_cutoff(fenv_level);
  lpf.setCutoffFreqAndResonance(tmp_cutoff, resonance);
  if (!DEBUG_DISABLE_HPF) {
    hpf.setCutoffFreqAndResonance(FIXED_LOW_CUT, resonance);
  }
}

int calc_cutoff(int fenv_level) {
  int antilog_reduction = 2;  // /4 to /8 comes from the manual visual referenc; make adjustable using TESTING knob, 1-6
  // Uses globals cutoff, env_mod (eventually accent_on, accent, resonance)
  int tmp_env_mod = env_mod >> antilog_reduction; 
  int tmp_fenv_range = 2 * tmp_env_mod;
  int tmp_cutoff = cutoff - tmp_env_mod;
  // TODO replace these comments with ones in pynb
  // TODO the way that env_mod reduces cutoff is "anti-log" but the knob is log, so it may not be a simple 1:1
  // for now, just use the linear change reduced by divisor

  // fenv_level is 0..255 along time domain for decay msec
  // NOTE currenctly fenv_level is a exponential decay
  // But cut bias (q9/q10) are anti-log resp and linear decay could hand wave these without complicated math
  
  // use it as a percent fenv_level/256 to scale tmp_fenv_range and add to tmp_cutoff
  // let this go below 0 and above 255, and hard clip before we exit this function
  int fenv_boost = ((fenv_level * tmp_fenv_range) >> 8);
  tmp_cutoff = tmp_cutoff + fenv_boost;  // unaccented cutoff from fenv plus the gimmick down-justed cutoff

  // ?? where should this comment go: to avoid some distortion it may be worth reducing resonance to ~240 when cutoff < 20
  // make sure you hear effect when env_mod = 0
  if (accent_on) {
    // tmp_cutoff is the unaccented cutoff
    // create an accent to add to unaccented
    // this is FENV * accent% - diode
    int tmp_accent = (fenv_level * accent) >> 8;
    // assume 6v bus and 0.6v fwd voltage drop through diode, so out of diode is 90%
    // subtract 10% of 255
    tmp_accent = tmp_accent - 26;
    if (tmp_accent < 0) {
      tmp_accent = 0;
    }
    tmp_accent = tmp_accent >> antilog_reduction; // to emulate antilog response
    // smooth this using resonance% as the smooth factor
    tmp_accent = aSmoothGain.next(tmp_accent);
    // sum the voltages
    tmp_cutoff = tmp_cutoff + tmp_accent;
  }
  
  // avoid aliasing due to integer overflow, this is like hard clipping, could benefit from compression-like alg, or assurances against int overflow
  tmp_cutoff = constrain(tmp_cutoff, CUT_MIN, CUT_MAX);
  return tmp_cutoff;
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
  // int8_t osc.next -128-127
  // int9   lpf.next -256-255  // TODO verify
  int32_t audio_out = oscils[0].next();  // AudioOutputStorage_t is an int
  if (!DEBUG_DISABLE_FENV) {
    if (!DEBUG_DISABLE_LPF) {
      audio_out = lpf.next(audio_out);
    }
    if (!DEBUG_DISABLE_HPF) {
      audio_out = hpf.next(audio_out);
    }
  }
  // Do we need a soft_clip? or general check that <24b before proceeding?
  // trusting a comment in MultiResonantFilter example::86 to allow 1 bit for resonance
  // filtered_osc should be <=9b
  // now scale for vca, use x*y>>8 which will need 8+9=17b
  venv[0].update();
  int32_t vca_exp = venv[0].next();
  if (!DEBUG_DISABLE_VENV) {
    int accent_mod = LVL_NORM;
    if (accent_on) {
      accent_mod = accent_level;
    }
    // adjust the VENV according to output level, which is NORM + !accent?0:accent*(MAX-NORM), set in control
    vca_exp = (vca_exp * accent_mod) >> 8;
    audio_out = (vca_exp * audio_out) >> 8;
  }
  audio_out = soft_clip(audio_out);
  // For reasons, allow 1 bit of headroom to bring us to 10 bits, which is perfect for the SAMD21 DAC
  // a bit of a hack, but let's clip this thing at 10b. see ::55 at https://sensorium.github.io/Mozzi/doc/html/_audio_output_8h_source.html
  // #define CLIP_AUDIO(x) constrain((x), (-(AudioOutputStorage_t) AUDIO_BIAS), (AudioOutputStorage_t) AUDIO_BIAS-1)
  return MonoOutput::fromNBit(10, audio_out);
}


int soft_clip(int sample) {
  /* use tube-ish compression and reserve last 10% for log(x bit of overflow) */
  // AudioOutputStorage_t is an int
  const int max_val = 512;  // M0 DAC is signed 10b, so -512..511
  const int compression_threshold = max_val - 64;  // let's use the 4 LSB to compress the 4 MSB into
  int tmp_audio_out = abs(sample);
  if (tmp_audio_out <= compression_threshold) {
    return sample;
  }
  tmp_audio_out = tmp_audio_out - compression_threshold;  // bias to 0
  // in observation, max/min was 8k once, 4k a number of times, 2k frequently, 1k-500 most often
  tmp_audio_out = constrain(tmp_audio_out, 0, 4095);  // hard clip for the rare case >= 4096
  // use a log function to squeeze 488..4095 into 0..64 and then add that to needs_compression
  // should give rounded shoulders, akin to tube compression, or at least better than hard clipping
  // get the 0..4095 into 0..255 and then use the exp table as a log table
  tmp_audio_out = tmp_audio_out >> 4;
  int compressed_bits = lin_to_log(tmp_audio_out);
  // now get it to fit into 0..63
  compressed_bits = compressed_bits >> 2;
  // finally store the result signed
  tmp_audio_out = compression_threshold + compressed_bits;
  if (sample >= 0) {
    if (tmp_audio_out == 512) {
      return 511;
    }
    return tmp_audio_out;
  }
  return -1 * tmp_audio_out; 
}

#endif
