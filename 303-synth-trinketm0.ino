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

#include <MIDI.h>

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


// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 64 // Hz, powers of 2 are most reliable

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

// for why DEF_pot* see lines 300ish in https://github.com/diyelectromusic/sdemp/blob/main/src/SDEMP/TrinketUSBMIDIMultiPotMozziSynth3/TrinketUSBMIDIMultiPotMozziSynth3.ino
// on-board ADC (pins 0..4) 10b 0..4095
//#define CUT_PIN 4
//#define DEF_potCUT POT_12b_MAX
// ADS7830 ADC (pins 0..7) 8b 0..255
//#define RES_PIN 0
//#define DEF_potRES POT_8b_MAX  // higher value means lower resonance for SVFilter
//#define OSC0WAVT_PIN 1
#define DEF_potOSC0WAVT 0

// Set the MIDI Channel to listen on
#define MIDI_CHANNEL 1
#define MIDI_RATE    16    // Cycles through the loop() function
#define MIDI_LED LED_BUILTIN // shows if MIDI is being recieved
int midicount = 0;
MIDI_CREATE_DEFAULT_INSTANCE();

#define NUM_OSCILS 1
Oscil<COS2048_NUM_CELLS, AUDIO_RATE> oscils[NUM_OSCILS];
int oscils_note[NUM_OSCILS];
int oscils_freq[NUM_OSCILS];
bool oscils_playing[NUM_OSCILS];
int oscils_wavt[NUM_OSCILS];
//int oscils_wavt_cnt[NUM_OSCILS];
int osc0_wavt_cnt = 4;  // count of the switch statement options
int potosc0wavt = DEF_potOSC0WAVT;
//int osc0_wavt_pot_to_val[POT_8b_MAX + 1];
// use: Oscil <table_size, update_rate> oscilName (wavetable), look in .h file of table #included above
// Oscil <COS2048_NUM_CELLS, CONTROL_RATE> kFilterMod(COS2048_DATA);
//Oscil <, AUDIO_RATE> WAVEFORM0 = [
//  Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aSin(SIN2048_DATA),
//  Oscil <WHITENOISE8192_NUM_CELLS, AUDIO_RATE> aNoise(WHITENOISE8192_DATA)  // noise always at end
//];
//const uint8_t WAVEFORM0_LEN = 2;
//const uint8_t WAVEFORM0_NOISE_START = 1;
//uint8_t WAVEFORM0_IDX = 0;  // default
//float WAVEFORM0_FRQ = 440; // default starting freq concert A
#define MIDI_NOTE_CNT 128
int note_on[MIDI_NOTE_CNT];
int note_on_order = 0;


StateVariable <LOWPASS> svf; // can be LOWPASS, BANDPASS, HIGHPASS or NOTCH
// svf freq range is 20 Hz to AUDIO_RATE/4 (32k/4 = 8192)
#define CUT_MIN 20
const int CUT_MAX = AUDIO_RATE >> 2;  // AUDIO_RATE/4;
int cutoff = CUT_MAX;
int potCUT = 255; //DEF_potCUT;
// const float cut_max = 20000.0;  // max moog filter is 20k, prev 16k, what is 303?
//int cut_pot_to_val[POT_12b_MAX + 1];
#define RES_MIN 1  // min value, but maximum resonance
const int RES_MAX = 255;  // 180; // observed, but technically 255, max value, but minimum res
int resonance = 1; //RES_MAX;
int potRES = 1; //DEF_potRES;
//int res_pot_to_val[POT_8b_MAX + 1];


// with accent off, these are the time for exponential decay to 10%
// accent on it is always 200, with off this is the min
#define DCY_MIN 200
// with accent off
#define DCY_MAX 2500
//int decay_pot_to_val[POT_8b_MAX + 1];
//float acc_pot_to_gain_val[POT_8b_MAX + 1];
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


void setup_debug () {
  /* if debug off, we don't need serial */
  if (!DEBUG) {
    return;
  }
  Serial.begin(115200);
  Serial.println("Hello world");
}


void setup () {
  pinMode(MIDI_LED, OUTPUT);
  setup_debug();
  adc_setup();
  dotstar_setup();

  // Connect the HandleNoteOn function to the library, so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(HandleNoteOn);  // Put only the name of the function
  MIDI.setHandleNoteOff(HandleNoteOff);  // Put only the name of the function
  MIDI.begin(MIDI_CHANNEL);  // MIDI_CHANNEL_OMNI means all channels
  init_note_on();
  oscils_freq[0] = 440;
  oscils_note[0] = 81;  // A4
  oscils_playing[0] = false;
  oscils_wavt[0] = 0;
  //oscils_wavt_cnt[0] = 2;
  // oscils[0].setTable(SIN2048_DATA);
  set_wavetables();
  oscils[0].setFreq(oscils_freq[0]); // set the frequency
  // set_wf0(WAVEFORM0_FRQ, WAVEFORM0_IDX);
  // kFilterMod.setFreq(1.3f);
  svf.setResonance(resonance);  // 25
  svf.setCentreFreq(cutoff);  // 1200
  venv[0].setADLevels(accent_level, 0);  // att, dcy; 0-255. 
  //venv[0].setTimes(50,200,10000,200); // 10000 is so the note will sustain 10 seconds unless a noteOff comes
  venv[0].setTimes(3, 10000, 0, 0); // 303 VENV is constant, accent changes levels
  fenv[0].setADLevels(LVL_NORM, 0);
  fenv[0].setTimes(3, 2500, 0, 0); // 303 FENV delay changes with knob, levels change as function of env mod or accent, res
  startMozzi(CONTROL_RATE); // :)
}


void loop(){
  audioHook(); // required here
  rainbowHook();
  midiHook();
}


void init_note_on () {
  /* we track the note on order within this, with one element per possible midi note value */
  for (int i = 0; i < MIDI_NOTE_CNT; i++) {
    note_on[i] = -1;
  }
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
  // TODO oscil specific order of tables (ex: LFO vs OSC)
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


void midiHook () {
  /* we don't need to check MIDI every loop, here's a sane amount */
  midicount++;
  if (midicount > MIDI_RATE) {
    MIDI.read();
    midicount = 0;
  }
}


void HandleNoteOn (byte channel, byte note, byte velocity) {
  /* MIDI Hook when note on message received */
  // there would be where channel mapping to instrument (bass vs kd vs hats) would route noteOns
  // assume only bass (303) for now
  if (DEBUG) { Serial.print("MIDI note on "); Serial.print(channel); Serial.print(" "); Serial.print(note); Serial.print(" "); Serial.println(velocity); }
  if (velocity == 0) {
    HandleNoteOff(channel, note, velocity);
    return;
  }
  // prob doesn't matter, but error check bc why not
  if (note_on_order >= MIDI_NOTE_CNT) {
    if (DEBUG) { Serial.println("Reach max note_on_order, ignoring note"); }
    return;
  }
  // if note is new or already on, update its order for most recent priority
  note_on_order++;
  note_on[note] = note_on_order;

  // note priority is last, aka most recent, so this could mean a freq change
  if (note != oscils_note[0]) {
    note_change(0, note);
    // this would be where the subosc offset could be calc'ed and set
    // note_change(1, note - suboscoffset, false);
  }
  // if venv is in ADS, then let it finish
  if (oscils_playing[0]) {
    // just let the freq change happen above and finish existing env trigger
    // TODO see if the env has gone idle, or generally fully expired. if so trigger, else then continue with this return
    return;
  }
  trigger_env(0);
  digitalWrite(MIDI_LED, HIGH);
}


void note_change (int osc_idx, int note) {
  /* the note changed, update globals, convert to freq, and set the osc */
  int freq = (int)mtof(note);
  if (DEBUG) { Serial.print("Freq "); Serial.print(oscils_freq[osc_idx]); Serial.print(" -> "); Serial.println(freq); }
  oscils_freq[osc_idx] = freq;
  oscils_note[osc_idx] = note;
  oscils[osc_idx].setFreq(oscils_freq[osc_idx]);  // +- bend
}


void trigger_env (int osc_idx) {
  /* trigger both envelopes, probably a new note (not a glide) happened */
  if (DEBUG) { Serial.println("Triggering ENV ADSR"); }
  oscils_playing[osc_idx] = true;
  venv[osc_idx].noteOn();
  fenv[osc_idx].noteOn();
}


void stop_env (int osc_idx) {
  /* when no note pressed, switch to rel. with 0 sustain you get here if decay not finished, 303 rel is 0 anyways */
  if (DEBUG) { Serial.println("Triggering VENV Rel (is currently playing"); }
  // if (venv[0].playing()) {
  oscils_playing[osc_idx] = false;
  venv[osc_idx].noteOff();
  fenv[osc_idx].noteOff();
}


void HandleNoteOff (byte channel, byte note, byte velocity) {
  /* MIDI Hook when note off message received (or note on of 0 velocity */
  if (DEBUG) { Serial.print("MIDI note off "); Serial.print(channel); Serial.print(" "); Serial.print(note); Serial.print(" "); Serial.println(velocity); }
  int note_order = note_on[note];
  if (note_order == -1) {
    // this note is already off, ignore
    if (DEBUG) { Serial.println("Note has no note order, shouldn't be here, ignoring"); }
    return;
  }
  if (note_on_order == 0) {
    if (DEBUG) { Serial.println("Zero note_on_order, shouldn't be here, we got a note while we think no notes are playing, ignoring"); }
    return;
  }
  note_on[note] = -1;
  // if this note is not the priority note, just remove it from the fallbacks
  if (note_order < note_on_order) {
    if (DEBUG) { Serial.println("Not priority note, removed from fallbacks"); }
    // earlier note_on[note] of -1 removes it from fallbacks
    return;
  }
  // shouldn't happen, ignore for now
  if (note_order > note_on_order) {
    if (DEBUG) { Serial.println("Out of bounds note order, ignoring"); }
    return;
  }
  // note_order == note_on_order, so this is the playing note (highest priority)
  if (note_on_order == 1) {
    // there is only one note
    note_on_order = 0;
    stop_env(0);
    digitalWrite(MIDI_LED, LOW);
    return; 
  }
  // note_on_order > 1. There are fallbacks, find it TODO TODO
  // handle press 1, 2, 3, let go of 2, then 3 should still play, let go of 3, then 1. TODO
  int fb_idx = -1;
  int fallback = -1;
  for (int i = 0; i < MIDI_NOTE_CNT; i++) {
    if (note_on[i] > fb_idx) {
      fb_idx = note_on[i];
      fallback = i;
    }
  }
  if (fallback == -1) {
    // 0 is note a valid MIDI note
    if (DEBUG) { Serial.println("No fallback found, ignoring"); }
    note_on_order = 0;
    stop_env(0);
    digitalWrite(MIDI_LED, LOW);
    return; 
  }
  // we have a fallback, so reduce the size of the queued notes
  if (note_on_order > 0) {
    note_on_order--;
  }
  // another note needs to played instead
  // just switch freqs and then let existing env keep playing
  if (DEBUG) { Serial.print("Found fallback note "); Serial.println(fallback); }
  note_change(0, fallback);
}


// for testing, this is c harmonic minor
byte notes[] = { 48, 50, 51, 53, 55, 56, 58, 60 };


void updateControl () {
  // called by mozzi every CONTROL_RATE loops
  // meant to update things, and then in audioHook, use faster next() to extrapolate what to output via DAC
  // this is where you read knobs and set/update any numbers extrapolated/used within audioHook
  int tbd_val = adc_read(TBD_PIN);
  tbd_val = map(tbd_val, 0, 255, 0, 8);  
  tbd_val = constrain(tbd_val, 0, 7); 
  if (tbd_val != tbd) {
    // TODO trigger handleNoteOn
    if (DEBUG) { Serial.print("Tbd "); Serial.print(tbd); Serial.print(" -> "); Serial.println(tbd_val); } 
    HandleNoteOff(MIDI_CHANNEL, notes[tbd], LVL_NORM);
    tbd = tbd_val;
    HandleNoteOn(MIDI_CHANNEL, notes[tbd_val], LVL_NORM);
  }
  // freq = fn(note, tuning offset, glide)
  // wave = ratio of knob to number of options
  // ctrl_wave();  // sets wave
  int waveform = adc_read(OSC0WAVT_PIN);
  waveform = map(waveform, 0, 255, 0, osc0_wavt_cnt);  
  waveform = constrain(waveform, 0, osc0_wavt_cnt - 1); // else 255 is osc0_wavt_cnt which would be an index error
  if (waveform != oscils_wavt[0]) {
    if (DEBUG) { Serial.print("Osc0-wave "); Serial.print(oscils_wavt[0]); Serial.print(" -> "); Serial.println(waveform); }
    oscils_wavt[0] = waveform;
    set_wavetable(0);
  }
  // acc = accent_on ? ratio of knob : 0
  // accent_on affects others
  //   * fenv = fenv(dcy=.2)  // dcy looses effect; per schem
  //   * cut = cut - env_mod_bias + fenv*env_mod% + smooth_via_c13(res, fenv*acc%)
  //   * venv = venv + acc%*(LVL_MAX-LVL_NORM)
  // int acc = ctrl_accent();  // nothing to directly set, only reads, ret 0..255
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
  //int res = ctrl_res();  // reads and sets res, ret 1..255
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
  // if !accent_on: fenv(atk=3 msec, dcy=200 msec +(2500-200)*ratio of knob)
  // if  accent_on: fenv(atk=3 msec, dcy=200 msec)
  // TODO enable ctrl_fenv_decay(accent_on);  // reads and sets fenv dcy
  int dcy_val = adc_read(DCY_PIN);
  int dcy_ms = 200;
  if (!accent_on) {
    dcy_ms = map(dcy_val, 0, 255, DCY_MIN, DCY_MAX);
  }
  if (dcy_ms != decay) {
    if (DEBUG) { Serial.print("Dcy "); Serial.print(decay); Serial.print(" -> "); Serial.println(dcy_ms); }
    decay = dcy_ms;    
    // see if you don't need to set setADLevel based on accent if you don't need to, use the boost ratio in control
    fenv[0].setTimes(3, decay, 0, 0);
  }
  fenv[0].update();
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


AudioOutput_t updateAudio(){  
  // return MonoOutput::from8Bit(aSin.next()); // return an int signal centred around 0
  // 'from' means to convert from 'Nbit' to audio output bit width (hardware specific)
  // return MonoOutput::fromAlmostNBit(12, svf.next(oscils[0].next()));
  return MonoOutput::from16Bit((int) (venv[0].next() * svf.next(oscils[0].next())));  // why (int)?? why not better faster math?
}
