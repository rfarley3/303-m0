/*  Create a 303-ish synth with potentiometer controllers.
 *  (c) Ryan Farley ryanjfarley@gmail.com rfarley3@github

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
      *   A5 glide
      *   A6
      *   A7

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

#include <Wire.h>  // for I2C
#include <MIDI.h>

#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
//  ~/Arduino/libraries/Mozzi/tables 
#include <tables/cos2048_int8.h> // for the modulation oscillators
#include <tables/sin2048_int8.h> // sine table for oscillator
#include <tables/saw2048_int8.h> // saw table for oscillator
#include <tables/triangle2048_int8.h> // triangle table for oscillator
#include <tables/square_no_alias_2048_int8.h> // square table for oscillator
// #include <tables/brownnoise8192_int8.h>
// #include <tables/noise_static_1_16384_int8.h>
// #include <tables/pinknoise8192_int8.h>
// #include <tables/whitenoise8192_int8.h>
#include <StateVariable.h>
#include <mozzi_rand.h> // for rand()
#include <mozzi_midi.h>
#include <ADSR.h>
#include <Ead.h>

#ifdef ADAFRUIT_TRINKET_M0
// setup Dotstar LED on Trinket M0
#include <Adafruit_DotStar.h>
Adafruit_DotStar strip(DOTSTAR_NUM, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLK, DOTSTAR_BRG);
#endif
unsigned int rainbow_loops = 0;
long firstPixelHue = 0;


// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 64 // Hz, powers of 2 are most reliable

#define ADDR_ADS0 0x4b // ADS7830 I2C address
// ADS7830 is 8b
#define POT_8b_ZERO 2  // Anything below this value is treated as "zero"
#define POT_8b_MAX 255
// Trinket M0 on-board ADC is 12b
#define POT_12b_ZERO 16  // Anything below this value is treated as "zero"
#define POT_12b_MAX 4095

// for why DEF_pot* see lines 300ish in https://github.com/diyelectromusic/sdemp/blob/main/src/SDEMP/TrinketUSBMIDIMultiPotMozziSynth3/TrinketUSBMIDIMultiPotMozziSynth3.ino
// on-board ADC (pins 0..4) 10b 0..4095
#define CUT_PIN 4
#define DEF_potCUT POT_12b_MAX
// ADS7830 ADC (pins 0..7) 8b 0..255
#define RES_PIN 0
#define DEF_potRES POT_8b_MAX  // higher value means lower resonance for SVFilter
#define OSC0WAVT_PIN 1
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
int osc0_wavt_pot_to_val[POT_8b_MAX + 1];
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
int potCUT = DEF_potCUT;
// const float cut_max = 20000.0;  // max moog filter is 20k, prev 16k, what is 303?
int cut_pot_to_val[POT_12b_MAX + 1];
#define RES_MIN 1  // min value, but maximum resonance
const int RES_MAX = 180; // observed, but technically 255, max value, but minimum res
int resonance = RES_MAX;
int potRES = DEF_potRES;
int res_pot_to_val[POT_8b_MAX + 1];


// with accent off, these are the time for exponential decay to 10%
// accent on it is always 200, with off this is the min
#define DCY_MIN 200
// with accent off
#define DCY_MAX 2500
int decay_pot_to_val[POT_8b_MAX + 1];
float acc_pot_to_gain_val[POT_8b_MAX + 1];


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


void setup(){
  Serial.begin(115200);
  Serial.println("Hello world");
  Wire.begin();
  pinMode(MIDI_LED, OUTPUT);

  // Start with built-in Dotstar RGB LED turned off
  strip.begin();
  strip.clear();
  strip.setBrightness(60);
  strip.show();

  // Connect the HandleNoteOn function to the library, so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(HandleNoteOn);  // Put only the name of the function
  MIDI.setHandleNoteOff(HandleNoteOff);  // Put only the name of the function
  MIDI.begin(MIDI_CHANNEL);  // MIDI_CHANNEL_OMNI means all channels
  
  // #ifdef ADAFRUIT_TRINKET_M0
  // https://diyelectromusic.wordpress.com/2021/03/20/trinket-fm-synthesis-with-mozzi/
  // Will changing it to 12 mean the 8b ADC will pad to 12b, or somehow be messed up?
  analogReadResolution(12);
  // analogWriteResolution(10); .fromAlmostNBit means you shouldn't need to set this
  populate_cut_pot_to_val();
  populate_res_pot_to_val();
  populate_osc0_wavt_pot_to_val();
  populate_acc_pot_to_gain_val();
  populate_decay_pot_to_val();
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


void init_note_on() {
  for (int i = 0; i < MIDI_NOTE_CNT; i++) {
    note_on[i] = -1;
  }
}


void populate_cut_pot_to_val() {
  for (int i = 0; i < POT_12b_ZERO; i++) {
    cut_pot_to_val[i] = CUT_MIN;
  }
  for(int i = POT_12b_ZERO; i < (POT_12b_MAX - POT_12b_ZERO); i++) {
    // Serial.print("Cut v "); Serial.println(cut_value);
    float perc = (float)(i - POT_12b_ZERO) / (float)(POT_12b_MAX - (2 * POT_12b_ZERO));
    // Serial.print("Cut % "); Serial.println(perc);
    cut_pot_to_val[i] = CUT_MIN + (int)((float)(CUT_MAX - CUT_MIN) * perc); 
    // Serial.print("Cut f "); Serial.println(cut_pot_to_val[i]);
  }
  for(int i = (POT_12b_MAX - POT_12b_ZERO); i <= POT_12b_MAX; i++) {
    cut_pot_to_val[i] = CUT_MAX;
  }
}


//  int res = (int)(RES_MIN + ((RES_MAX - RES_MIN) * ((float)data / 255.0)));
void populate_res_pot_to_val() {
  // res_pot_to_val[i] = 0;
  for (int i = 0; i < POT_8b_ZERO; i++) {
    res_pot_to_val[i] = RES_MIN;
  }
  for(int i = POT_8b_ZERO; i < (POT_8b_MAX - POT_8b_ZERO); i++) {
    float perc = (float)(i - POT_8b_ZERO) / (float)(POT_8b_MAX - (2 * POT_8b_ZERO));
    res_pot_to_val[i] = RES_MIN + (int)((float)(RES_MAX - RES_MIN) * perc); 
  }
  for(int i = (POT_8b_MAX - POT_8b_ZERO); i <= POT_8b_MAX; i++) {
    res_pot_to_val[i] = RES_MAX;
  }
}


void populate_osc0_wavt_pot_to_val() {
  for (int i = 0; i < POT_8b_ZERO; i++) {
    osc0_wavt_pot_to_val[i] = 0;
  }
  int pot_8b_max_zone = POT_8b_MAX - POT_8b_ZERO + 1;
  for(int i = POT_8b_ZERO; i < pot_8b_max_zone; i++) {
    float perc = (float)(i - POT_8b_ZERO) / (float)(POT_8b_MAX - (2 * POT_8b_ZERO));
    osc0_wavt_pot_to_val[i] = (int)((float)osc0_wavt_cnt * perc);
    if (osc0_wavt_pot_to_val[i] == osc0_wavt_cnt) {
      osc0_wavt_pot_to_val[i] = osc0_wavt_cnt - 1;
    }
  }
  for(int i = pot_8b_max_zone; i <= POT_8b_MAX; i++) {
    osc0_wavt_pot_to_val[i] = osc0_wavt_cnt - 1;
  }
}


void populate_acc_pot_to_gain_val() {
  for (int i = 0; i < POT_8b_ZERO; i++) {
    acc_pot_to_gain_val[i] = 1.0;
  }
  float max_gain = (float)LVL_MAX / (float)LVL_NORM;
  int pot_8b_max_zone = POT_8b_MAX - POT_8b_ZERO + 1;
  for(int i = POT_8b_ZERO; i < pot_8b_max_zone; i++) {
    float perc = (float)(i - POT_8b_ZERO) / (float)(POT_8b_MAX - (2 * POT_8b_ZERO));
    acc_pot_to_gain_val[i] = 1.0 + ((max_gain - 1.0) * perc);
  }
  for(int i = pot_8b_max_zone; i <= POT_8b_MAX; i++) {
    acc_pot_to_gain_val[i] = max_gain;
  }
}


void populate_decay_pot_to_val() {
  for (int i = 0; i < POT_8b_ZERO; i++) {
    decay_pot_to_val[i] = DCY_MIN;
  }
  int pot_8b_max_zone = POT_8b_MAX - POT_8b_ZERO + 1;
  for(int i = POT_8b_ZERO; i < pot_8b_max_zone; i++) {
    float perc = (float)(i - POT_8b_ZERO) / (float)(POT_8b_MAX - (2 * POT_8b_ZERO));
    decay_pot_to_val[i] = DCY_MIN + (int)((float)(DCY_MAX - DCY_MIN) * perc);    
  }
  for(int i = pot_8b_max_zone; i <= POT_8b_MAX; i++) {
    decay_pot_to_val[i] = DCY_MAX;
  }
}


int scale_8b_by_perc(int bit8, float perc) {
  int scaled = (int) ((float)bit8 * perc);
  if (scaled > 255) {
    return 255;
  }
  if (scaled < 0) {
    return 0; 
  }
  return scaled;
}


void set_wavetables() {
  for (int i = 0; i < NUM_OSCILS; i++) {
    set_wavetable(i);
  }
}


void set_wavetable(int oscil_idx) {
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

void midiHook() {
  midicount++;
  if (midicount > MIDI_RATE) {
    MIDI.read();
    midicount = 0;
  }
}


void HandleNoteOn(byte channel, byte note, byte velocity) {
  // there would be where channel mapping to instrument (bass vs kd vs hats) would route noteOns
  // assume only bass (303) for now
  Serial.print("MIDI note on "); Serial.print(channel); Serial.print(" "); Serial.print(note); Serial.print(" "); Serial.println(velocity);
  if (velocity == 0) {
    HandleNoteOff(channel, note, velocity);
    return;
  }
  // prob doesn't matter, but error check bc why not
  if (note_on_order >= MIDI_NOTE_CNT) {
    Serial.println("Reach max note_on_order, ignoring note");
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
    return;
  }
  trigger_env(0);
  digitalWrite(MIDI_LED, HIGH);
}


void note_change (int osc_idx, int note) {
  int freq = (int)mtof(note);
  Serial.print("Freq "); Serial.print(oscils_freq[osc_idx]); Serial.print(" -> "); Serial.println(freq);
  oscils_freq[osc_idx] = freq;
  oscils_note[osc_idx] = note;
  oscils[osc_idx].setFreq(oscils_freq[osc_idx]);  // +- bend
}


void trigger_env (int osc_idx) {
  Serial.println("Triggering ENV ADSR");
  oscils_playing[0] = true;
  venv[osc_idx].noteOn();
  fenv[osc_idx].noteOn();
}


void stop_env (int osc_idx) {
  Serial.println("Triggering VENV Rel (is currently playing");
  // if (venv[0].playing()) {
  oscils_playing[osc_idx] = false;
  venv[osc_idx].noteOff();
  fenv[osc_idx].noteOff();
}


void HandleNoteOff(byte channel, byte note, byte velocity) {
  Serial.print("MIDI note off "); Serial.print(channel); Serial.print(" "); Serial.print(note); Serial.print(" "); Serial.println(velocity);  
  int note_order = note_on[note];
  if (note_order == -1) {
    // this note is already off, ignore
    Serial.println("Note has no note order, shouldn't be here, ignoring");
    return;
  }
  if (note_on_order == 0) {
    Serial.println("Zero note_on_order, shouldn't be here, ignoring");
    return;
  }
  note_on[note] = -1;
  // if this note is not the priority note, just remove it from the fallbacks
  if (note_order < note_on_order) {
    Serial.println("Not priority note, removed from fallbacks");
    // earlier note_on[note] of -1 removes it from fallbacks
    return;
  }
  // shouldn't happen, ignore for now
  if (note_order > note_on_order) {
    Serial.println("Out of bounds note order, ignoring");
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
    Serial.println("No fallback found, ignoring");
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
  Serial.print("Found fallback note "); Serial.println(fallback);
  note_change(0, fallback);
}


void updateControl(){
  // freq = fn(note, tuning offset, glide)
  // wave = ratio of knob to number of options
  ctrl_wave();  // sets wave
  // acc = accent_on ? ratio of knob : 0
  // accent_on affects others
  //   * fenv = fenv(dcy=.2)
  //   * cut = cut + smooth_via_c13(res, fenv*acc%)
  //   * env_mod looses any effect on fenv>cut
  //   * venv = venv + acc%*(LVL_MAX-LVL_NORM)
  int acc = ctrl_accent();  // nothing to directly set, only reads, ret 0..255
  // res = ratio of knob
  int res = ctrl_res();  // reads and sets res, ret 1..255
  // if !accent_on: env_mod = ratio of knob
  // if  accent_on: env_mod = 0
  int env_mod = 0;
  if (!accent_on) {
    env_mod = ctrl_env_mod();  // nothing to directly set, only reads, ret 0..255
  }
  // if !accent_on: fenv(atk=3 msec, dcy=200 msec +(2500-200)*ratio of knob)
  // if  accent_on: fenv(atk=3 msec, dcy=200 msec)
  // TODO enable ctrl_fenv_decay(accent_on);  // reads and sets fenv dcy
  fenv[0].update();
  // if !accent_on: cut = cut + fn(fenv(dcy=knob)*env_mod%)
  // if  accent_on: cut = cut + smooth_via_c13(res, fenv*acc%)
  //                res 0% is fenv/acc. res 100% smooth(fenv*acc%)
  //                see https://www.firstpr.com.au/rwi/dfish/303-unique.html
  int cut = ctrl_cut();  // only reads, ret 20..8192
  int fenv_boost = 0;
  if (false /*TODO*/ && env_mod && !accent_on) {
    int fenv_level = fenv[0].next();
    // float fenv_boost_perc = acc_pot_to_gain_val[env_mod];  // >= 1.0
    float fenv_attenuate_perc = (float) fenv_level * ((float) env_mod / 255.0);  // 0..255
    fenv_attenuate_perc = fenv_attenuate_perc / 255.0;  // 0..1
    float gain = ((float)LVL_MAX - (float)LVL_NORM) / (float)LVL_NORM;  // >= 1.0
    float attenuated_gain = 1.0 + ((gain - 1.0) * fenv_attenuate_perc);  // >= 1.0
    fenv_boost = (int)((float)cut + ((float)(CUT_MAX - cut) * attenuated_gain));
    // fenv_boost = calc_cut(cut, fenv[0], env_mod);
  }
  else if (accent_on) {
    // TODO
    // fenv_boost = calc_cut_acc(cut, fenv[0], acc, res);
  }
  svf.setCentreFreq(cut + fenv_boost);
  float venv_accent_boost = acc_pot_to_gain_val[acc];  // ret 1..(LVL_MAX/LVL_NORM)
  // venv(atk=3 msec, dcy=5000) * acc%*(LVL_MAX/LVL_NORM)
  // in audio: venv.next()*venv_accent_boost
  //           scale_8b_by_perc(venv.next(), venv_accent_boost)
  venv[0].update();
}


uint8_t adc_read_cmd(uint8_t ain_idx) {
  // TODO assert ain_idx < 8
  // for ADS7830 8 chan 8b ADC over I2C
  // https://kevinboone.me/adc.html?i=1
  // uint8_t single_ended = 0x80;
  uint8_t single_ended = 0b10000000; // bit 7 set for single-ended (vs differential)
  // uint8_t dac_on_ref_off = 0x04;
  uint8_t dac_on_ref_off = 0b00000100; // bit 3 (PD1) internal ref off; bit 2 (PD0) AD on
  // uint8_t channel = 0x00; // bits 4-6 contain the channel number to sample
  // uint8_t channel = 0b0xxx0000; // bits 4-6 contain the channel number to sample
  // according to specs though ain are not seqential, 0=000, 2=001, 4=010, 6=011, 1=100
  uint8_t channel = 0b00000000;
  switch (ain_idx) {
    case 0:
      channel = 0b00000000;
      break;
    case 1:
      channel = 0b01000000;
      break;
    case 2:
      channel = 0b00010000;
      break;
    case 3:
      channel = 0b01010000;
      break;
    case 4:
      channel = 0b00100000;
      break;
    case 5:
      channel = 0b01100000;
      break;
    case 6:
      channel = 0b00110000;
      break;
    case 7:
      channel = 0b01110000;
      break;
    default: // case 0
      channel = 0b00000000;
  }
  uint8_t cmd = single_ended | dac_on_ref_off | channel;
  // 0x80 0b10000000
  // 0x04 0b00000100
  // 0x00 0b0xxx0000
  // write (i2c, &cmd, 1);
  return cmd;
}


void ctrl_wave() {
  uint8_t data = 0;
  Wire.beginTransmission(ADDR_ADS0);
  Wire.write(adc_read_cmd(OSC0WAVT_PIN));
  Wire.endTransmission();
  // Request 1 byte of data
  Wire.requestFrom(ADDR_ADS0, 1);
  // Read 1 byte of data
  if (Wire.available() == 1) {
    data = Wire.read();
  }
  int waveform = osc0_wavt_pot_to_val[data];
  if (waveform == oscils_wavt[0]) {
    // no change
    return;
  }
  Serial.print("Osc0-wave "); Serial.print(oscils_wavt[0]); Serial.print(" -> "); Serial.println(waveform);
  oscils_wavt[0] = waveform;
  set_wavetable(0);
}


int ctrl_cut() {
  // mozziAnalogRead value is 0-1023 AVR, 0-4095 on STM32; set with analogReadResolution in setup
  int cut_value = mozziAnalogRead(CUT_PIN);
  int cut_freq = cut_pot_to_val[cut_value];
  // pot wired to return higher ADC when turned clockwise
  // Serial.print("Cut v "); Serial.println(cut_value);
  // Serial.print("Cut f "); Serial.println(cut_freq);
  if (cut_freq == cutoff) {
    // no change
    return cutoff;
  }
  Serial.print("Cut "); Serial.print(cutoff); Serial.print(" -> "); Serial.println(cut_freq);
  cutoff = cut_freq;
  return cutoff;
}


int ctrl_res() {
  uint8_t data = 0;
  Wire.beginTransmission(ADDR_ADS0);
  // Wire.write(0x08);
  // Wire.write(cmd);
  // write (i2c, &cmd, 1);
  Wire.write(adc_read_cmd(RES_PIN));
  Wire.endTransmission();
  // Request 1 byte of data
  Wire.requestFrom(ADDR_ADS0, 1);
  // Read 1 byte of data
  if (Wire.available() == 1) {
    data = Wire.read();
  }
  // lower res value means more resonance, valid values are 0-255
  // pot wired to return lower ADC when turned clockwise
  // too little res (>180) was pointless, so scale 0-255 into between 1-180
  int res = res_pot_to_val[data];
  if (res == resonance) {
    // no change
    return resonance;
  }
  Serial.print("Res "); Serial.print(resonance); Serial.print(" -> "); Serial.println(res);
  resonance = res;
  svf.setResonance(resonance);
  return resonance;
}


int ctrl_accent() {
  accent_level = 0; // TODO read knob, maybe make map from 0..255 to LVL_NORM..LVL_MAX
  return accent_level;
  
//  // setADLevel based on accent
//  accent_level = LVL_NORM + accent_level;
//  
//  // TODO use cc code or velocity, probably move to midi noteon or handle hook
//  accent_on = false;
//  if (accent_on) {
//    venv[0].setADLevels(accent_level, accent_level);
//    // TODO set fenv ADLevels as function of env mod or accent*inverse(res)
//  }
}


void ctrl_fenv_decay(bool accent_on) {
  // if !accent_on: fenv(atk=3 msec, dcy=200 msec +(2500-200)*ratio of knob)
  // if  accent_on: fenv(atk=3 msec, dcy=200 msec)
  int decay_msec = 200;
  if (!accent_on) {
    int decay_val = 64; // TODO read knob, maybe make map from 0..255 to 200..2500;
    decay_msec = decay_pot_to_val[decay_val];
  }
  // TODO
  // don't setADLevel based on accent if you don't need to, use the boost ratio in control
  // also don't change these unless you need to
  fenv[0].setTimes(3, decay_msec, 0, 0);
}


int ctrl_env_mod() {
  int mod_val = 255;  // 0..255 TODO read knob
  return mod_val;  
}


AudioOutput_t updateAudio(){  
  // return MonoOutput::from8Bit(aSin.next()); // return an int signal centred around 0
  // 'from' means to convert from 'Nbit' to audio output bit width (hardware specific)
  // return MonoOutput::fromAlmostNBit(12, svf.next(oscils[0].next()));
  return MonoOutput::from16Bit((int) (venv[0].next() * svf.next(oscils[0].next())));
}


void rainbowHook() {
  rainbow_loops++;
  if (rainbow_loops % 8192 == 0) {
    rainbow(firstPixelHue);
    // Color wheel has a range of 65536 and it's OK if we roll over, but % bc why not
    firstPixelHue = (firstPixelHue + 256) % 65536;
    rainbow_loops = 0;
  }
}


void rainbow(long firstPixelHue) {
    for(int i = 0; i < strip.numPixels(); i++) {
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
}


void loop(){
  audioHook(); // required here
  rainbowHook();
  midiHook();
}
