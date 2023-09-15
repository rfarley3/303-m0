/*  Create a 303-ish synth with potentiometer controllers.
 *  (c) Ryan Farley ryanjfarley@gmail.com rfarley3@github
*/

#include <Wire.h>  // for I2C

#define ADDR_ADS0 0x48 // ADS7830 I2C address when D0 & D1 pull low (connected to ground), 4b is when both are not connected to ground or pulled high
// ADS7830 is 8b
#define POT_8b_ZERO 2  // Anything below this value is treated as "zero"
#define POT_8b_MAX 255
//// Trinket M0 on-board ADC is 12b
//#define POT_12b_ZERO 16  // Anything below this value is treated as "zero"
//#define POT_12b_MAX 4095

#define ONBOARD_ADC 4
#define FAKE_INDEX_FOR_ONBOARD_ADC 8



//// for why DEF_pot* see lines 300ish in https://github.com/diyelectromusic/sdemp/blob/main/src/SDEMP/TrinketUSBMIDIMultiPotMozziSynth3/TrinketUSBMIDIMultiPotMozziSynth3.ino
//// on-board ADC (pins 0..4) 10b 0..4095
//#define DEF_potCUT POT_12b_MAX
//// ADS7830 ADC (pins 0..7) 8b 0..255
//#define DEF_potRES POT_8b_MAX  // higher value means lower resonance for SVFilter
//#define DEF_potOSC0WAVT 0
//
//
//int osc0_wavt_cnt = 4;  // count of the switch statement options
//int potosc0wavt = DEF_potOSC0WAVT;
//int osc0_wavt_pot_to_val[POT_8b_MAX + 1];
//
//// svf freq range is 20 Hz to AUDIO_RATE/4 (32k/4 = 8192)
//#define CUT_MIN 20
//const int CUT_MAX = AUDIO_RATE >> 2;  // AUDIO_RATE/4;
//int cutoff = CUT_MAX;
//int potCUT = DEF_potCUT;
//// const float cut_max = 20000.0;  // max moog filter is 20k, prev 16k, what is 303?
//int cut_pot_to_val[POT_12b_MAX + 1];
//#define RES_MIN 1  // min value, but maximum resonance
//const int RES_MAX = 180; // observed, but technically 255, max value, but minimum res
//int resonance = RES_MAX;
//int potRES = DEF_potRES;
//int res_pot_to_val[POT_8b_MAX + 1];
//
//
//// with accent off, these are the time for exponential decay to 10%
//// accent on it is always 200, with off this is the min
//#define DCY_MIN 200
//// with accent off
//#define DCY_MAX 2500
//int decay_pot_to_val[POT_8b_MAX + 1];
//float acc_pot_to_gain_val[POT_8b_MAX + 1];
//int accent_level = LVL_NORM;


// most knobs are 0..255 
//   they are either raw that value
//   or it represents a % to apply against a range
//   or is a gain
int adc_linear[256];
// max res is res=1, min is res=255
//   to make it config in software, turning the knob "up" aka clockwise needs to be inverted
int adc_linear_inv[256];
// convert int 255..0 linear to exponential decay, TODO move to mozzi filter calcs
// int lin_to_exp[256];
int adc_exponential[256];  // TODO for cut


void adc_setup(){
  Wire.begin();
  // #ifdef ADAFRUIT_TRINKET_M0
  // https://diyelectromusic.wordpress.com/2021/03/20/trinket-fm-synthesis-with-mozzi/
  // Changing this to 12 only affects on-board; 8b ADC will not be messed up
  // analogReadResolution(12);
  analogReadResolution(8);  // for ease, just make everything 8b, this may cause filter cutoff stepping
  fill_adc_maps();
  // analogWriteResolution(10); .fromAlmostNBit means you shouldn't need to set this
//  populate_cut_pot_to_val();
//  populate_res_pot_to_val();
//  populate_osc0_wavt_pot_to_val();
//  populate_acc_pot_to_gain_val();
//  populate_decay_pot_to_val();
}


void fill_adc_maps() {
  for (int i = 0; i < POT_8b_ZERO; i++) {
    adc_exponential[i] = adc_linear_inv[255 - i] = adc_linear[i] = 0;
  }
  for(int i = POT_8b_ZERO; i < (POT_8b_MAX - POT_8b_ZERO); i++) {
    float perc = (float)(i - POT_8b_ZERO) / (float)(POT_8b_MAX - (2 * POT_8b_ZERO));
    adc_linear_inv[255 - i] = adc_linear[i] = (int)(255.0 * perc);
    adc_exponential[i] = 0; // TODO each step is larger as i increases
  }
  for(int i = (POT_8b_MAX - POT_8b_ZERO); i <= POT_8b_MAX; i++) {
    adc_exponential[i] = adc_linear_inv[255 - i] = adc_linear[i] = 255;
  }
}


int scale_8b_by_perc(int bit8, float perc) {
  int scaled = (int) ((float)bit8 * perc);
  return constrain(scaled, 0, 255);
}


int adc_read(uint8_t ain_idx) { //, bool inverted=false) {
  //  if (ain_idx > FAKE_INDEX_FOR_ONBOARD_ADC) {
  //    // error TODO print stmt
  //    return 0;
  //  }
  int val = 0;
  // HACK ain_idx 8 is onboard ADC
  if (ain_idx == FAKE_INDEX_FOR_ONBOARD_ADC) {
    val = mozziAnalogRead(ONBOARD_ADC);
    // verify 0..255
  }
  else {
    val = _adc_read(ain_idx);
    //if (ain_idx == RES_PIN) { Serial.print("1 "); Serial.println(val); }
  }
  // now smooth the ends that may jiggle and assure min/max exist
//  if (inverted) {
//    return adc_linear_inv[val];
//  }
  val = adc_linear[val];
  // if (ain_idx == RES_PIN) { Serial.print("2 "); Serial.println(val); }
  return val;
}


int _adc_read(uint8_t ain_idx) {
  uint8_t data = 0;
  Wire.beginTransmission(ADDR_ADS0);
  Wire.write(adc_read_cmd(ain_idx));
  Wire.endTransmission();  
  // Request 1 byte of data
  Wire.requestFrom(ADDR_ADS0, 1);
  // Read 1 byte of data; TODO add timeout
  if (Wire.available() == 1) {
    data = Wire.read();
  }
  // it's faster to send the request for the next sample now
  // then then it'll be closer to ready next time
  return (int) data;
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


//// int ctrl_read(uint8_t ain_pin, bool inverted=false, int range=0) {
//int ctrl_read(uint8_t ain_pin) { // , bool inverted, int range) {
//  // read ain_pin as 0..255, map to range
//  // use range 100 for float conversion or gain calcs
//  int val = adc_read(ain_pin); // , inverted);
////  if (!range) {
//    return val;
////  }
////  // val of 255 (and only 255) will return range
////  // if range is used as an index, use constrain to avoid index error
////  return map(val, 0, 255, 0, range);  
//}


//void populate_cut_pot_to_val() {
//  for (int i = 0; i < POT_12b_ZERO; i++) {
//    cut_pot_to_val[i] = CUT_MIN;
//  }
//  for(int i = POT_12b_ZERO; i < (POT_12b_MAX - POT_12b_ZERO); i++) {
//    // Serial.print("Cut v "); Serial.println(cut_value);
//    float perc = (float)(i - POT_12b_ZERO) / (float)(POT_12b_MAX - (2 * POT_12b_ZERO));
//    // Serial.print("Cut % "); Serial.println(perc);
//    cut_pot_to_val[i] = CUT_MIN + (int)((float)(CUT_MAX - CUT_MIN) * perc); 
//    // Serial.print("Cut f "); Serial.println(cut_pot_to_val[i]);
//  }
//  for(int i = (POT_12b_MAX - POT_12b_ZERO); i <= POT_12b_MAX; i++) {
//    cut_pot_to_val[i] = CUT_MAX;
//  }
//}
//
//
////  int res = (int)(RES_MIN + ((RES_MAX - RES_MIN) * ((float)data / 255.0)));
//void populate_res_pot_to_val() {
//  // res_pot_to_val[i] = 0;
//  for (int i = 0; i < POT_8b_ZERO; i++) {
//    res_pot_to_val[i] = RES_MIN;
//  }
//  for(int i = POT_8b_ZERO; i < (POT_8b_MAX - POT_8b_ZERO); i++) {
//    float perc = (float)(i - POT_8b_ZERO) / (float)(POT_8b_MAX - (2 * POT_8b_ZERO));
//    res_pot_to_val[i] = RES_MIN + (int)((float)(RES_MAX - RES_MIN) * perc); 
//  }
//  for(int i = (POT_8b_MAX - POT_8b_ZERO); i <= POT_8b_MAX; i++) {
//    res_pot_to_val[i] = RES_MAX;
//  }
//}
//
//
//void populate_osc0_wavt_pot_to_val() {
//  for (int i = 0; i < POT_8b_ZERO; i++) {
//    osc0_wavt_pot_to_val[i] = 0;
//  }
//  int pot_8b_max_zone = POT_8b_MAX - POT_8b_ZERO + 1;
//  for(int i = POT_8b_ZERO; i < pot_8b_max_zone; i++) {
//    float perc = (float)(i - POT_8b_ZERO) / (float)(POT_8b_MAX - (2 * POT_8b_ZERO));
//    osc0_wavt_pot_to_val[i] = (int)((float)osc0_wavt_cnt * perc);
//    if (osc0_wavt_pot_to_val[i] == osc0_wavt_cnt) {
//      osc0_wavt_pot_to_val[i] = osc0_wavt_cnt - 1;
//    }
//  }
//  for(int i = pot_8b_max_zone; i <= POT_8b_MAX; i++) {
//    osc0_wavt_pot_to_val[i] = osc0_wavt_cnt - 1;
//  }
//}
//
//
//void populate_acc_pot_to_gain_val() {
//  for (int i = 0; i < POT_8b_ZERO; i++) {
//    acc_pot_to_gain_val[i] = 1.0;
//  }
//  float max_gain = (float)LVL_MAX / (float)LVL_NORM;
//  int pot_8b_max_zone = POT_8b_MAX - POT_8b_ZERO + 1;
//  for(int i = POT_8b_ZERO; i < pot_8b_max_zone; i++) {
//    float perc = (float)(i - POT_8b_ZERO) / (float)(POT_8b_MAX - (2 * POT_8b_ZERO));
//    acc_pot_to_gain_val[i] = 1.0 + ((max_gain - 1.0) * perc);
//  }
//  for(int i = pot_8b_max_zone; i <= POT_8b_MAX; i++) {
//    acc_pot_to_gain_val[i] = max_gain;
//  }
//}
//
//
//void populate_decay_pot_to_val() {
//  for (int i = 0; i < POT_8b_ZERO; i++) {
//    decay_pot_to_val[i] = DCY_MIN;
//  }
//  int pot_8b_max_zone = POT_8b_MAX - POT_8b_ZERO + 1;
//  for(int i = POT_8b_ZERO; i < pot_8b_max_zone; i++) {
//    float perc = (float)(i - POT_8b_ZERO) / (float)(POT_8b_MAX - (2 * POT_8b_ZERO));
//    decay_pot_to_val[i] = DCY_MIN + (int)((float)(DCY_MAX - DCY_MIN) * perc);    
//  }
//  for(int i = pot_8b_max_zone; i <= POT_8b_MAX; i++) {
//    decay_pot_to_val[i] = DCY_MAX;
//  }
//}
