#ifndef synth303_adc_ino
#define synth303_adc_ino
/*  Create a 303-ish synth with potentiometer controllers.
 *  (c) Ryan Farley rfarley3@github
*/
#include <Wire.h>  // for I2C

#define ADDR_ADS0 0x48 // ADS7830 I2C address
// 0x48 when D0 & D1 pull low (connected to ground)
// 0x4b is when both are not connected to ground (floating) or pulled high
// ADS7830 is 8b
#define POT_8b_ZERO 2  // Anything below this value is treated as "zero"
#define POT_8b_MAX 255
//// Trinket M0 on-board ADC is 12b
//#define POT_12b_ZERO 16  // Anything below this value is treated as "zero"
//#define POT_12b_MAX 4095

#define ONBOARD_ADC 4
#define FAKE_INDEX_FOR_ONBOARD_ADC 8


// most knobs are 0..255 
//   they are either raw that value
//   or it represents a % to apply against a range
//   or is a gain
int adc_linear[256];


void adc_setup() {
  /* Same purpose as primary file setup */
  Wire.begin();
  // #ifdef ADAFRUIT_TRINKET_M0
  // https://diyelectromusic.wordpress.com/2021/03/20/trinket-fm-synthesis-with-mozzi/
  // Changing this to 12 only affects on-board; 8b ADC will not be messed up
  // default arduino analogReadResolution is for on-board ADC (pins 0..4) 10b 0..4095
  // analogReadResolution(12);
  analogReadResolution(8);  // for ease, just make everything 8b, this may cause filter cutoff stepping
  fill_adc_maps();
}


void fill_adc_maps() {
  /* Create lookup tables to avoid float calcs during main loop */
  for (int i = 0; i < POT_8b_ZERO; i++) {
    adc_linear[i] = 0;
  }
  for(int i = POT_8b_ZERO; i < (POT_8b_MAX - POT_8b_ZERO); i++) {
    float perc = (float)(i - POT_8b_ZERO) / (float)(POT_8b_MAX - (2 * POT_8b_ZERO));
    adc_linear[i] = (int)(255.0 * perc);
  }
  for(int i = (POT_8b_MAX - POT_8b_ZERO); i <= POT_8b_MAX; i++) {
    adc_linear[i] = 255;
  }
}


int adc_read(uint8_t ain_idx) {
  /* Determine if need to read on-board or ADS/I2C board
   *  for inverted values (like res in SVF) handle in caller, such as 255-val
   *  for values not 0..255, use min + map(val, 0, 255, 0, (max - min))  
   *  being careful if that number is an array index, such that val of 255 may return out of index error
   */
  int val = 0;
  // HACK ain_idx 8 is onboard ADC
  if (ain_idx == FAKE_INDEX_FOR_ONBOARD_ADC) {
    val = mozziAnalogRead(ONBOARD_ADC);
    // TODO SAMD21 has an ADC bug, verify if firmware updated, consider raw reader for speed
  }
  else {
    val = _adc_read(ain_idx);
  }
  val = adc_linear[val];
  return val;
}


int _adc_read(uint8_t ain_idx) {
  /* Do the actual request for data and then wait for/read the response 
   *  it's faster to send the request for the next sample now
   *  then then it'll be closer to ready next time
   *  but you can't do that when you don't know what the read is answering
   *  not worth the overhead to track
   */
  Wire.beginTransmission(ADDR_ADS0);
  Wire.write(adc_read_cmd(ain_idx));
  Wire.endTransmission();
  // Request 1 byte of data
  Wire.requestFrom(ADDR_ADS0, 1);
  uint8_t data = 0;
  // Read 1 byte of data; consider adding timeout
  if (Wire.available() == 1) {
    data = Wire.read();
  }
  return (int) data;
}


uint8_t adc_read_cmd(uint8_t ain_idx) {
  /* Use the datasheet to create the command to read a single-ended, internal ref, Ax */
  // consider asserting ain_idx < 8
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

#endif
