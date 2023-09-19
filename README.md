# 303-m0
A 303 style synth for Cortex M0 (SAM D21) like Adafruit Trinket M0.
This code should work (minus ADC knob reading) for other M0 boards like Feather 
and Metro.

Uses external ADC board (8 channel 8b ADS7830 breakout for I2C) for more knobs.
Uses M0 on-board 10b true analog DAC for audio output.
Responds to MIDI note events over Rx pin.

