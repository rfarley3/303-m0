# 303-m0
A 303 style synth for Cortex M0 (SAM D21) like Adafruit Trinket M0.
This code should work (minus ADC knob reading) for other M0 boards like Feather 
and Metro.

Uses external ADC board (8 channel 8b ADS7830 breakout for I2C) for more knobs.
Uses M0 on-board 10b true analog DAC for audio output.
Responds to MIDI note events over Rx pin.

# Knobs
			*   A7 Button selectable, Candidates:
          * Some sort of wave shaping to make playability cover more types of music  
          * Glide {on-rate}
          * Snappy {attack transient bpdt}{expoential decay rate}
          * Saturation {distortion, overdrive, wavefolder}
          * LFO {wave, rate, send-to (>) attenuator, >pitch, >flt-cut, >vca/trem, >flt-res, >pwm}
          * Sub {wave, rel-note, detune, level, >fm amt}.
            * two directions, <64 sq, towards 0 is increasing in level/vol, >=64 saw, towards 127 is increasing in level/vol
          * Noise {wave, level}
          * Drone mode
          * Not VENV Attack, Sustain, Release as that'd be too not 303
          * Kick vol
          * Snare/hats vol
          * Decay when accent is on, as percent of min (200msec) to decay setting
          * Filter tracking
          * Filter FM

# History/Tracking
		Version road map/TODO:
      * Y verify turns on, use dotstar
      * Y dac: play tone and test speaker/line out
      * Y cut pot: respond to cut adjust, no other effects from knob
      * Y res pot on adc: respond to res adjust
          * figure out accent driven smoother circuit later (C13/Wow circuit per Devil Fish)
      * Y osc waveform selector: add pot to adc and test changing waveforms
      * Y midi input for note on/off (gate and pitch)
      * Y fix multiple notes held, etc
      * Y determine default 303 env shape
          * N/A add env shaping over cc
      * Y venv to be exp decay, with fixed long gate
          * N/A add Serial.read to send notes over TTY and not need keyboard for testing
          * Y Used a spare knob to trigger notes in a scale when turned, will be removed after testing
          * N/A add drone at max env
      * Y decay pot on adc: adjust fenv decay time, no other effects from knob
      * Y fenv
          * Y adjustable length by decay knob, but if accent_on then fixed
          * Y exp decay using lin_to_exp
      * Y env_mod pot on adc:
          * Y adjust fenv as % headroom of cut
          * Y adjusts cut lower as env_mod increased (Gimmick circuit per manual)
              * Y allows filter sweep even with cut is max
              * Y allows sweep to move through (above/at/below) cut freq instead of only above it
      * Y fix lpf integer overflow by using consistent 0..255 and then soft_clip function for tube-ish compression
      * Are all OSC in tune?
          * Notice this method for setting freq wave_saw.setFreq(freq * (float) SAW8192_SAMPLERATE / (float) SAW8192_NUM_CELLS); 
      * Add notch to remove any aliasing harmonics due to AUDIO_RATE or CONTROL_RATE
      * Y Fix VENV clicking by adding star grounding and grouping digital ground
          * Y ADC adds lots of digital noise to ground, and when volume turned down LM386's input is exposed to it
          * Y MIDI is causing lots of power fluctuations and they are also showing in ground
          * Y Shows up as a click on key press, or grown/growl/buzz on CC knob turn/aftertouch/mod/bend
          * Y For certain waves, the VENV attack is louds (namely sine/triangle)---but fix the 10% at t per FENV rate too
              * N/A really long attack, like 1000 shows that the switch to decay is the pop
              * Y move from exp of e, to 1.1ish (or 1.01) for softer decay. keep adsr always 0..255 and scale in this
      * Y Hidden HPF 
          * N/A https://www.timstinchcombe.co.uk/index.php?pge=diode2 https://www.patreon.com/posts/patch-ideas-i-tb-41917616
          * Y LPF->HPF
          * Y res increases with LPF resonance 
          * Y cut is fixed at ~10 Hz
          * TBD OSCILLOSCOPE Changes the wave shapes tri looks like sq with hump or tri on top (like the 303 sq) and sq looks like squiggly tri (like 303 tri)
          * https://olney.ai/ct-modular-book/tb-303.html
          * When I scope the square with low pass, I can replicate that url, but not for saw. Consider
            * make custom wave tables of the high-passed versions, to avoid need for HPF conflicting with resonance in LPF
            * SKIP for now, disabling HPF until after accent wah circuit done
      * Y fixing VENV curve fixed click from FENV
      * Y Restore noteoff/gate off and use 50% gate on keystep...
      * Y tested adjust env_mod to cut adj, so cut is still playable at max env_mod, but wasn't interesting
      * Y add vel threshold from noteon as accented
          * N/A alternate any input as accented or not
          * N/A add cc to turn it on and off
      * Y accent pot on adc
          * Y Boost VCA within audioUpdate by LVL_MAX/LVL_NORM=255/208 in some quick non-int way       
 HERE     * Make formula to make a duplicate (aka dup-fenv, accented fenv) of fenv (primary, from env_mod)
            * Reduced by accent knob
            * Constant value reduction from a diode
            * Smooth it more as res increases
              * If on a new note, the FENV decay != 0, then add that remainder to next FENV (wavey-steps increasing per remainder, drives cut higher and higher on fast repeat notes)
            * adapt fenv_boost to be the sum some ratio of accented and the primary
      * handle glide/legato if two notes overlap
           * glide_range is 10, https://github.com/treisti/303duino/blob/master/_303/_303.ino#L204
      * Do audio comparison tests against well known 303 sequences with glides, octaves, accent, etc
      * Check if need to smooth the adcs with a running average
          * (curr = new/16-oldest/16; append(val)), see CircularBuffer.h
          * consider Arduino Zero fast 10b adc read
      * Consider midi reads per diyelectromusic
      * Consider small sequencer
          * struct seq_note {int note, bool accent, octupdown -1,0,1, bool glide}
          * seq_note seq_notes[16];
          * easter egg of da funk 303 line
      * sub-osc: play constant sub-osc square -12
      * sub-osc waveform selector and level, fm amount
      * add cc or knob to select lfo waveform, rate
      * add mod/aft or cc or knob to do lfo send intensity/amount to pitch/OSC, cut, res, trem/VCA
      * add cc or knob to do saturation: overdrive/distortion (per daft punk), wavefolder
      * add cc or knob for VENV attack, decay sustain, release
      * add cc or knob for noise wave, level
      * add cc or knob for glide rate/speed
      * add cc or knob for attack/snappy (add 808 blip or add noise transient p-b-d-t)
      * Consider swap cut knob for button (mv cut to adc)
          * tap for tempo, press+hold for menu, tap for generative pattern, press+hold for next 
      * Is a super saw possible? or general de-tune (wave through 2 delays on LFOs and mixed for chorus/detune effect)
      * ...

per some random internet post:
There is an 'official' tuning for the VCF.
According to the Roland TB-303 manual, when playing C1 (65.4Hz) with the cutoff 
frequency knob set to 50%, saw waveform, and resonance knob at 100%, the 
resonance frequency should be 500Hz (give or take 100Hz depending on how it 
sounds to you).
You can perform this tuning by either applying 3.0VDC to the VCO or finishing 
the entire assembly and using something like Keyboard mode to trigger the note. 
"

