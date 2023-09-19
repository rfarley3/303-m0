#ifndef synth303_midi_ino
#define synth303_midi_ino
/*  Create a 303-ish synth with potentiometer controllers.
 *  (c) Ryan Farley rfarley3@github
 */
#include <MIDI.h>

// Set the MIDI Channel to listen on
#define MIDI_RATE    16    // Cycles through the loop() function
#define MIDI_LED LED_BUILTIN // shows if MIDI is being recieved
int midicount = 0;
MIDI_CREATE_DEFAULT_INSTANCE();

#define MIDI_NOTE_CNT 128
int note_on[MIDI_NOTE_CNT];
int note_on_order = 0;

#define ENABLE_MIDI 1  // test speed up, use only the pot


void midi_setup () {
  pinMode(MIDI_LED, OUTPUT);

  // Connect the HandleNoteOn function to the library, so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(HandleNoteOn);  // Put only the name of the function
  MIDI.setHandleNoteOff(HandleNoteOff);  // Put only the name of the function
  if (ENABLE_MIDI) { MIDI.begin(MIDI_CHANNEL); }
  init_note_on();
}


void init_note_on () {
  /* we track the note on order within this, with one element per possible midi note value */
  for (int i = 0; i < MIDI_NOTE_CNT; i++) {
    note_on[i] = -1;
  }
}


void midiHook () {
  /* we don't need to check MIDI every loop, here's a sane amount
   *  and we don't want it in control loop as that is too slow/laggy
   */
  midicount++;
  if (midicount > MIDI_RATE) {
    if (ENABLE_MIDI) { MIDI.read(); }
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
  // callee will check if there is an actual change
  note_change(0, note);
  // if venv is in ADS, then let it finish, determined in trigger_env
  if (trigger_env(0)) {
    digitalWrite(MIDI_LED, HIGH);
  }
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
  // note_on_order > 1. There are fallbacks, find it
  // handle press 1, 2, 3, let go of 2, then 3 should still play, let go of 3, then 1 should be playing
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

#endif
