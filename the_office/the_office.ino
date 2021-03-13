#include "notes.h"

int melody[] = {
  NOTE_G3, NOTE_G4, NOTE_FS4, NOTE_G4, NOTE_FS4, NOTE_D4, NOTE_E4, 
  NOTE_C4, 0, NOTE_C4, 0, NOTE_C4, NOTE_B3, NOTE_A3, NOTE_B3, NOTE_G3, 0, 
  NOTE_G4, NOTE_FS4, NOTE_G4, NOTE_A4, NOTE_FS4, NOTE_E4, 
  NOTE_D4, NOTE_E4, NOTE_D4, NOTE_B3, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_B3, NOTE_A3, NOTE_B3, NOTE_G3,

  NOTE_G4, NOTE_FS4, NOTE_G4, NOTE_FS4, NOTE_D4, NOTE_E4, 
  NOTE_C4, 0, NOTE_C4, 0, NOTE_C4, NOTE_B3, NOTE_A3, NOTE_B3, NOTE_G3, 0, 
  NOTE_G4, NOTE_FS4, NOTE_G4, NOTE_A4, NOTE_FS4, NOTE_E4, 
  NOTE_D4, NOTE_E4, NOTE_D4, NOTE_B3, NOTE_C4, NOTE_C4, NOTE_C4, NOTE_B3, NOTE_A3, NOTE_B3, NOTE_G3
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  2, 1, 8, 8, 8, 8, 1, 
  4, 32, 4, 32, 8, 8, 8, 8, 2, 32, 
  1, 8, 8, 8, 8, 2, 
  8, 8, 8, 10, 4, 4, 8, 8, 8, 8, 2,

  1, 8, 8, 8, 8, 1, 
  4, 32, 4, 32, 8, 8, 8, 8, 2, 32, 
  1, 8, 8, 8, 8, 2, 
  8, 8, 8, 10, 4, 4, 8, 8, 8, 8, 1
};

int noteDuration = 0;

int pauseBetweenNotes = 0;

void setup() {
  //Serial.begin(9600);
}

void loop() {
//  for(int i=0; i<31; i++){
//    tone(8, melody[i], 1000/noteDurations[i]);
//    delay(noteDurations[i]);
//    noTone(8);
//  }

  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 68; thisNote++) {

    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    noteDuration = 1000 / noteDurations[thisNote];
    tone(8, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }
}

void pwm(float freq, int t){
  float ton = 500/freq;
  int k = t*freq;
  for(int i = 1; i<=k; i++){
    digitalWrite(13, HIGH);
    delay(ton);
    digitalWrite(13, LOW);
    delay(ton);
  }
}

void bzz(float f){
  Serial.println(f);
  pwm(f,3);
  delay(100);
}

