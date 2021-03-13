#include <Servo.h>

#define TEST_DELAY 2000
#define DELTA 300

Servo serv;
int angle[] = {120,90,60}; // lewa, środkowa, prawa dioda
int fotod[3];
int covered;

void setup() {
  Serial.begin(9600);
  serv.attach(9);
  serv.write(angle[0]);
  delay(TEST_DELAY);
  serv.write(angle[1]);
  delay(TEST_DELAY);
  serv.write(angle[2]);
  delay(TEST_DELAY);
}

void loop() {
  fotod[0] = analogRead(A0); // prawa fotodioda
  fotod[1] = analogRead(A1); // środkowa fotodioda
  fotod[2] = analogRead(A2); // lewa fotodioda
  
  Serial.println(fotod[0]);
  Serial.println(fotod[1]);
  Serial.println(fotod[2]);
   
  covered = find_index(fotod);
  
  Serial.println(covered);
  Serial.println(' ');
  
  serv.write(angle[covered]);
  delay(DELTA);
}

int find_index(int fotod[]){
  int i;
  int buf = fotod[0];
  int res = 0;
  for(i = 1; i < 3; i++){
    if(fotod[i] < buf){
      buf = fotod[i];
      res = i;
    }
  }
  return res;
}

