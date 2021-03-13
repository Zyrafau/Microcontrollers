#include <Stepper.h>

#define TEST_DELAY 1000
#define DELTA 2000
#define KROKI 32
#define SET_POS (512/2)
#define TURN_LEFT -256
#define TURN_RIGHT 256

Stepper motor(KROKI,9,10,11,12);
int fotod[3];
int covered;
int pos;

void setup() {
  Serial.begin(9600);
  motor.setSpeed(500);
  //motor.step(SET_POS);
  
  motor.step(TURN_LEFT); // lewy
  delay(TEST_DELAY);

  motor.step(2*TURN_RIGHT); // prawy
  delay(TEST_DELAY);

  motor.step(TURN_LEFT);  // środek
}

void loop() {
  fotod[0] = analogRead(A0); // prawa fotodioda
  fotod[1] = analogRead(A1); // środkowa fotodioda
  fotod[2] = analogRead(A2); // lewa fotodioda
  
  Serial.println(fotod[0]);
  Serial.println(fotod[1]);
  Serial.println(fotod[2]);
   
  covered = find_index(fotod, pos);
  
  Serial.println(covered);
  Serial.println(' ');

  movement(&pos, covered);
}

int find_index(int fotod[], int pos){
  int i;
  int buf = fotod[0];
  int res = 0;
  for(i = 1; i < 3; i++){
    if(fotod[i] < buf){
      buf = fotod[i];
      res = i;
    }
  }
  if(buf < 500) return res;
  else return pos+1;
}

void movement(int *pos, int covered){
  if(*pos == -1){
    switch(covered){
      case 1:
        motor.step(TURN_RIGHT);
        *pos = 0;
        break;
      case 2:
        motor.step(2*TURN_RIGHT);
        *pos = 1;
        break;
      default:
        break;
    }
  }
  else if(*pos == 0){
      switch(covered){
      case 0:
        motor.step(TURN_LEFT);
        *pos = -1;
        break;
      case 2:
        motor.step(TURN_RIGHT);
        *pos = 1;
        break;
      default:
        break;
    }
  }
  else if(*pos == 1){
      switch(covered){
      case 0:
        motor.step(2*TURN_LEFT);
        *pos = -1;
        break;
      case 1:
        motor.step(TURN_LEFT);
        *pos = 0;
        break;
      default:
        break;
    }
  }
}
