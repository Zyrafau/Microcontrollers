/*
wystarczyło zainteresować się bibliotekami LedControl, DateTime i Metro zamiast pisać ten program

    A
   ---
F |   | B
  | G |
   ---
E |   | C
  |   |
   ---
    D

LED PINs:

 _13_12_11__8__9__10_
[____.____.____.____.]
  2  3  4   6  5  7

*/

#define ON 0
#define OFF 1

int A = 12;
int B = 10;
int C = 6;
int D = 3;
int E = 2;
int F = 11;
int G = 5;
int dot = 4;
int D4 = 7;
int D3 = 9;
int D2 = 8;
int D1 = 13;

int h = 14;
int m = 24;
int refresh = 5;

unsigned long start;
unsigned long now;

void setup(){
  pinMode(A, OUTPUT);     
  pinMode(B, OUTPUT);     
  pinMode(C, OUTPUT);     
  pinMode(D, OUTPUT);     
  pinMode(E, OUTPUT);     
  pinMode(F, OUTPUT);     
  pinMode(G, OUTPUT);   
  pinMode(dot, OUTPUT);
  pinMode(D1, OUTPUT);  
  pinMode(D2, OUTPUT);  
  pinMode(D3, OUTPUT);  
  pinMode(D4, OUTPUT);
}

//=====================

void loop(){
  for(int j=h; j<24; j++){
    h=0;
    for(int i=m; i<60; i++){
      m=0;
      start=millis();
      now=start;
      while(now-start<59980){
        digitalWrite(dot, OFF);
        condition(i%10);
        fourth();
        delay(refresh);
    
        digitalWrite(dot, OFF);
        condition((i-i%10)/10);
        third();
        delay(refresh);

        if(now%2000<1000) digitalWrite(dot, OFF);
        else digitalWrite(dot, ON);
        condition(j%10);
        second();
        delay(refresh);

        digitalWrite(dot, OFF);
        condition((j-j%10)/10);
        first();
        delay(refresh);

        now=millis();
      }
    }
  }
}

//===================

void zero(){
  digitalWrite(A, ON);
  digitalWrite(B, ON);
  digitalWrite(C, ON);
  digitalWrite(D, ON);
  digitalWrite(E, ON);   
  digitalWrite(F, ON);   
  digitalWrite(G, OFF);
}

void one(){
  digitalWrite(A, OFF);
  digitalWrite(B, ON);
  digitalWrite(C, ON);
  digitalWrite(D, OFF);
  digitalWrite(E, OFF);
  digitalWrite(F, OFF);
  digitalWrite(G, OFF);
}

void two(){
  digitalWrite(A, ON);
  digitalWrite(B, ON);
  digitalWrite(C, OFF);
  digitalWrite(D, ON);
  digitalWrite(E, ON);
  digitalWrite(F, OFF);
  digitalWrite(G, ON);
}

void three(){
  digitalWrite(A, ON);
  digitalWrite(B, ON);
  digitalWrite(C, ON);
  digitalWrite(D, ON);
  digitalWrite(E, OFF);
  digitalWrite(F, OFF);
  digitalWrite(G, ON);
}

void four(){
  digitalWrite(A, OFF);
  digitalWrite(B, ON);
  digitalWrite(C, ON);
  digitalWrite(D, OFF);
  digitalWrite(E, OFF);
  digitalWrite(F, ON);
  digitalWrite(G, ON);
}

void five(){
  digitalWrite(A, ON);
  digitalWrite(B, OFF);
  digitalWrite(C, ON);
  digitalWrite(D, ON);
  digitalWrite(E, OFF);
  digitalWrite(F, ON);
  digitalWrite(G, ON);
}

void six(){
  digitalWrite(A, ON);
  digitalWrite(B, OFF);
  digitalWrite(C, ON);
  digitalWrite(D, ON);
  digitalWrite(E, ON);
  digitalWrite(F, ON);
  digitalWrite(G, ON);
}

void seven(){
  digitalWrite(A, ON);
  digitalWrite(B, ON);
  digitalWrite(C, ON);
  digitalWrite(D, OFF);
  digitalWrite(E, OFF);
  digitalWrite(F, OFF);
  digitalWrite(G, OFF);
}

void eight(){
  digitalWrite(A, ON);
  digitalWrite(B, ON);
  digitalWrite(C, ON);
  digitalWrite(D, ON);
  digitalWrite(E, ON);
  digitalWrite(F, ON);
  digitalWrite(G, ON);
}

void nine(){
  digitalWrite(A, ON);
  digitalWrite(B, ON);
  digitalWrite(C, ON);
  digitalWrite(D, ON);
  digitalWrite(E, OFF);
  digitalWrite(F, ON);
  digitalWrite(G, ON);
}

//====================

void first(){
  digitalWrite(D1, HIGH);   
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);   
  digitalWrite(D4, LOW);
}

void second(){
  digitalWrite(D1, LOW);   
  digitalWrite(D2, HIGH);
  digitalWrite(D3, LOW);   
  digitalWrite(D4, LOW);
}

void third(){
  digitalWrite(D1, LOW);   
  digitalWrite(D2, LOW);
  digitalWrite(D3, HIGH);   
  digitalWrite(D4, LOW);
}

void fourth(){
  digitalWrite(D1, LOW);   
  digitalWrite(D2, LOW);
  digitalWrite(D3, LOW);   
  digitalWrite(D4, HIGH);
}


void condition(int var){
  switch(var){
    case 0:
      zero();
      break;
    case 1:
      one();
      break;
    case 2:
      two();
      break;
    case 3:
      three();
      break;
    case 4:
      four();
      break;
    case 5:
      five();
      break;
    case 6:
      six();
      break;
    case 7:
      seven();
      break;
    case 8:
      eight();
      break;
    case 9:
      nine();
      break;
    }
}

