#define B 11
#define G 12
#define R 13

void pwm(int color, int ms = 1000, int sat_percent = 100, int period = 15);

void setup() {
  pinMode(R,OUTPUT);
  pinMode(G,OUTPUT);  
  pinMode(B,OUTPUT);
}

void loop() {
  pwm(R,1000,33);
  pwm(R,1000,67);
  pwm(R,1000,100);
  setWhite(5000,50);
  /*setWhite();
  delay(2000);
  setYellow();
  delay(2000);
  setCyan();
  delay(2000);
  setRed();
  delay(2000);
  setGreen();
  delay(2000);
  setBlue();
  delay(2000);*/
}

void setColor();

void setWhite(int t, int sat_percent){
  int period = 15;
  int k = t/(3*period);
  for(int i = 1; i<=k; i++){
    pwm(R,period,sat_percent,period);
    pwm(G,period,sat_percent,period);
    pwm(B,period,sat_percent,period);
  }
  delay(t%(3*period));
}

void setYellow(){
  digitalWrite(R,HIGH);
  digitalWrite(G,HIGH);
  digitalWrite(B,LOW);
}

void setCyan(){
  digitalWrite(R,LOW);
  digitalWrite(G,HIGH);
  digitalWrite(B,HIGH);
}

void setRed(){
  digitalWrite(R,HIGH);
  digitalWrite(G,LOW);
  digitalWrite(B,LOW);
}

void setGreen(){
  digitalWrite(R,LOW);
  digitalWrite(G,HIGH);
  digitalWrite(B,LOW);
}

void setBlue(){
  digitalWrite(R,LOW);
  digitalWrite(G,LOW);
  digitalWrite(B,HIGH);
}

void pwm(int color, int ms, int sat_percent, int period){
  int ton = period*sat_percent/100;
  int toff = period-ton;
  int k = ms/period;
  for(int i = 1; i<=k; i++){
    digitalWrite(color, HIGH);
    delay(ton);
    digitalWrite(color, LOW);
    delay(toff);
  }
  digitalWrite(color, HIGH);
  if(ms%period<=ton) delay(ms%period);
  else{
    delay(ton);
    digitalWrite(color, LOW);
    delay(ms%period-ton);
  }
  digitalWrite(color, LOW);
}
