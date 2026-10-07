void setup() {
  pinMode(33,OUTPUT);
  pinMode(25,OUTPUT);
  pinMode(27,OUTPUT);
}

void loop() {
  for(int i=0;i<=255;i++){
    digitalWrite(27,0);
    digitalWrite(25,0);
    analogWrite(33,i);
    delay(4);
  }
  for(int i=0;i<=255;i++){
    digitalWrite(27,0);
    digitalWrite(33,0);
    analogWrite(25,i);
    delay(4);
  }
  for(int i=0;i<=255;i++){
    digitalWrite(33,0);
    digitalWrite(25,0);
    analogWrite(27,i);
    delay(4);
  }
}
