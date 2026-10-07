int switchL = 16;
int switchR = 35;

void setup() {
  pinMode(switchL, INPUT_PULLDOWN);
  pinMode(switchR, INPUT_PULLDOWN);
  Serial.begin(115200);
}

void loop() {
  int L = digitalRead(switchL);
  int R = digitalRead(switchR);

  Serial.print("L: ");
  Serial.print(L);
  Serial.print("  R: ");
  Serial.println(R);

  delay(100);
}