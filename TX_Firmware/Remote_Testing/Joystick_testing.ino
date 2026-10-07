int joyx = 32;
int joyy = 4;

void setup() {
  pinMode(joyx, INPUT);
  pinMode(joyy, INPUT);
  Serial.begin(115200);
}

void loop() {
  int x = analogRead(joyx);
  int y = analogRead(joyy);

  Serial.print("X: ");
  Serial.print(x);
  Serial.print("  Y: ");
  Serial.println(y);

  delay(100);
}