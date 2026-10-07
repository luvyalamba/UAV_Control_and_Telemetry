int potr = 36;
int potl = 39;

void setup() {
  Serial.begin(115200);
}

void loop() {
  int l = analogRead(potl);
  int r = analogRead(potr);

  Serial.print("L: ");
  Serial.print(l);
  Serial.print("  R: ");
  Serial.println(r);

  delay(100);
}