int slider = 34;

void setup() {
  pinMode(slider,INPUT);
  Serial.begin(115200);
}

void loop() {
  int a=analogRead(slider);
  Serial.print("Slider: ");
  Serial.println(a);
  delay(100);
}