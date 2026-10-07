#include <ESP32Servo.h>

Servo esc;

void setup() {
  esc.attach(14);

  // Arm ESC
  esc.writeMicroseconds(1000);
  delay(2000);

  // Run motor
  esc.writeMicroseconds(2000);
  delay(1000);
}

void loop() {
} 