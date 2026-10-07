#include <Wire.h>
#include <U8g2lib.h>

#define BATTERY_PIN 13

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);

  analogReadResolution(12);

  u8g2.begin();
}

void loop() {

  int adcRaw = analogRead(BATTERY_PIN);

  float adcVoltage = (adcRaw * 3.3) / 4095.0;

  // Divider ratio:
  // 220k top, 470k bottom
  float batteryVoltage = adcVoltage * 1.513;

  Serial.print("ADC Raw: ");
  Serial.print(adcRaw);

  Serial.print("  Battery: ");
  Serial.print(batteryVoltage, 3);
  Serial.println(" V");

  // Approximate 1S battery percentage
  float percentage =
      (batteryVoltage - 3.3) /
      (4.2 - 3.3) *
      100.0;

  if (percentage > 100) percentage = 100;
  if (percentage < 0) percentage = 0;

  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(5, 20, "Battery");

  char voltageText[16];
  sprintf(voltageText, "%.2f V", batteryVoltage);

  u8g2.setFont(u8g2_font_logisoso18_tf);
  u8g2.drawStr(5, 48, voltageText);

  char percentText[16];
  sprintf(percentText, "%.0f%%", percentage);

  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.drawStr(90, 15, percentText);

  // Battery icon
  u8g2.drawFrame(100, 22, 24, 12);
  u8g2.drawBox(124, 25, 3, 6);

  int fillWidth = (int)(20 * percentage / 100.0);
  u8g2.drawBox(102, 24, fillWidth, 8);

  u8g2.sendBuffer();

  delay(500);
}