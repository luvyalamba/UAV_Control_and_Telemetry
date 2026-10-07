#include <Wire.h>
#include <U8g2lib.h>

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

void setup() {
  Wire.begin(21, 22);

  u8g2.begin();
}

void loop() {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(15, 30, "HELLO");

  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(20, 50, "ESP32 OLED TEST");

  u8g2.sendBuffer();

  delay(100);
}