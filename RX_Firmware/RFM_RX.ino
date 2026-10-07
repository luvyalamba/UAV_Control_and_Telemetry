#include <SPI.h>
#include <RFM69.h>

#define NETWORKID 100
#define NODEID 2

#define FREQUENCY RF69_433MHZ
#define IS_RFM69HCW true

#define CS_PIN 21
#define DIO0_PIN 22
#define RESET_PIN 13

#define LED_PIN 2

RFM69 radio(CS_PIN, DIO0_PIN, IS_RFM69HCW, DIO0_PIN);

unsigned long lastPacket = 0;

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  SPI.begin(18, 19, 23, CS_PIN);

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(10);
  digitalWrite(RESET_PIN, HIGH);
  delay(10);
  digitalWrite(RESET_PIN, LOW);
  delay(100);

  if (!radio.initialize(FREQUENCY, NODEID, NETWORKID)) {
    Serial.println("Radio Init Failed");
    while (1);
  }

  radio.setHighPower();
  radio.setPowerLevel(31);

  Serial.println("RX READY");
}

void loop() {

  if (radio.receiveDone()) {

    lastPacket = millis();

    digitalWrite(LED_PIN, HIGH);

    Serial.println("Packet Received");
  }

  if (millis() - lastPacket > 500) {
    digitalWrite(LED_PIN, LOW);
  }
}