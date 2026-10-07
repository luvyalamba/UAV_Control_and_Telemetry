#include <SPI.h>
#include <RFM69.h>

#define NETWORKID 100
#define NODEID 1
#define RECEIVERID 2

#define FREQUENCY RF69_433MHZ
#define IS_RFM69HCW true

#define CS_PIN 17
#define DIO0_PIN 25
#define RESET_PIN 5

RFM69 radio(CS_PIN, DIO0_PIN, IS_RFM69HCW, DIO0_PIN);

void setup() {
  Serial.begin(115200);

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

  Serial.println("TX READY");
}

void loop() {

  uint8_t heartbeat = 0xAA;

  radio.send(RECEIVERID, &heartbeat, 1);

  Serial.println("Heartbeat Sent");

  delay(100);
}