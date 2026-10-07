#include <SPI.h>
#include <RF24.h>

#define CE_PIN 17
#define CSN_PIN 5
#define LED_PIN 2

RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "00001";

struct Packet {
  bool button;
};

unsigned long lastPacket = 0;

void setup() {

  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);

  SPI.begin(18, 19, 23, 5);

  if (!radio.begin()) {
    Serial.println("NRF FAILED");
    while (1);
  }

  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(120);

  radio.setAutoAck(false);

  radio.openReadingPipe(1, address);

  radio.startListening();

  Serial.println("RX READY");
}

void loop() {

  if (radio.available()) {

    Packet data;

    radio.read(&data, sizeof(data));

    lastPacket = millis();

    digitalWrite(LED_PIN, data.button);

    Serial.print("Button = ");
    Serial.println(data.button);
  }

  if (millis() - lastPacket > 500) {
    digitalWrite(LED_PIN, LOW);
  }
}