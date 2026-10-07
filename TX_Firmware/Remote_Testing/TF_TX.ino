#include <SPI.h>
#include <RF24.h>

#define CE_PIN 17
#define CSN_PIN 5
#define BUTTON_PIN 14

RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "00001";

struct Packet {
  bool button;
};

void setup() {

  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLDOWN);

  SPI.begin(18, 19, 23, 5);

  if (!radio.begin()) {
    Serial.println("NRF FAILED");
    while (1);
  }

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(50);

  radio.setAutoAck(false);

  radio.openWritingPipe(address);

  radio.stopListening();

  Serial.println("TX READY");
}

void loop() {

  Packet data;

  data.button = digitalRead(BUTTON_PIN);

  radio.write(&data, sizeof(data));

  delay(20);
}