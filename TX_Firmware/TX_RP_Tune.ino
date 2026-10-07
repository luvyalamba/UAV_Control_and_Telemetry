// ============================================================================
//  REMOTE_TX.ino  -  Transmitter Firmware (Remote PCB)
//  Calculates and sends direct float values for Kp, Ki, Kd over RFM69.
// ============================================================================

#include <SPI.h>
#include <RFM69.h>

// -------------------- Radio Config --------------------
#define NETWORKID     100
#define NODEID        1     // Transmitter ID
#define RECEIVERID    2     // Drone Receiver ID

#define FREQUENCY     RF69_433MHZ
#define IS_RFM69HCW   true

#define CS_PIN        17
#define DIO0_PIN      25
#define RESET_PIN     5
#define SCK_PIN       18
#define MISO_PIN      19
#define MOSI_PIN      23

RFM69 radio(CS_PIN, DIO0_PIN, IS_RFM69HCW, DIO0_PIN);

// -------------------- Hardware Input Pins --------------------
#define THROTTLE_PIN  34
#define JOYX_PIN      32
#define JOYY_PIN      4
#define POT1_PIN      39   // Kp pot
#define POT2_PIN      35   // Kd pot (Swapped)
#define POT3_PIN      36   // Ki pot (Swapped)
#define SW_ARM_PIN    16   // Arm switch

// -------------------- Packet Struct (Direct Floats) --------------------
struct __attribute__((packed)) ControlPacket {
  uint16_t throttle;   // 0-1000
  int16_t  roll;       // -500..500
  int16_t  pitch;      // -500..500
  float    kp;         // Directly sent PID values
  float    kd;
  float    ki;
  uint8_t  armSwitch;  // 0 / 1
  uint16_t seq;        // Packet counter
};

ControlPacket pkt;
uint16_t seqCounter = 0;

// -------------------- Joystick Parameters --------------------
int joyXCenter = 2048;   
int joyYCenter = 2048;   
const int JOY_DEADZONE = 60;      
const int JOY_RANGE = 1500; 

// -------------------- Analog Filtering --------------------
float throttleFilt = 0, joyXFilt = 0, joyYFilt = 0;
float pot1Filt = 0, pot2Filt = 0, pot3Filt = 0;
const float SMOOTH_ALPHA = 0.25f; 

const uint32_t SEND_PERIOD_MS = 20; // 50 Hz output

// -------------------- Safe 10-Inch Tuning Bounds --------------------
const float KP_RP_MIN = 0.00f;
const float KP_RP_MAX = 4.00f;    
const float KI_RP_MIN = 0.00f;
const float KI_RP_MAX = 1.20f;
const float KD_RP_MIN = 0.00f;
const float KD_RP_MAX = 0.05f;   

void calibrateJoystick() {
  Serial.println("Calibrating joystick... Keep stick centered.");
  long sumX = 0, sumY = 0;
  const int N = 200;
  for (int i = 0; i < N; i++) {
    sumX += analogRead(JOYX_PIN);
    sumY += analogRead(JOYY_PIN);
    delay(2);
  }
  joyXCenter = sumX / N;
  joyYCenter = sumY / N;
  Serial.printf("Joystick calibrated: X=%d, Y=%d\n", joyXCenter, joyYCenter);
}

int applyDeadzone(int raw, int center, int deadzone) {
  int delta = raw - center;
  if (abs(delta) < deadzone) return 0;
  return (delta > 0) ? (delta - deadzone) : (delta + deadzone);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(SW_ARM_PIN, INPUT_PULLDOWN);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW); delay(10);
  digitalWrite(RESET_PIN, HIGH); delay(10);
  digitalWrite(RESET_PIN, LOW); delay(100);

  if (!radio.initialize(FREQUENCY, NODEID, NETWORKID)) {
    Serial.println("Radio Init Failed");
    while (1);
  }
  radio.setHighPower();
  radio.setPowerLevel(31);

  calibrateJoystick();

  throttleFilt = analogRead(THROTTLE_PIN);
  joyXFilt     = analogRead(JOYX_PIN);
  joyYFilt     = analogRead(JOYY_PIN);
  pot1Filt     = analogRead(POT1_PIN);
  pot2Filt     = analogRead(POT2_PIN);
  pot3Filt     = analogRead(POT3_PIN);

  Serial.println("TX READY");
}

void loop() {
  static uint32_t lastSend = 0;
  uint32_t now = millis();
  if (now - lastSend < SEND_PERIOD_MS) return;
  lastSend = now;

  // ---- Filter Analog Inputs ----
  throttleFilt += SMOOTH_ALPHA * (analogRead(THROTTLE_PIN) - throttleFilt);
  joyXFilt     += SMOOTH_ALPHA * (analogRead(JOYX_PIN)     - joyXFilt);
  joyYFilt     += SMOOTH_ALPHA * (analogRead(JOYY_PIN)     - joyYFilt);
  pot1Filt     += SMOOTH_ALPHA * (analogRead(POT1_PIN)     - pot1Filt);
  pot2Filt     += SMOOTH_ALPHA * (analogRead(POT2_PIN)     - pot2Filt);
  pot3Filt     += SMOOTH_ALPHA * (analogRead(POT3_PIN)     - pot3Filt);

  // ---- Map Throttle & Joysticks ----
  int rawThrottle = constrain(map((int)throttleFilt, 0, 4095, 0, 1000), 0, 1000);
  int throttleVal = (rawThrottle == 0) ? 0 : map(rawThrottle, 1, 1000, 75, 1000);

  int rollRaw  = applyDeadzone((int)joyXFilt, joyXCenter, JOY_DEADZONE);
  int pitchRaw = applyDeadzone((int)joyYFilt, joyYCenter, JOY_DEADZONE);
  int usableRange = JOY_RANGE - JOY_DEADZONE; 
  int rollVal  = constrain(map(rollRaw,  -usableRange, usableRange, -500, 500), -500, 500);
  int pitchVal = constrain(map(pitchRaw, -usableRange, usableRange, -500, 500), -500, 500);

  // ---- Map Pots (0 - 1000) with Swapped Kd / Ki Controls ----
  int pot1Val = constrain(map((int)pot1Filt, 0, 4095, 0, 1000), 0, 1000);
  int pot2Val = 1000 - constrain(map((int)pot2Filt, 0, 4095, 0, 1000), 0, 1000); // Pot 2 mapped to Kd
  int pot3Val = constrain(map((int)pot3Filt, 0, 4095, 0, 1000), 0, 1000);       // Pot 3 mapped to Ki

  // ---- Calculate Actual Float PID Values (Kd on Pot 2, Ki on Pot 3) ----
  float calculatedKp = KP_RP_MIN + (pot1Val / 1000.0f) * (KP_RP_MAX - KP_RP_MIN);
  float calculatedKd = KD_RP_MIN + (pot2Val / 1000.0f) * (KD_RP_MAX - KD_RP_MIN);
  float calculatedKi = KI_RP_MIN + (pot3Val / 1000.0f) * (KI_RP_MAX - KI_RP_MIN);

  // ---- Read Arm Switch ----
  uint8_t armSw = (digitalRead(SW_ARM_PIN) == HIGH) ? 1 : 0;

  // ---- Assemble & Send Packet ----
  pkt.throttle  = throttleVal;
  pkt.roll      = rollVal;
  pkt.pitch     = pitchVal;
  pkt.kp        = calculatedKp;
  pkt.kd        = calculatedKd;
  pkt.ki        = calculatedKi;
  pkt.armSwitch = armSw;
  pkt.seq       = seqCounter++;

  radio.send(RECEIVERID, (const void*)&pkt, sizeof(pkt));

  Serial.printf("T:%4d R:%4d P:%4d | Sent -> Kp:%.2f Kd:%.3f Ki:%.2f | ARM:%d | seq:%u\n",
                pkt.throttle, pkt.roll, pkt.pitch, pkt.kp, pkt.kd, pkt.ki, pkt.armSwitch, pkt.seq);
}