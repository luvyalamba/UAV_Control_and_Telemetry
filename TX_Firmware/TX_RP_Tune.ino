// ============================================================================
//  REMOTE_TX.ino  -  Transmitter Firmware (Remote PCB)
//  Calculates and sends direct float values for Kp, Ki, Kd over RFM69.
//  Gains are for the RATE loop of the cascaded PID in RX_RP_Tune.
// ============================================================================

#include <SPI.h>
#include <RFM69.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

// -------------------- OLED (128x64, I2C) --------------------
// Two common chips look identical from the outside:
//   SH1106  - most 1.3" modules. Driven with the SSD1306 library they show
//             garbled / shifted text with a strip of noise at the edge.
//   SSD1306 - most 0.96" modules.
// OLED_SH1106 1 -> library "Adafruit SH110X"   (default)
// OLED_SH1106 0 -> library "Adafruit SSD1306"
// Both also need "Adafruit GFX". If the screen stays blank, try OLED_ADDR 0x3D.
#define OLED_SH1106   1
#define OLED_SDA_PIN  21
#define OLED_SCL_PIN  22
#define OLED_WIDTH    128
#define OLED_HEIGHT   64
#define OLED_ADDR     0x3C
const uint32_t OLED_PERIOD_MS = 200;   // 5 Hz: a full redraw takes ~25 ms on I2C

#if OLED_SH1106
  #include <Adafruit_SH110X.h>
  Adafruit_SH1106G oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
  #define OLED_WHITE SH110X_WHITE
  #define OLED_BLACK SH110X_BLACK
  #define OLED_BEGIN() oled.begin(OLED_ADDR, true)
#else
  #include <Adafruit_SSD1306.h>
  Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
  #define OLED_WHITE SSD1306_WHITE
  #define OLED_BLACK SSD1306_BLACK
  #define OLED_BEGIN() oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)
#endif
bool oledOk = false;

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
bool radioOk = false;   // false -> keep printing data, skip sending

// -------------------- Joystick Parameters --------------------
int joyXCenter = 2048;
int joyYCenter = 2048;
const int JOY_DEADZONE = 60;      // ADC counts around centre that read as 0

// Each side of each stick is scaled on its own, so full stick reaches +/-500
// on both sides even when the centre is not mid-scale or one side has less
// travel. The ends start at centre +/- JOY_RANGE_INIT and grow automatically
// the first time you push the stick further (move both sticks to all four
// corners once after power-up). JOY_RANGE_INIT is set a bit short so full
// stick is always reachable, even before the ends are learned.
const int JOY_RANGE_INIT = 1200;
const int JOY_END_MARGIN = 40;    // counts short of the learned end = full stick
int joyXMin, joyXMax, joyYMin, joyYMax;

// Expo: softer around centre for fine corrections, full output at the ends.
// 0 = linear, 1 = fully cubic. 0.3 is a mild curve.
const float STICK_EXPO = 0.3f;

// Roll/pitch sensitivity: full stick sends 500 * STICK_SCALE.
// 1/2 -> full stick = +/-250 = +/-15 deg on the drone (RX MAX_ANGLE_DEG 30).
const float STICK_SCALE = 1.0f / 2.0f;

// Flip a stick direction here if it moves the drone the wrong way
const bool ROLL_INVERT  = false;
const bool PITCH_INVERT = false;

// -------------------- Analog Filtering --------------------
float throttleFilt = 0, joyXFilt = 0, joyYFilt = 0;
float pot1Filt = 0, pot2Filt = 0, pot3Filt = 0;
const float SMOOTH_ALPHA = 0.25f;   // throttle and pots
const float STICK_ALPHA  = 0.45f;   // roll/pitch sticks: less lag, still smooth

const uint32_t SEND_PERIOD_MS = 20; // 50 Hz output

// -------------------- Rate-Loop Tuning Bounds --------------------
// RX now runs a cascaded controller (fixed angle P = 2 -> rate PID).
// These pots tune the inner RATE loop. working_drone defaults
// (Kp 0.6, Ki 3.5, Kd 0.03) sit near the middle of each pot.
const float KP_RP_MIN = 0.00f;
const float KP_RP_MAX = 2.00f;
const float KI_RP_MIN = 0.00f;
const float KI_RP_MAX = 8.00f;
const float KD_RP_MIN = 0.00f;
const float KD_RP_MAX = 0.08f;

// Pot shaping: 0 = linear, 1 = fully quadratic. A mild curve gives finer
// steps near zero where small gains matter most.
const float POT_EXPO = 0.5f;
const int   POT_ZERO_BAND = 15;   // pot counts (of 1000) that read as exact 0

float potToGain(int potVal, float gMin, float gMax) {
  if (potVal <= POT_ZERO_BAND) return gMin;
  float x = (potVal - POT_ZERO_BAND) / (1000.0f - POT_ZERO_BAND);
  x = (1.0f - POT_EXPO) * x + POT_EXPO * x * x;
  return gMin + x * (gMax - gMin);
}

// Startup stick-centre calibration: averages each stick for
// JOY_CAL_MS (after a short settle) so the resting drift reads as 0.
// If a stick moves during the window (spread > JOY_CAL_MAX_SPREAD) it is
// repeated, up to 3 times. Leave the sticks alone while the remote boots.
const uint32_t JOY_CAL_SETTLE_MS  = 0;     // setup() already waits 300 ms after power-up
const uint32_t JOY_CAL_MS         = 1000;
const int      JOY_CAL_MAX_SPREAD = 120;   // ADC counts

void calibrateJoystick() {
  Serial.println("Calibrating joystick centre... don't touch the sticks.");
  delay(JOY_CAL_SETTLE_MS);
  for (int attempt = 1; attempt <= 3; attempt++) {
    long sumX = 0, sumY = 0, n = 0;
    int minX = 4095, maxX = 0, minY = 4095, maxY = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < JOY_CAL_MS) {
      int x = analogRead(JOYX_PIN), y = analogRead(JOYY_PIN);
      sumX += x; sumY += y; n++;
      minX = min(minX, x); maxX = max(maxX, x);
      minY = min(minY, y); maxY = max(maxY, y);
      delay(1);
    }
    joyXCenter = sumX / n;
    joyYCenter = sumY / n;
    int spreadX = maxX - minX, spreadY = maxY - minY;
    Serial.printf("Joystick centre: X=%d (spread %d)  Y=%d (spread %d)  [%ld samples]\n",
                  joyXCenter, spreadX, joyYCenter, spreadY, n);
    if (spreadX <= JOY_CAL_MAX_SPREAD && spreadY <= JOY_CAL_MAX_SPREAD) break;
    Serial.println("  stick moved during calibration, repeating...");
  }
  joyXMin = joyXCenter - JOY_RANGE_INIT; joyXMax = joyXCenter + JOY_RANGE_INIT;
  joyYMin = joyYCenter - JOY_RANGE_INIT; joyYMax = joyYCenter + JOY_RANGE_INIT;
}

// Raw stick ADC -> -500..500 with per-side scaling, deadzone and expo.
// Also learns the stick ends.
int stickToCommand(float rawF, int center, int &lo, int &hi, bool invert) {
  int raw = (int)rawF;
  if (raw < lo) lo = raw;
  if (raw > hi) hi = raw;

  int delta = raw - center;
  if (abs(delta) < JOY_DEADZONE) return 0;

  float span = (delta > 0) ? (hi - center - JOY_DEADZONE - JOY_END_MARGIN)
                           : (center - lo - JOY_DEADZONE - JOY_END_MARGIN);
  if (span < 100) span = 100;
  float x = (abs(delta) - JOY_DEADZONE) / span;
  x = constrain(x, 0.0f, 1.0f);
  x = (1.0f - STICK_EXPO) * x + STICK_EXPO * x * x * x;
  int out = (int)(x * 500.0f * STICK_SCALE + 0.5f);
  if (delta < 0) out = -out;
  return invert ? -out : out;
}

void oledSetup() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(400000);
  oledOk = OLED_BEGIN();
  if (!oledOk) { Serial.println("OLED not found - continuing without it"); return; }
  oled.setTextWrap(false);
  oled.clearDisplay();
  oled.setTextColor(OLED_WHITE, OLED_BLACK);
  oled.setTextSize(2);
  oled.setCursor(0, 0);  oled.print("TX");
  oled.setTextSize(1);
  oled.setCursor(0, 24); oled.print("Calibrating sticks");
  oled.setCursor(0, 36); oled.print("don't touch...");
  oled.display();
}

// Layout (128x64):
//   y 0  small: SAFE / ARMED / THR DOWN!   + radio
//   y 9  small: T throttle  R roll  P pitch (as sent)
//   y18  big:   P 0.500           pot %
//   y34  big:   I 1.50            pot %
//   y50  big:   D 0.0200          pot %
// Called at 5 Hz from loop(), right after a send.
void oledUpdate(int potKp, int potKi, int potKd) {
  if (!oledOk) return;
  char buf[24];
  oled.clearDisplay();

  oled.setTextSize(1);
  oled.setCursor(0, 0);
  if (pkt.armSwitch && pkt.throttle >= 30) oled.print("THR DOWN!");   // RX arms only at low throttle
  else                                     oled.print(pkt.armSwitch ? "ARMED" : "SAFE");
  oled.setCursor(80, 0);
  oled.print(radioOk ? "RADIO OK" : "NO RADIO");

  snprintf(buf, sizeof(buf), "T%4d R%4d P%4d", pkt.throttle, pkt.roll, pkt.pitch);
  oled.setCursor(0, 9);
  oled.print(buf);

  oled.setTextSize(2);
  snprintf(buf, sizeof(buf), "P %5.3f", pkt.kp);
  oled.setCursor(0, 18); oled.print(buf);
  snprintf(buf, sizeof(buf), "I %5.2f", pkt.ki);
  oled.setCursor(0, 34); oled.print(buf);
  snprintf(buf, sizeof(buf), "D %6.4f", pkt.kd);
  oled.setCursor(0, 50); oled.print(buf);

  // pot positions 0-100 %, small, right-aligned
  oled.setTextSize(1);
  snprintf(buf, sizeof(buf), "%3d%%", potKp / 10); oled.setCursor(104, 22); oled.print(buf);
  snprintf(buf, sizeof(buf), "%3d%%", potKi / 10); oled.setCursor(104, 38); oled.print(buf);
  snprintf(buf, sizeof(buf), "%3d%%", potKd / 10); oled.setCursor(104, 54); oled.print(buf);

  oled.display();
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

  radioOk = radio.initialize(FREQUENCY, NODEID, NETWORKID);
  if (radioOk) {
    radio.setHighPower();
    radio.setPowerLevel(31);
  } else {
    Serial.println("Radio Init Failed - printing inputs only, nothing is sent");
  }

  oledSetup();
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
  static uint32_t lastSendUs = 0;   // how long the previous radio.send() took
  uint32_t now = millis();
  if (now - lastSend < SEND_PERIOD_MS) return;
  lastSend = now;

  // ---- Filter Analog Inputs ----
  throttleFilt += SMOOTH_ALPHA * (analogRead(THROTTLE_PIN) - throttleFilt);
  joyXFilt     += STICK_ALPHA  * (analogRead(JOYX_PIN)     - joyXFilt);
  joyYFilt     += STICK_ALPHA  * (analogRead(JOYY_PIN)     - joyYFilt);
  pot1Filt     += SMOOTH_ALPHA * (analogRead(POT1_PIN)     - pot1Filt);
  pot2Filt     += SMOOTH_ALPHA * (analogRead(POT2_PIN)     - pot2Filt);
  pot3Filt     += SMOOTH_ALPHA * (analogRead(POT3_PIN)     - pot3Filt);

  // ---- Map Throttle & Joysticks ----
  int rawThrottle = constrain(map((int)throttleFilt, 0, 4095, 0, 1000), 0, 1000);
  int throttleVal = (rawThrottle == 0) ? 0 : map(rawThrottle, 1, 1000, 75, 1000);

  int rollVal  = stickToCommand(joyXFilt, joyXCenter, joyXMin, joyXMax, ROLL_INVERT);
  int pitchVal = stickToCommand(joyYFilt, joyYCenter, joyYMin, joyYMax, PITCH_INVERT);

  // Learned stick ends, every 2 s (full stick = these minus JOY_END_MARGIN)
  static uint32_t lastEndsPrint = 0;
  if (now - lastEndsPrint >= 2000) {
    lastEndsPrint = now;
    Serial.printf("STICK ENDS X:%d..%d (c %d)  Y:%d..%d (c %d)\n",
                  joyXMin, joyXMax, joyXCenter, joyYMin, joyYMax, joyYCenter);
  }

  // ---- Map Pots (0 - 1000) with Swapped Kd / Ki Controls ----
  int pot1Val = constrain(map((int)pot1Filt, 0, 4095, 0, 1000), 0, 1000);
  int pot2Val = 1000 - constrain(map((int)pot2Filt, 0, 4095, 0, 1000), 0, 1000); // Pot 2 mapped to Kd
  int pot3Val = constrain(map((int)pot3Filt, 0, 4095, 0, 1000), 0, 1000);       // Pot 3 mapped to Ki

  // ---- Calculate Actual Float PID Values (Kd on Pot 2, Ki on Pot 3) ----
  float calculatedKp = potToGain(pot1Val, KP_RP_MIN, KP_RP_MAX);
  float calculatedKd = potToGain(pot2Val, KD_RP_MIN, KD_RP_MAX);
  float calculatedKi = potToGain(pot3Val, KI_RP_MIN, KI_RP_MAX);

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

  // ---- Continuous Debug Print (every 20 ms, before the send) ----
  // raw = filtered ADC counts (0-4095), pot = position 0-1000 after Kd inversion
  Serial.printf("T:%4d R:%4d P:%4d | Kp:%.3f Ki:%.2f Kd:%.4f | ARM:%d | seq:%5u | "
                "raw T:%4d X:%4d Y:%4d P1:%4d P2:%4d P3:%4d | pot Kp:%4d Ki:%4d Kd:%4d | %s:%luus\n",
                pkt.throttle, pkt.roll, pkt.pitch, pkt.kp, pkt.ki, pkt.kd, pkt.armSwitch, pkt.seq,
                (int)throttleFilt, (int)joyXFilt, (int)joyYFilt,
                (int)pot1Filt, (int)pot2Filt, (int)pot3Filt,
                pot1Val, pot3Val, pot2Val,
                radioOk ? "tx" : "NO-RADIO", (unsigned long)lastSendUs);

  if (radioOk) {
    uint32_t t0 = micros();
    radio.send(RECEIVERID, (const void*)&pkt, sizeof(pkt));
    lastSendUs = micros() - t0;
  }

  // ---- OLED (5 Hz, after the send so the packet goes out on time) ----
  static uint32_t lastOled = 0;
  if (now - lastOled >= OLED_PERIOD_MS) {
    lastOled = now;
    oledUpdate(pot1Val, pot3Val, pot2Val);
  }
}
