// ============================================================================
//  RX_RP_Tune_FF.ino  -  Flight Controller for the 10 inch quad
//  (363 mm DJI-style frame, 2212 1000kv, 30A ESCs, 3S, MPU9250 SPI, RFM69)
//
//  Cascaded attitude controller with TX gain tuning.
//
//  Based on the Carbon Aeronautics quadcopter flight controller (v1.0, 2022):
//  its pid_equation, cascaded angle -> rate loops, gains and mixer. That work
//  is licensed CC BY-NC-SA 4.0, so this file is too: credit Carbon
//  Aeronautics, non-commercial use only, share changes under the same
//  licence. Carbon Aeronautics does not endorse this version.
// ============================================================================

#include <SPI.h>
#include <RFM69.h>
#include <ESP32Servo.h>

// -------------------- Radio Config --------------------
#define NETWORKID     100
#define NODEID        2     // Drone ID
#define TX_ID         1     // Transmitter ID

#define FREQUENCY     RF69_433MHZ
#define IS_RFM69HCW   true

#define RFM_CS_PIN    21
#define RFM_DIO0_PIN  22
#define RFM_RESET_PIN 13
#define SCK_PIN       18
#define MISO_PIN      19
#define MOSI_PIN      23

RFM69 radio(RFM_CS_PIN, RFM_DIO0_PIN, IS_RFM69HCW, RFM_DIO0_PIN);

// -------------------- MPU9250 Registers --------------------
#define MPU_CS_PIN     5
#define SMPLRT_DIV     0x19
#define CONFIG         0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_CONFIG2  0x1D
#define ACCEL_XOUT_H   0x3B
#define USER_CTRL      0x6A
#define PWR_MGMT_1     0x6B
#define WHO_AM_I       0x75

// --- 1 kHz loop ---
const uint32_t SAMPLE_PERIOD_US = 1000UL;

const float GYRO_SCALE  = 65.5f;    // +/-500 dps
const float ACCEL_SCALE = 4096.0f;  // +/-8g

SPISettings spiSlow(1000000, MSBFIRST, SPI_MODE0);
SPISettings spiFast(8000000, MSBFIRST, SPI_MODE0);

const uint8_t GYRO_DLPF_CFG  = 0x03;
const uint8_t ACCEL_DLPF_CFG = 0x05;

// ---- Level trim -------------------------------------------------------------
const float ACC_TRIM_X_G      = -0.02f;   // added to ax (g)
const float ACC_TRIM_Y_G      = -0.01f;   // added to ay (g)
const float LEVEL_TRIM_ROLL   = -0.07f;   // deg, subtracted from roll
const float LEVEL_TRIM_PITCH  =  1.24f;   // deg, subtracted from pitch

float gxBias = 0, gyBias = 0, gzBias = 0;
float axBias = 0, ayBias = 0, azBias = 0;

// -------------------- 1D Kalman Filter --------------------

struct Kalman1D {
  float angle;
  float uncertainty;
  void update(float rate, float measurement, float dt) {
    (void)rate; (void)dt;
    angle = measurement;
  }
  void reset(float startAngle) { angle = startAngle; uncertainty = 4.0f; }
};

Kalman1D kalmanRoll  = { 0.0f, 4.0f };
Kalman1D kalmanPitch = { 0.0f, 4.0f };

float accelRollDeg(float ax, float ay, float az) {
  return atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
}
float accelPitchDeg(float ax, float ay, float az) {
  return -atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

// -------------------- ESC Outputs --------------------
#define ESC_FL_PIN 25
#define ESC_FR_PIN 26
#define ESC_RL_PIN 32
#define ESC_RR_PIN 33

Servo escFL, escFR, escRL, escRR;

const int ESC_MIN_US  = 1000;
const int ESC_MAX_US  = 2000;
const int ESC_RATE_HZ = 400;

// -------------------- Packet Structure (same as TX) --------------------
struct __attribute__((packed)) ControlPacket {
  uint16_t throttle;
  int16_t  roll;
  int16_t  pitch;
  float    kp;
  float    kd;
  float    ki;
  uint8_t  armSwitch;
  uint16_t seq;
};

ControlPacket pkt;
volatile uint32_t lastPacketMs = 0;
const uint32_t FAILSAFE_TIMEOUT_MS = 300;
const float MAX_ANGLE_DEG = 30.0f;      // full stick = 30 deg

bool armed = false;
const int ARM_THROTTLE_THRESHOLD = 30;
const int MIN_THROTTLE_FOR_PID   = 60;   // P, D, FF on from here (pkt units 0-1000)
const int I_ENABLE_THROTTLE      = 300;

const int THROTTLE_MAX_US = 1800;       // base throttle cap: leaves room for PID
const int ESC_IDLE_US     = 1100;       // flying: no motor below this

// -------------------- Controller Gains --------------------
const float ANGLE_KP = 4.0f;
const float ANGLE_FF = 0.5f;

const float RATE_KP_DEFAULT = 0.6f;
const float RATE_KI_DEFAULT = 3.5f;
const float RATE_KD_DEFAULT = 0.03f;
const float RATE_FF = 0.08f;

const bool USE_POT_GAINS = true;        // false = ignore pots, use defaults

const float YAW_KP = 2.0f;
const float YAW_KI = 12.0f;
const float YAW_KD = 0.0f;

const float RATE_OUT_US   = 400.0f;
const float RATE_I_US     = 300.0f;
const float YAW_OUT_US    = 200.0f;     // keep yaw from eating roll/pitch authority
const float ANGLE_OUT_DPS = 250.0f;     // max rate the angle loop may ask for
const float RATE_D_LPF_HZ = 30.0f;      // D-term filter
const float ANGLE_FF_LPF_HZ = 15.0f;    // smooths the 50 Hz stick steps

// -------------------- PidFF (PID + Feedforward) --------------------
enum FFMode : uint8_t { FF_SETPOINT = 0, FF_SETPOINT_RATE = 1 };

struct PidFF {
  float kp = 0, ki = 0, kd = 0, ff = 0;
  float outLimit = 400, iLimit = 400;
  FFMode ffMode = FF_SETPOINT;
  float dLpfHz = 0, ffLpfHz = 0;

  float P = 0, I = 0, D = 0, F = 0, out = 0;   // last terms, for prints

  float prevError = 0, prevMeas = 0, prevSet = 0;
  float dFilt = 0, setRateFilt = 0;
  bool  first = true;
  float cacheDt = -1, dAlpha = 1, ffAlpha = 1;

  void configure(float outLim, float iLim, FFMode mode, float dHz, float ffHz) {
    outLimit = outLim; iLimit = iLim; ffMode = mode; dLpfHz = dHz; ffLpfHz = ffHz;
    cacheDt = -1;
  }
  void setGains(float p, float i, float d, float f) { kp = p; ki = i; kd = d; ff = f; }
  void reset() {
    I = 0; prevError = 0; dFilt = 0; setRateFilt = 0; first = true;
    P = D = F = out = 0;
  }

  float update(float setpoint, float measured, float dt, bool iHold) {
    return run(setpoint, measured, dt, false, 0, iHold);
  }
  float update(float setpoint, float measured, float dt, float measuredRate, bool iHold) {
    return run(setpoint, measured, dt, true, measuredRate, iHold);
  }

 private:
  static float lpfAlpha(float hz, float dt) {
    if (hz <= 0) return 1.0f;
    float w = 2.0f * PI * hz * dt;
    return w / (1.0f + w);
  }

  float run(float setpoint, float measured, float dt, bool haveRate, float measuredRate, bool iHold) {
    (void)dt; (void)haveRate; (void)measuredRate; (void)iHold;
    float error = setpoint - measured;
    P = kp * error;
    I = D = F = 0;
    out = constrain(P, -outLimit, outLimit);
    return out;
  }
};

PidFF pidAngleRoll, pidAnglePitch;           // deg   -> deg/s
PidFF pidRateRoll, pidRatePitch, pidRateYaw; // deg/s -> us

void setupPid() {
  pidAngleRoll.configure (ANGLE_OUT_DPS, 0,         FF_SETPOINT_RATE, 0,             ANGLE_FF_LPF_HZ);
  pidAnglePitch.configure(ANGLE_OUT_DPS, 0,         FF_SETPOINT_RATE, 0,             ANGLE_FF_LPF_HZ);
  pidRateRoll.configure  (RATE_OUT_US,   RATE_I_US, FF_SETPOINT,      RATE_D_LPF_HZ, 0);
  pidRatePitch.configure (RATE_OUT_US,   RATE_I_US, FF_SETPOINT,      RATE_D_LPF_HZ, 0);
  pidRateYaw.configure   (YAW_OUT_US,    RATE_I_US, FF_SETPOINT,      RATE_D_LPF_HZ, 0);

  pidAngleRoll.setGains (ANGLE_KP, 0, 0, ANGLE_FF);
  pidAnglePitch.setGains(ANGLE_KP, 0, 0, ANGLE_FF);
  pidRateRoll.setGains  (RATE_KP_DEFAULT, RATE_KI_DEFAULT, RATE_KD_DEFAULT, RATE_FF);
  pidRatePitch.setGains (RATE_KP_DEFAULT, RATE_KI_DEFAULT, RATE_KD_DEFAULT, RATE_FF);
  pidRateYaw.setGains   (YAW_KP, YAW_KI, YAW_KD, 0);
}

void resetAllPid() {
  pidAngleRoll.reset(); pidAnglePitch.reset();
  pidRateRoll.reset();  pidRatePitch.reset(); pidRateYaw.reset();
}

// -------------------- MPU9250 Functions --------------------
void writeRegister(uint8_t reg, uint8_t value) {
  digitalWrite(RFM_CS_PIN, HIGH);
  SPI.beginTransaction(spiSlow);
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t readRegister(uint8_t reg) {
  digitalWrite(RFM_CS_PIN, HIGH);
  SPI.beginTransaction(spiSlow);
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();
  return val;
}

void readBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
  digitalWrite(RFM_CS_PIN, HIGH);
  SPI.beginTransaction(spiFast);
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();
}

bool verifySensor() {
  for (int attempt = 0; attempt < 10; attempt++) {
    uint8_t who = readRegister(WHO_AM_I);
    if (who == 0x71 || who == 0x73 || who == 0x70 || who == 0x68) return true;
    delay(50);
  }
  return false;
}

void readSensors(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  uint8_t raw[14];
  readBurst(ACCEL_XOUT_H, raw, 14);

  int16_t rax = (raw[0] << 8) | raw[1];
  int16_t ray = (raw[2] << 8) | raw[3];
  int16_t raz = (raw[4] << 8) | raw[5];
  int16_t rgx = (raw[8] << 8) | raw[9];
  int16_t rgy = (raw[10] << 8) | raw[11];
  int16_t rgz = (raw[12] << 8) | raw[13];

  ax = rax / ACCEL_SCALE; ay = ray / ACCEL_SCALE; az = raz / ACCEL_SCALE;
  gx = rgx / GYRO_SCALE;  gy = rgy / GYRO_SCALE;  gz = rgz / GYRO_SCALE;
}

void correctAccel(float &ax, float &ay, float &az) {
  ax += axBias + ACC_TRIM_X_G;
  ay += ayBias + ACC_TRIM_Y_G;
  az += azBias;
}

void calibrateSensors() {
  const int samples = 1000;
  float sg[3] = {0, 0, 0}, sa[3] = {0, 0, 0};
  Serial.println("Calibrating IMU... Keep drone still and level.");

  for (int i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz;
    readSensors(ax, ay, az, gx, gy, gz);
    sg[0] += gx; sg[1] += gy; sg[2] += gz;
    sa[0] += ax; sa[1] += ay; sa[2] += az;
    delayMicroseconds(2000);
  }

  gxBias = sg[0] / samples; gyBias = sg[1] / samples; gzBias = sg[2] / samples;
  float axM = sa[0] / samples, ayM = sa[1] / samples, azM = sa[2] / samples;
  float mag = sqrtf(axM * axM + ayM * ayM + azM * azM);
  float k = (mag > 0.0001f) ? (1.0f / mag) : 1.0f;
  axBias = axM * k - axM; ayBias = ayM * k - ayM; azBias = azM * k - azM;

  float ax = axM, ay = ayM, az = azM;
  correctAccel(ax, ay, az);
  float r = accelRollDeg(ax, ay, az), p = accelPitchDeg(ax, ay, az);
  kalmanRoll.reset(r);
  kalmanPitch.reset(p);

  Serial.printf("Gyro bias  X:%.2f Y:%.2f Z:%.2f deg/s\n", gxBias, gyBias, gzBias);
  Serial.printf("IMU level (before LEVEL_TRIM): roll %.2f pitch %.2f deg\n", r, p);
  Serial.printf("  if the drone is LEVEL, paste into the sketch:\n"
                "  const float LEVEL_TRIM_ROLL   = %.2ff;\n"
                "  const float LEVEL_TRIM_PITCH  = %.2ff;\n", r, p);
}

// -------------------- ESC helpers --------------------
void writeAllEscs(int us) {
  us = constrain(us, ESC_MIN_US, ESC_MAX_US);
  escFL.writeMicroseconds(us); escFR.writeMicroseconds(us);
  escRL.writeMicroseconds(us); escRR.writeMicroseconds(us);
}

void disarm() {
  armed = false;
  resetAllPid();
  writeAllEscs(ESC_MIN_US);
}

void setup() {
  Serial.setTxBufferSize(1024);   // telemetry must never block the 1 kHz loop
  Serial.begin(115200);
  delay(500);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  escFL.setPeriodHertz(ESC_RATE_HZ); escFR.setPeriodHertz(ESC_RATE_HZ);
  escRL.setPeriodHertz(ESC_RATE_HZ); escRR.setPeriodHertz(ESC_RATE_HZ);
  escFL.attach(ESC_FL_PIN, ESC_MIN_US, ESC_MAX_US); escFR.attach(ESC_FR_PIN, ESC_MIN_US, ESC_MAX_US);
  escRL.attach(ESC_RL_PIN, ESC_MIN_US, ESC_MAX_US); escRR.attach(ESC_RR_PIN, ESC_MIN_US, ESC_MAX_US);
  writeAllEscs(ESC_MIN_US);

  pinMode(MPU_CS_PIN, OUTPUT); pinMode(RFM_CS_PIN, OUTPUT);
  digitalWrite(MPU_CS_PIN, HIGH); digitalWrite(RFM_CS_PIN, HIGH);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
  writeRegister(PWR_MGMT_1, 0x01); delay(10);   // wake, PLL clock
  writeRegister(USER_CTRL, 0x10);               // I2C_IF_DIS: SPI only

  if (!verifySensor()) {
    Serial.println("FATAL: MPU9250 not responding.");
    while (true) delay(1000);
  }

  writeRegister(SMPLRT_DIV, 0);
  writeRegister(CONFIG, GYRO_DLPF_CFG);
  writeRegister(GYRO_CONFIG, 0x08);             // +/-500 dps, DLPF on
  writeRegister(ACCEL_CONFIG, 0x10);            // +/-8g
  writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_CFG);
  delay(50);

  uint8_t c1 = readRegister(CONFIG), c2 = readRegister(GYRO_CONFIG);
  uint8_t c3 = readRegister(ACCEL_CONFIG), c4 = readRegister(ACCEL_CONFIG2);
  Serial.printf("MPU CONFIG=0x%02X GYRO_CONFIG=0x%02X ACCEL_CONFIG=0x%02X ACCEL_CONFIG2=0x%02X\n", c1, c2, c3, c4);
  if (c1 != GYRO_DLPF_CFG || c2 != 0x08 || c3 != 0x10 || (c4 & 0x0F) != ACCEL_DLPF_CFG)
    Serial.println("WARNING: MPU9250 config readback does not match - check SPI wiring");

  calibrateSensors();

  pinMode(RFM_RESET_PIN, OUTPUT);
  digitalWrite(RFM_RESET_PIN, LOW); delay(10);
  digitalWrite(RFM_RESET_PIN, HIGH); delay(10);
  digitalWrite(RFM_RESET_PIN, LOW); delay(100);

  if (!radio.initialize(FREQUENCY, NODEID, NETWORKID)) {
    Serial.println("Radio Init Failed");
    while (1) delay(1000);
  }
  radio.setHighPower(); radio.setPowerLevel(31);

  setupPid();

  Serial.println("Arming ESCs...");
  writeAllEscs(ESC_MIN_US);
  delay(2000);

  lastPacketMs = millis();
  Serial.printf("DRONE READY (1 kHz PID+FF, ESC %d Hz, angle P %.1f, rate FF %.2f)\n",
                ESC_RATE_HZ, ANGLE_KP, RATE_FF);
}

void applyTuning() {
  if (!USE_POT_GAINS) return;
  pidRateRoll.kp = pkt.kp;   pidRatePitch.kp = pkt.kp;   // Pot 1
  pidRateRoll.kd = pkt.kd;   pidRatePitch.kd = pkt.kd;   // Pot 2
  pidRateRoll.ki = pkt.ki;   pidRatePitch.ki = pkt.ki;   // Pot 3
}

void handleArming() {
  bool switchOn    = (pkt.armSwitch == 1);
  bool throttleLow = (pkt.throttle < ARM_THROTTLE_THRESHOLD);

  if (!armed && switchOn && throttleLow) {
    armed = true; resetAllPid();
    Serial.println("ARMED");
  } else if (armed && !switchOn) {
    disarm(); Serial.println("DISARMED (switch)");
  }
}

void loop() {
  if (radio.receiveDone()) {
    if (radio.DATALEN == sizeof(ControlPacket)) {
      memcpy(&pkt, (const void*)radio.DATA, sizeof(ControlPacket));
      lastPacketMs = millis();
    }
    if (radio.ACKRequested()) radio.sendACK();
  }

  if (millis() - lastPacketMs > FAILSAFE_TIMEOUT_MS) {
    if (armed) Serial.println("DISARMED (failsafe - link lost)");
    disarm();
  }

  handleArming();
  applyTuning();

  // --- 1 kHz loop timing ---
  static uint32_t lastSample = micros();
  uint32_t now = micros();
  if ((uint32_t)(now - lastSample) < SAMPLE_PERIOD_US) return;
  float dt = (now - lastSample) / 1000000.0f;
  lastSample = now;
  dt = constrain(dt, 0.0005f, 0.004f);   // a stalled loop must not blow up I/D

  float ax, ay, az, gx, gy, gz;
  readSensors(ax, ay, az, gx, gy, gz);
  correctAccel(ax, ay, az);
  gx -= gxBias; gy -= gyBias; gz -= gzBias;

  const float rateRoll = gx, ratePitch = gy, rateYaw = gz;

  kalmanRoll.update(rateRoll,   accelRollDeg(ax, ay, az),  dt);
  kalmanPitch.update(ratePitch, accelPitchDeg(ax, ay, az), dt);

  float currentRoll  = kalmanRoll.angle  - LEVEL_TRIM_ROLL;
  float currentPitch = kalmanPitch.angle - LEVEL_TRIM_PITCH;

  if (!armed) {
    writeAllEscs(ESC_MIN_US);
    static uint32_t lastIdlePrint = 0;
    if (millis() - lastIdlePrint >= 500) {
      lastIdlePrint = millis();
      uint32_t age = millis() - lastPacketMs;
      Serial.printf("DISARMED | link %s (last pkt %lu ms ago, seq %u) | T:%4u R:%4d P:%4d ARM-SW:%u | "
                    "K P%.2f I%.2f D%.4f FF%.2f | ANG R%5.1f P%5.1f\n",
                    age <= FAILSAFE_TIMEOUT_MS ? "OK" : "LOST", (unsigned long)age, pkt.seq,
                    pkt.throttle, pkt.roll, pkt.pitch, pkt.armSwitch,
                    pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd, pidRateRoll.ff,
                    currentRoll, currentPitch);
      if (pkt.armSwitch == 1 && pkt.throttle >= ARM_THROTTLE_THRESHOLD)
        Serial.println("  -> lower the throttle fully to arm");
    }
    return;
  }

  float rollSetpoint  = (pkt.roll  / 500.0f) * MAX_ANGLE_DEG;
  float pitchSetpoint = (pkt.pitch / 500.0f) * MAX_ANGLE_DEG;

  float escBaseUs = map(pkt.throttle, 0, 1000, ESC_MIN_US, ESC_MAX_US);
  if (escBaseUs > THROTTLE_MAX_US) escBaseUs = THROTTLE_MAX_US;

  float rollOut = 0, pitchOut = 0, yawOut = 0;
  float desiredRateRoll = 0, desiredRatePitch = 0;
  bool pidActive = (pkt.throttle >= MIN_THROTTLE_FOR_PID);
  bool iHold     = (pkt.throttle <  I_ENABLE_THROTTLE);   // on the ground

  if (pidActive) {
    desiredRateRoll  = pidAngleRoll.update (rollSetpoint,  currentRoll,  dt, rateRoll,  true);
    desiredRatePitch = pidAnglePitch.update(pitchSetpoint, currentPitch, dt, ratePitch, true);

    rollOut  = pidRateRoll.update (desiredRateRoll,  rateRoll,  dt, iHold);
    pitchOut = pidRatePitch.update(desiredRatePitch, ratePitch, dt, iHold);
    yawOut   = pidRateYaw.update  (0.0f,             rateYaw,   dt, iHold);
  } else {
    resetAllPid();
  }

  // --- MOTOR MIXING ---
  float fl = escBaseUs - pitchOut + rollOut + yawOut;
  float fr = escBaseUs - pitchOut - rollOut - yawOut;
  float rl = escBaseUs + pitchOut + rollOut - yawOut;
  float rr = escBaseUs + pitchOut - rollOut + yawOut;

  writeAllEscs(ESC_MIN_US);

  // --- TELEMETRY (10 Hz) ---
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.printf("ANG R%5.1f/%5.1f P%5.1f/%5.1f | RATE R%6.1f/%6.1f P%6.1f/%6.1f Y%6.1f | "
                  "OUT R%4.0f P%4.0f Y%4.0f | I R%4.0f P%4.0f | K P%.2f I%.2f D%.4f FF%.2f | "
                  "M %4d %4d %4d %4d | dt%.4f\n",
                  currentRoll, rollSetpoint, currentPitch, pitchSetpoint,
                  rateRoll, desiredRateRoll, ratePitch, desiredRatePitch, rateYaw,
                  rollOut, pitchOut, yawOut, pidRateRoll.I, pidRatePitch.I,
                  pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd, pidRateRoll.ff,
                  (int)fl, (int)fr, (int)rl, (int)rr, dt);
  }
}
