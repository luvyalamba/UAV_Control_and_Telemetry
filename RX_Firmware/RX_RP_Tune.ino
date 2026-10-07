// ============================================================================
//  DRONE_RX.ino  -  Flight Controller Firmware (1 kHz Loop + Digital Filter)
//  Reads MPU9250 over 8MHz SPI, processes attitude via Madgwick AHRS at 1kHz,
//  filters high-frequency gyro noise, updates PID, and drives ESCs.
// ============================================================================

#include <SPI.h>
#include <RFM69.h>
#include <MadgwickAHRS.h>
#include <ESP32Servo.h>

Madgwick filter;

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
#define PWR_MGMT_1     0x6B
#define WHO_AM_I       0x75

// --- 1 kHz TIMING CONFIGURATION ---
const float SAMPLE_RATE = 1000.0f; 
const uint32_t SAMPLE_PERIOD_US = 1000UL; 

const float GYRO_SCALE  = 65.5f;    // +/-500 dps
const float ACCEL_SCALE = 4096.0f;  // +/-8g
const float GYRO_DEADBAND = 0.08f;

float gxBias = 0, gyBias = 0, gzBias = 0;
float axBias = 0, ayBias = 0, azBias = 0;

// --- SOFTWARE LOW-PASS FILTER VARIABLES ---
float gxFilt = 0, gyFilt = 0, gzFilt = 0;
const float GYRO_ALPHA = 0.05f; // Aggressive 10-inch filtering

SPISettings spiSettings(8000000, MSBFIRST, SPI_MODE0);

// -------------------- ESC Outputs --------------------
#define ESC_FL_PIN 25
#define ESC_FR_PIN 26
#define ESC_RL_PIN 32
#define ESC_RR_PIN 33

Servo escFL, escFR, escRL, escRR;

const int ESC_MIN_US = 1000;
const int ESC_MAX_US = 2000;

// -------------------- Packet Structure --------------------
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
const float MAX_ANGLE_DEG = 30.0f; 

bool armed = false;
const int ARM_THROTTLE_THRESHOLD = 30;   
const int MIN_THROTTLE_FOR_PID   = 60;   

// Fixed Yaw Gains
const float FIXED_YAW_KP = 2.0f;
const float FIXED_YAW_KI = 0.05f;
const float FIXED_YAW_KD = 0.00f;

// -------------------- PID Class --------------------
struct PID {
  float kp, ki, kd;
  float integral;
  float outMin, outMax;
  float integralLimit;

  float update(float error, float derivativeTerm, float dt) {
    integral += error * dt;
    integral = constrain(integral, -integralLimit, integralLimit);
    float out = kp * error + ki * integral - kd * derivativeTerm;
    return constrain(out, outMin, outMax);
  }
  void reset() { integral = 0; }
};

PID pidRoll  = { 0.5f, 0.00f, 0.10f, 0, -400, 400, 150 };
PID pidPitch = { 0.5f, 0.00f, 0.10f, 0, -400, 400, 150 };
PID pidYaw   = { FIXED_YAW_KP, FIXED_YAW_KI, FIXED_YAW_KD, 0, -200, 200, 100 };

// -------------------- MPU9250 Functions --------------------
void writeRegister(uint8_t reg, uint8_t value) {
  digitalWrite(RFM_CS_PIN, HIGH);
  SPI.beginTransaction(spiSettings);
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t readRegister(uint8_t reg) {
  digitalWrite(RFM_CS_PIN, HIGH);
  SPI.beginTransaction(spiSettings);
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();
  return val;
}

void readBurst(uint8_t reg, uint8_t *buf, uint8_t len) {
  digitalWrite(RFM_CS_PIN, HIGH);
  SPI.beginTransaction(spiSettings);
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

void calibrateSensors() {
  const int samples = 1000;
  float sums_g[3] = {0, 0, 0}; float sums_a[3] = {0, 0, 0};
  Serial.println("Calibrating IMU... Keep drone still.");
  
  for (int i = 0; i < samples; i++) {
    float ax, ay, az, gx, gy, gz;
    readSensors(ax, ay, az, gx, gy, gz);
    sums_g[0] += gx; sums_g[1] += gy; sums_g[2] += gz;
    sums_a[0] += ax; sums_a[1] += ay; sums_a[2] += az;
    delayMicroseconds(1000);
  }

  gxBias = sums_g[0] / samples; gyBias = sums_g[1] / samples; gzBias = sums_g[2] / samples;
  float axMean = sums_a[0] / samples; float ayMean = sums_a[1] / samples; float azMean = sums_a[2] / samples;
  float mag = sqrtf(axMean * axMean + ayMean * ayMean + azMean * azMean);
  float correction = (mag > 0.0001f) ? (1.0f / mag) : 1.0f;

  axBias = axMean * correction - axMean; ayBias = ayMean * correction - ayMean; azBias = azMean * correction - azMean;
  Serial.println("IMU calibration complete.");
}

void writeAllEscs(int us) {
  us = constrain(us, ESC_MIN_US, ESC_MAX_US);
  escFL.writeMicroseconds(us); escFR.writeMicroseconds(us);
  escRL.writeMicroseconds(us); escRR.writeMicroseconds(us);
}

void armEscs() {
  Serial.println("Arming ESCs...");
  writeAllEscs(ESC_MIN_US);
  delay(2000);
}

void disarm() {
  armed = false;
  pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
  writeAllEscs(ESC_MIN_US);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(MPU_CS_PIN, OUTPUT); pinMode(RFM_CS_PIN, OUTPUT);
  digitalWrite(MPU_CS_PIN, HIGH); digitalWrite(RFM_CS_PIN, HIGH);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
  writeRegister(PWR_MGMT_1, 0x01); delayMicroseconds(100);

  if (!verifySensor()) {
    Serial.println("FATAL: MPU9250 not responding.");
    while (true) delay(1000);
  }
  
  writeRegister(SMPLRT_DIV, 0);
  writeRegister(CONFIG, 0x03);       // DLPF ~41Hz filtering
  writeRegister(GYRO_CONFIG, 0x08);  // +/-500 dps
  writeRegister(ACCEL_CONFIG, 0x10); // +/-8g
  writeRegister(ACCEL_CONFIG2, 0x03);// Accel DLPF ~41Hz

  filter.begin(SAMPLE_RATE); 
  calibrateSensors();

  pinMode(RFM_RESET_PIN, OUTPUT);
  digitalWrite(RFM_RESET_PIN, LOW); delay(10);
  digitalWrite(RFM_RESET_PIN, HIGH); delay(10);
  digitalWrite(RFM_RESET_PIN, LOW); delay(100);

  if (!radio.initialize(FREQUENCY, NODEID, NETWORKID)) {
    Serial.println("Radio Init Failed");
    while (1);
  }
  radio.setHighPower(); radio.setPowerLevel(31);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  escFL.setPeriodHertz(50); escFR.setPeriodHertz(50);
  escRL.setPeriodHertz(50); escRR.setPeriodHertz(50);
  
  escFL.attach(ESC_FL_PIN, ESC_MIN_US, ESC_MAX_US); escFR.attach(ESC_FR_PIN, ESC_MIN_US, ESC_MAX_US);
  escRL.attach(ESC_RL_PIN, ESC_MIN_US, ESC_MAX_US); escRR.attach(ESC_RR_PIN, ESC_MIN_US, ESC_MAX_US);
  
  armEscs();
  lastPacketMs = millis();
  Serial.println("DRONE READY (1 kHz + Kd/Ki Pots Swapped)");
}

void applyTuning() {
  pidRoll.kp  = pkt.kp;   pidPitch.kp  = pkt.kp;
  pidRoll.kd  = pkt.kd;   pidPitch.kd  = pkt.kd; // Pot 2 controls Kd
  pidRoll.ki  = pkt.ki;   pidPitch.ki  = pkt.ki; // Pot 3 controls Ki
}

void handleArming() {
  bool switchOn = (pkt.armSwitch == 1);
  bool throttleLow = (pkt.throttle < ARM_THROTTLE_THRESHOLD);

  if (!armed && switchOn && throttleLow) {
    armed = true; pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
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

  // --- 1 kHz STRICT LOOP TIMING ---
  static uint32_t lastSample = micros();
  uint32_t now = micros();
  if ((uint32_t)(now - lastSample) < SAMPLE_PERIOD_US) return;
  
  float dt = (now - lastSample) / 1000000.0f;
  lastSample = now;

  float ax, ay, az, gx, gy, gz;
  readSensors(ax, ay, az, gx, gy, gz);

  ax += axBias; ay += ayBias; az += azBias;
  gx -= gxBias; gy -= gyBias; gz -= gzBias;

  if (fabs(gx) < GYRO_DEADBAND) gx = 0.0f;
  if (fabs(gy) < GYRO_DEADBAND) gy = 0.0f;
  if (fabs(gz) < GYRO_DEADBAND) gz = 0.0f;

  // --- DIGITAL LOW PASS FILTER (Alpha = 0.05) ---
  gxFilt = (gxFilt * (1.0f - GYRO_ALPHA)) + (gx * GYRO_ALPHA);
  gyFilt = (gyFilt * (1.0f - GYRO_ALPHA)) + (gy * GYRO_ALPHA);
  gzFilt = (gzFilt * (1.0f - GYRO_ALPHA)) + (gz * GYRO_ALPHA);

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  float currentRoll  = filter.getRoll()  - 0.60f;
  float currentPitch = filter.getPitch() + 0.38f;

  if (!armed) {
    writeAllEscs(ESC_MIN_US);
    return;
  }

  float rollSetpoint    = (pkt.roll  / 500.0f) * MAX_ANGLE_DEG;
  float pitchSetpoint   = (pkt.pitch / 500.0f) * MAX_ANGLE_DEG;
  
  float escBaseUs = map(pkt.throttle, 0, 1000, ESC_MIN_US, ESC_MAX_US);
  float rollOut = 0, pitchOut = 0, yawOut = 0;

  if (pkt.throttle >= MIN_THROTTLE_FOR_PID) {
    rollOut  = pidRoll.update(rollSetpoint - currentRoll,  gxFilt, dt);
    pitchOut = pidPitch.update(pitchSetpoint - currentPitch, gyFilt, dt);
    yawOut   = pidYaw.update(0.0f - gzFilt, gzFilt, dt); 
  } else {
    pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
  }

  // --- MOTOR MIXING WITH STATIC TRIM BIAS ---
  // Adjust these trim values (+/- microseconds) if front/rear motors start unevenly
  const int FRONT_TRIM = 15; // Boosts Front Left & Right
  const int REAR_TRIM  = 15; // Drops Rear Left & Right

  int flUs = escBaseUs - pitchOut + rollOut + yawOut + FRONT_TRIM;
  int frUs = escBaseUs - pitchOut - rollOut - yawOut + FRONT_TRIM-2;
  int rlUs = escBaseUs + pitchOut + rollOut - yawOut - REAR_TRIM+2;
  int rrUs = escBaseUs + pitchOut - rollOut + yawOut - REAR_TRIM;

  escFL.writeMicroseconds(constrain(flUs, ESC_MIN_US, ESC_MAX_US));
  escFR.writeMicroseconds(constrain(frUs, ESC_MIN_US, ESC_MAX_US));
  escRL.writeMicroseconds(constrain(rlUs, ESC_MIN_US, ESC_MAX_US));
  escRR.writeMicroseconds(constrain(rrUs, ESC_MIN_US, ESC_MAX_US));
  // --- TELEMETRY PRINTING (10 Hz) ---
  static uint32_t lastPrint = 0;
  // --- IN-DEPTH IMU TELEMETRY PRINTING (10 Hz) ---
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    
    // Prints:
    // 1. Raw Accelerometer values (g) with bias applied
    // 2. Raw Gyroscope rates (dps) vs Filtered Gyro rates (dps)
    // 3. Calculated Madgwick Angles (Roll & Pitch)
    Serial.printf("ACC[X:%5.2f Y:%5.2f Z:%5.2f] | GYRO_R[X:%6.1f Y:%6.1f Z:%6.1f] | GYRO_F[X:%6.1f Y:%6.1f Z:%6.1f] | ANG[R:%5.1f P:%5.1f]\n",
                  ax, ay, az, 
                  gx, gy, gz, 
                  gxFilt, gyFilt, gzFilt, 
                  currentRoll, currentPitch);
  }
}