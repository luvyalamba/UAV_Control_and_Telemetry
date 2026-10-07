#include <SPI.h>
#include <MadgwickAHRS.h>

Madgwick filter;

// -------------------- Pins --------------------
#define MPU_CS_PIN 5
#define RFM_CS_PIN 21 

// -------------------- MPU9250 Registers --------------------
#define SMPLRT_DIV     0x19
#define CONFIG         0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_CONFIG2  0x1D
#define ACCEL_XOUT_H   0x3B
#define USER_CTRL      0x6A
#define PWR_MGMT_1     0x6B
#define PWR_MGMT_2     0x6C
#define WHO_AM_I       0x75

// -------------------- Settings --------------------
const float SAMPLE_RATE = 200.0f;
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE;

const float GYRO_SCALE = 65.5f;
const float ACCEL_SCALE = 4096.0f;
const float GYRO_DEADBAND = 0.05f; 

float gxBias = 0, gyBias = 0, gzBias = 0;
float axBias = 0, ayBias = 0, azBias = 0;

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); 

// -------------------- SPI Handlers --------------------
void writeRegister(uint8_t reg, uint8_t value)
{
  digitalWrite(RFM_CS_PIN, HIGH); // Ensure RFM is released
  SPI.beginTransaction(spiSettings);
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t readRegister(uint8_t reg)
{
  digitalWrite(RFM_CS_PIN, HIGH);
  SPI.beginTransaction(spiSettings);
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();
  return val;
}

void readBurst(uint8_t reg, uint8_t *buf, uint8_t len)
{
  digitalWrite(RFM_CS_PIN, HIGH);
  SPI.beginTransaction(spiSettings);
  digitalWrite(MPU_CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++)
  {
    buf[i] = SPI.transfer(0x00);
  }
  digitalWrite(MPU_CS_PIN, HIGH);
  SPI.endTransaction();
}

void readRaw(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
  uint8_t raw[14];
  readBurst(ACCEL_XOUT_H, raw, 14);

  int16_t rax = (raw[0] << 8) | raw[1];
  int16_t ray = (raw[2] << 8) | raw[3];
  int16_t raz = (raw[4] << 8) | raw[5];
  
  int16_t rgx = (raw[8] << 8) | raw[9];
  int16_t rgy = (raw[10] << 8) | raw[11];
  int16_t rgz = (raw[12] << 8) | raw[13];

  ax = rax / ACCEL_SCALE;
  ay = ray / ACCEL_SCALE;
  az = raz / ACCEL_SCALE;
  gx = rgx / GYRO_SCALE;
  gy = rgy / GYRO_SCALE;
  gz = rgz / GYRO_SCALE;
}

bool verifySensor()
{
  for (int attempt = 0; attempt < 10; attempt++)
  {
    uint8_t who = readRegister(WHO_AM_I);
    if (who == 0x71 || who == 0x73 || who == 0x70 || who == 0x68)
    {
      Serial.printf("MPU detected successfully, WHO_AM_I = 0x%02X\n", who);
      return true;
    }
    delay(50);
  }
  return false;
}

void reinitMPU()
{
  writeRegister(USER_CTRL, 0x10); // Force SPI mode
  delayMicroseconds(100);
  writeRegister(PWR_MGMT_1, 0x01);
  delayMicroseconds(100);
}

void calibrateSensors()
{
  const int discard = 100;
  const int samples = 1000;

  float sums_g[3] = {0, 0, 0};
  float sums_a[3] = {0, 0, 0};

  Serial.println("Calibrating... Keep PCB still.");

  for (int i = 0; i < discard; i++)
  {
    float ax, ay, az, gx, gy, gz;
    readRaw(ax, ay, az, gx, gy, gz);
    delayMicroseconds(1000);
  }

  for (int i = 0; i < samples; i++)
  {
    float ax, ay, az, gx, gy, gz;
    readRaw(ax, ay, az, gx, gy, gz);

    sums_g[0] += gx; sums_g[1] += gy; sums_g[2] += gz;
    sums_a[0] += ax; sums_a[1] += ay; sums_a[2] += az;

    delayMicroseconds(1000);
  }

  gxBias = sums_g[0] / samples;
  gyBias = sums_g[1] / samples;
  gzBias = sums_g[2] / samples;

  float axMean = sums_a[0] / samples;
  float ayMean = sums_a[1] / samples;
  float azMean = sums_a[2] / samples;

  float mag = sqrtf(axMean * axMean + ayMean * ayMean + azMean * azMean);
  float correction = (mag > 0.0001f) ? (1.0f / mag) : 1.0f;
  axBias = axMean * correction - axMean;
  ayBias = ayMean * correction - ayMean;
  azBias = azMean * correction - azMean;

  Serial.println("Calibration complete.");
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  pinMode(MPU_CS_PIN, OUTPUT);
  pinMode(RFM_CS_PIN, OUTPUT);
  digitalWrite(MPU_CS_PIN, HIGH);
  digitalWrite(RFM_CS_PIN, HIGH);

  SPI.begin(18, 19, 23, -1);
  delay(50);

  reinitMPU();

  if (!verifySensor())
  {
    Serial.println("FATAL: MPU9250 not responding. Halting.");
    while (true) { delay(1000); }
  }

  writeRegister(SMPLRT_DIV, 0);       
  writeRegister(CONFIG, 0x03);        
  writeRegister(GYRO_CONFIG, 0x08);   
  writeRegister(ACCEL_CONFIG, 0x10);  
  writeRegister(ACCEL_CONFIG2, 0x03); 

  filter.begin(SAMPLE_RATE);
  calibrateSensors();
}

void loop()
{
  static uint32_t lastSample = micros();
  uint32_t now = micros();

  // Watchdog-safe timer catch-up logic
  if ((uint32_t)(now - lastSample) < SAMPLE_PERIOD_US)
  {
    yield(); // Keeps ESP32 background tasks and WDT alive
    return;
  }
  
  // Non-cumulative update prevents loop starvation freezes
  lastSample = now; 

  float ax, ay, az, gx, gy, gz;
  readRaw(ax, ay, az, gx, gy, gz);

  // Recovery check: If all readings are zero (indicates MPU internal reset), re-initialize SPI mode
  static uint8_t zeroCount = 0;
  if (ax == 0.0f && ay == 0.0f && az == 0.0f)
  {
    zeroCount++;
    if (zeroCount > 5)
    {
      reinitMPU();
      zeroCount = 0;
    }
  }
  else
  {
    zeroCount = 0;
  }

  ax += axBias; ay += ayBias; az += azBias;
  gx -= gxBias; gy -= gyBias; gz -= gzBias;

  if (fabs(gx) < GYRO_DEADBAND) gx = 0.0f;
  if (fabs(gy) < GYRO_DEADBAND) gy = 0.0f;
  if (fabs(gz) < GYRO_DEADBAND) gz = 0.0f;

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 50) 
  {
    lastPrint = millis();
    Serial.print("Roll: ");   Serial.print(filter.getRoll(), 2);
    Serial.print("\tPitch: "); Serial.print(filter.getPitch(), 2);
    Serial.print("\tYaw: ");   Serial.println(filter.getYaw(), 2);
  }
}