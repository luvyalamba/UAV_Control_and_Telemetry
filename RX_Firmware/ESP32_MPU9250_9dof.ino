#include <SPI.h>
#include <MadgwickAHRS.h>

Madgwick filter;

// -------------------- Pins --------------------
#define MPU_CS_PIN 5
#define RFM_CS_PIN 21 

// -------------------- MPU9250 / AK8963 Registers --------------------
#define SMPLRT_DIV     0x19
#define CONFIG         0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_CONFIG2  0x1D
#define ACCEL_XOUT_H   0x3B
#define INT_PIN_CFG    0x37
#define USER_CTRL      0x6A
#define PWR_MGMT_1     0x6B
#define WHO_AM_I       0x75

// Magnetometer (AK8963) Registers over I2C Bypass
#define AK8963_I2C_ADDR  0x0C
#define AK8963_WHO_AM_I  0x00
#define AK8963_ST1       0x02
#define AK8963_XOUT_L    0x03
#define AK8963_CNTL1     0x0A
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV0_DO      0x63
#define I2C_SLV0_EN      0x80
#define EXT_SENS_DATA_00 0x49

// -------------------- Settings --------------------
const float SAMPLE_RATE = 1000.0f;
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE;

const float GYRO_SCALE = 65.5f;    // +/- 500 dps
const float ACCEL_SCALE = 4096.0f; // +/- 8g
const float MAG_SCALE = 0.15f;     // uT per LSB (16-bit mode)

const float GYRO_DEADBAND = 0.08f; 

float gxBias = 0, gyBias = 0, gzBias = 0;
float axBias = 0, ayBias = 0, azBias = 0;
float mxBias = 0, myBias = 0, mzBias = 0;

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); 

// -------------------- Safe SPI Handlers --------------------
void writeRegister(uint8_t reg, uint8_t value)
{
  digitalWrite(RFM_CS_PIN, HIGH); 
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

// Write to magnetometer via MPU internal I2C master passthrough
void writeMagRegister(uint8_t reg, uint8_t data)
{
  writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR); // Write to AK8963
  writeRegister(I2C_SLV0_REG, reg);              // Target register
  writeRegister(I2C_SLV0_DO, data);              // Data to write
  writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | 1); // Enable transfer 1 byte
  delay(10);
}

void reinitMPU()
{
  writeRegister(USER_CTRL, 0x10);  // Force SPI mode
  delayMicroseconds(100);
  writeRegister(PWR_MGMT_1, 0x01); // Auto select clock source
  delayMicroseconds(100);
  
  // Enable I2C Bypass so we can talk directly or set up slave reads for the magnetometer
  writeRegister(INT_PIN_CFG, 0x02); 
  delay(50);

  // Configure Magnetometer: Continuous measurement mode 2 (100Hz, 16-bit)
  writeMagRegister(AK8963_CNTL1, 0x16);
  
  // Setup MPU Slave 0 to automatically read 7 bytes from Magnetometer every sample cycle
  writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80); // Set as Read
  writeRegister(I2C_SLV0_REG, AK8963_ST1);               // Start reading from status register 1
  writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | 7);         // Read 7 bytes (ST1, Xout, Yout, Zout, ST2)
}

bool verifySensor()
{
  for (int attempt = 0; attempt < 10; attempt++)
  {
    uint8_t who = readRegister(WHO_AM_I);
    if (who == 0x71 || who == 0x73 || who == 0x70 || who == 0x68)
    {
      Serial.printf("MPU detected, WHO_AM_I = 0x%02X\n", who);
      return true;
    }
    delay(50);
  }
  return false;
}

void readSensors(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz)
{
  uint8_t raw[21];
  // Read 21 bytes starting from ACCEL_XOUT_H down through EXT_SENS_DATA (Mag bytes)
  readBurst(ACCEL_XOUT_H, raw, 21);

  int16_t rax = (raw[0] << 8) | raw[1];
  int16_t ray = (raw[2] << 8) | raw[3];
  int16_t raz = (raw[4] << 8) | raw[5];
  
  int16_t rgx = (raw[8] << 8) | raw[9];
  int16_t rgy = (raw[10] << 8) | raw[11];
  int16_t rgz = (raw[12] << 8) | raw[13];

  // Magnetometer data starts at index 14 (EXT_SENS_DATA_00)
  // AK8963 data output order is: ST1 [14], X_L [15], X_H [16], Y_L [17], Y_H [18], Z_L [19], Z_H [20]
  int16_t rmx = (raw[16] << 8) | raw[15]; // Little endian format for AK8963
  int16_t rmy = (raw[18] << 8) | raw[17];
  int16_t rmz = (raw[20] << 8) | raw[19];

  ax = rax / ACCEL_SCALE;
  ay = ray / ACCEL_SCALE;
  az = raz / ACCEL_SCALE;
  gx = rgx / GYRO_SCALE;
  gy = rgy / GYRO_SCALE;
  gz = rgz / GYRO_SCALE;
  
  mx = rmx * MAG_SCALE;
  my = rmy * MAG_SCALE;
  mz = rmz * MAG_SCALE;
}

void calibrateSensors()
{
  const int samples = 500;
  float sums_g[3] = {0, 0, 0};
  float sums_a[3] = {0, 0, 0};
  float sums_m[3] = {0, 0, 0};

  Serial.println("Calibrating 9-DOF... Keep PCB still.");

  for (int i = 0; i < samples; i++)
  {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    readSensors(ax, ay, az, gx, gy, gz, mx, my, mz);

    sums_g[0] += gx; sums_g[1] += gy; sums_g[2] += gz;
    sums_a[0] += ax; sums_a[1] += ay; sums_a[2] += az;
    sums_m[0] += mx; sums_m[1] += my; sums_m[2] += mz;

    delayMicroseconds(2000);
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

  mxBias = sums_m[0] / samples;
  myBias = sums_m[1] / samples;
  mzBias = sums_m[2] / samples;

  Serial.println("9-DOF Calibration complete!");
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
    Serial.println("FATAL: MPU9250 not responding.");
    while (true) { delay(1000); }
  }

  writeRegister(SMPLRT_DIV, 0);       
  writeRegister(CONFIG, 0x03);        // Gyro DLPF ~41Hz
  writeRegister(GYRO_CONFIG, 0x08);   // +/-500 dps
  writeRegister(ACCEL_CONFIG, 0x10);  // +/-8 g
  writeRegister(ACCEL_CONFIG2, 0x03); // Accel DLPF ~41Hz

  filter.begin(SAMPLE_RATE);
  calibrateSensors();
}

void loop()
{
  static uint32_t lastSample = micros();
  uint32_t now = micros();

  if ((uint32_t)(now - lastSample) < SAMPLE_PERIOD_US)
  {
    yield();
    return;
  }
  
  lastSample = now; 

  float ax, ay, az, gx, gy, gz, mx, my, mz;
  readSensors(ax, ay, az, gx, gy, gz, mx, my, mz);

  // Apply offsets & bias corrections
  ax += axBias; ay += ayBias; az += azBias;
  gx -= gxBias; gy -= gyBias; gz -= gzBias;
  mx -= mxBias; my -= myBias; mz -= mzBias;

  if (fabs(gx) < GYRO_DEADBAND) gx = 0.0f;
  if (fabs(gy) < GYRO_DEADBAND) gy = 0.0f;
  if (fabs(gz) < GYRO_DEADBAND) gz = 0.0f;

  // 9-DOF Update using Gyro, Accel, and Magnetometer inputs
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 10) 
  {
    lastPrint = millis();
    Serial.print("Roll: ");   Serial.print(filter.getRoll()-0.60f, 2);
    Serial.print("\tPitch: "); Serial.print(filter.getPitch()+0.38f, 2);
    Serial.print("\tYaw: ");   Serial.println(filter.getYaw(), 2); // Now drift-corrected via magnetometer!
  }
}