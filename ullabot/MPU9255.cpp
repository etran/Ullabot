#include "MPU9255.h"

// Private functions
void writeToRegister(int reg, int data)
{
  Wire.beginTransmission(MPU9255_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

byte readFromRegister(int reg)
{
  Wire.beginTransmission(MPU9255_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9255_ADDRESS, 1, false);
  byte val = Wire.read();
  Wire.endTransmission(true);
  return val;
}

void writeToRegisterMag(int reg, int data)
{
  Wire.beginTransmission(MAGNETOMETER_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

byte readFromRegisterMag(int reg)
{
  Wire.beginTransmission(MAGNETOMETER_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MAGNETOMETER_ADDRESS, 1, false);
  byte val = Wire.read();
  Wire.endTransmission(true);
  return val;
}

// Public functions
void MPU9255Setup()
{
  // I2C setup
  Serial.begin(115200);
  Wire.begin();

  // Power Management 1 Register
  writeToRegister(PWR_MGMT_1, 0);

  // User Control Register
  writeToRegister(USER_CTRL, 0);

  // Gyroscope Configuration Register
  writeToRegister(GYRO_CONFIG, 0x18); // +-2000 dps

  // Accelerometer Configuration Register
  writeToRegister(ACCEL_CONFIG, 0x18); // +-16g

  // Enable bypass
  writeToRegister(INT_PIN_CFG, 0x02);

  // Enable continuous measurement (magnetometer)
  writeToRegisterMag(CNTL1, 0x12);
}

int readAccelX()
{
  byte h = readFromRegister(ACCEL_XOUT_H);
  byte l = readFromRegister(ACCEL_XOUT_L);
  return (h << 8) | l;
}

int readAccelY()
{
  byte h = readFromRegister(ACCEL_YOUT_H);
  byte l = readFromRegister(ACCEL_YOUT_L);
  return (h << 8) | l;
}

int readAccelZ()
{
  byte h = readFromRegister(ACCEL_ZOUT_H);
  byte l = readFromRegister(ACCEL_ZOUT_L);
  return (h << 8) | l;
}

int readGyroX()
{
  byte h = readFromRegister(GYRO_XOUT_H);
  byte l = readFromRegister(GYRO_XOUT_L);
  return (h << 8) | l;
}

int readGyroY()
{
  byte h = readFromRegister(GYRO_YOUT_H);
  byte l = readFromRegister(GYRO_YOUT_L);
  return (h << 8) | l;
}

int readGyroZ()
{
  byte h = readFromRegister(GYRO_ZOUT_H);
  byte l = readFromRegister(GYRO_ZOUT_L);
  return (h << 8) | l;
}

int readMagX()
{
  byte h = readFromRegisterMag(HXH);
  byte l = readFromRegisterMag(HXL);
  readFromRegisterMag(ST2);
  return (h << 8) | l;
}

int readMagY()
{
  byte h = readFromRegisterMag(HYH);
  byte l = readFromRegisterMag(HYL);
  readFromRegisterMag(ST2);
  return (h << 8) | l;
}

int readMagZ()
{
  byte h = readFromRegisterMag(HZH);
  byte l = readFromRegisterMag(HZL);
  readFromRegisterMag(ST2);
  return (h << 8) | l;
}

