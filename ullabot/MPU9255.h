#pragma once

#include "Arduino.h"
#include <Wire.h>
#include "Adafruit_BMP085_U.h"

#define MPU9255_ADDRESS 0x68
#define MAGNETOMETER_ADDRESS 0x0C
#define BMP180_ADDRESS 0xEE

// MPU9255 Other Registers
#define INT_PIN_CFG 0x37
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B

// MPU9255 Accelerometer Registers
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

// MPU9255 Gyroscope Registers
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

// Magnetometer Registers
#define ST2 0x09
#define CNTL1 0x0A
#define HXL 0x03
#define HXH 0x04
#define HYL 0x05
#define HYH 0x06
#define HZL 0x07
#define HZH 0x08

void MPU9255Setup();
int readAccelX();
int readAccelY();
int readAccelZ();
int readGyroX();
int readGyroY();
int readGyroZ();
int readMagX();
int readMagY();
int readMagZ();
