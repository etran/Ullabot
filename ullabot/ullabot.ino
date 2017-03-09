#include "MPU9255.h"

// Motor controller constants
#define LEFT_MOTOR_DIRECTION 22
#define RIGHT_MOTOR_DIRECTION 24
#define LEFT_MOTOR_PWM 7
#define RIGHT_MOTOR_PWM 6

// Barometric Pressure Sensor Declarations
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
sensors_event_t event;
float altitudeOffset = 0;

void setup() {
  // Motor Controller Setup
  pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);
  pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  // IMU Setup 
  MPU9255Setup();

  // Barometric Pressure Sensor Setup
  bmp.begin(BMP085_MODE_STANDARD);
  bmp.getEvent(&event);
  altitudeOffset = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure); // Do this better later

  // IR Sensor Setup
}

void loop() {
  // Retrieve Pressure Data
  bmp.getEvent(&event);
  float heightOffGround = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure) - altitudeOffset;

  // Get Accel Data
  int accelX = readAccelX();
  int accelY = readAccelY();
  int accelZ = readAccelZ();

  // Get Gyro Data
  int gyroX = readGyroX();
  int gyroY = readGyroY();
  int gyroZ = readGyroZ();

  // Get Mag Data
  int magX = readMagX();
  int mayY = readMagY();
  int magZ = readMagZ();
}
