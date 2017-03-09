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
  // Motor controller setup
  pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);
  pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  // IMU setup 
  MPU9255Setup();

  // Barometric Pressure Sensor setup
  bmp.begin(BMP085_MODE_STANDARD);
  bmp.getEvent(&event);
  altitudeOffset = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure); // Do this better later
}

void loop() {
  // Retrieve Pressure Data
  bmp.getEvent(&event);
  float height = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure) - altitudeOffset;
  Serial.println(height);
  delay(150);
}
