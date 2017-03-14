#include "MPU9255.h"
#include <Servo.h>

// Servo Motor Constants
#define SERVO_SIGNAL 26

// Motor Controller Constants
#define LEFT_MOTOR_DIRECTION 22
#define RIGHT_MOTOR_DIRECTION 24
#define LEFT_MOTOR_PWM 7
#define RIGHT_MOTOR_PWM 6
#define LEFT_MOTOR_FLIP 1
#define RIGHT_MOTOR_FLIP 1

// IR Sensor Constants
#define IR_SIGNAL A5

// Function Prototypes
void driveRobot(int speed); // + = forward, - = backward, 0-255 speed
void driveRobot(int leftSpeed, int rightSpeed); // + = forward, - = backward, 0-255 speed
void rotateRobotOnSpot(int speed); // + = CCW, - = CW, 0-255 speed

// Barometric Pressure Sensor Declarations
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
sensors_event_t event;
float altitudeOffset = 0;

// Servo Declaration
Servo sensorServo;

void setup() 
{
  // Motor Controller Setup
  pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);
  pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  // Servo Setup
  sensorServo.attach(SERVO_SIGNAL);

  // IMU Setup 
  MPU9255Setup();

  // Barometric Pressure Sensor Setup
  bmp.begin(BMP085_MODE_STANDARD);
  bmp.getEvent(&event);
  altitudeOffset = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure); // Do this better later

  // IR Sensor Setup
  pinMode(IR_SIGNAL, INPUT);
}

void loop() 
{
  // Retrieve Pressure Data
  bmp.getEvent(&event);
  float heightOffGround = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, event.pressure) - altitudeOffset;

  // Get Accel Data
  double accelX = readAccelX();
  double accelY = readAccelY();
  double accelZ = readAccelZ();

  // Get Gyro Data
  double gyroX = readGyroX();
  double gyroY = readGyroY();
  double gyroZ = readGyroZ();

  // Get Mag Data
  double magX = readMagX();
  double mayY = readMagY();
  double magZ = readMagZ();



  //Code to zero out gravity
  accelX = accelX - 79;
  accelY = accelY - 14.5;
  accelZ = accelZ - 8.55;


  Serial.println(accelX);
  Serial.println(accelY);
  Serial.println(accelZ);
  double accelMag =  sqrt((accelX*accelX)+(accelY*accelY)+(accelZ*accelZ));
  Serial.println(accelMag);
  
  delay(1000);

  driveRobot(127);
}

void driveRobot(int speed)
{
  bool dir = speed > 0 ? LOW : HIGH;
  digitalWrite(LEFT_MOTOR_DIRECTION, dir ^ LEFT_MOTOR_FLIP);
  digitalWrite(RIGHT_MOTOR_DIRECTION, dir ^ RIGHT_MOTOR_FLIP);
  analogWrite(LEFT_MOTOR_PWM, abs(speed));
  analogWrite(RIGHT_MOTOR_PWM, abs(speed));
}

void driveRobot(int leftSpeed, int rightSpeed)
{
  bool leftDir = leftSpeed > 0 ? LOW : HIGH;
  bool rightDir = rightSpeed > 0 ? LOW : HIGH;
  digitalWrite(LEFT_MOTOR_DIRECTION, (leftDir) ^ LEFT_MOTOR_FLIP);
  digitalWrite(RIGHT_MOTOR_DIRECTION, (rightDir) ^ RIGHT_MOTOR_FLIP);
  analogWrite(LEFT_MOTOR_PWM, abs(leftSpeed));
  analogWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));
}

void rotateRobotOnSpot(int speed)
{
  bool dir = speed > 0 ? LOW : HIGH;
  digitalWrite(LEFT_MOTOR_DIRECTION, (!dir) ^ LEFT_MOTOR_FLIP);
  digitalWrite(RIGHT_MOTOR_DIRECTION, (dir) ^ RIGHT_MOTOR_FLIP);
  analogWrite(LEFT_MOTOR_PWM, abs(speed));
  analogWrite(RIGHT_MOTOR_PWM, abs(speed));
}

