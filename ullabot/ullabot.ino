#include "Adafruit_BMP085_U.h"
#include "Adafruit_BNO055.h"
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

// Push Button Constants
#define PUSH_BUTTON_START 8
#define PUSH_BUTTON_EXTRA 9

// Function Prototypes
void driveRobot(int speed); // + = forward, - = backward, 0-255 speed
void driveRobot(int leftSpeed, int rightSpeed); // + = forward, - = backward, 0-255 speed
void rotateRobotOnSpot(int speed); // + = CCW, - = CW, 0-255 speed

// IMU Declarations
Adafruit_BNO055 IMU = Adafruit_BNO055(55);
sensors_event_t imuEvent;

// Barometric Pressure Sensor Declarations
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
sensors_event_t barometerEvent;
float altitudeOffset = 0;

// Servo Declaration
Servo sensorServo;

void setup() 
{
  Serial.begin(115200);
  
  // Motor Controller Setup
  pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);
  pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  // Servo Setup
  sensorServo.attach(SERVO_SIGNAL);

  // IMU Setup 
  IMU.begin();
  delay(1000);
  IMU.setExtCrystalUse(true);

  // Barometric Pressure Sensor Setup
  bmp.begin(BMP085_MODE_STANDARD);
  bmp.getEvent(&barometerEvent);

  // IR Sensor Setup
  pinMode(IR_SIGNAL, INPUT);

  // Push Button Setup
  pinMode(PUSH_BUTTON_START, INPUT);
  pinMode(PUSH_BUTTON_EXTRA, INPUT);
}

void loop() 
{
  // Read Button
  bool startButtonState = digitalRead(PUSH_BUTTON_START);
  if (startButtonState == HIGH)
  {
    while (true)
    {
      // Retrieve IMU Data
      IMU.getEvent(&imuEvent);
      Serial.print("X: ");
      Serial.print(imuEvent.orientation.x, 4);
      Serial.print("\tY: ");
      Serial.print(imuEvent.orientation.y, 4);
      Serial.print("\tZ: ");
      Serial.print(imuEvent.orientation.z, 4);
      Serial.println("");
    
      // Once every calibration value is 3, we're good to go
      displayCalStatus();
      
      // Retrieve Pressure Data
      bmp.getEvent(&barometerEvent);
      float heightFromSea = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA, barometerEvent.pressure);
      Serial.print("Height: ");
      Serial.print(heightFromSea);
      Serial.println("");
    
      delay(500);
    
      driveRobot(255*0.1);
    }
  }
  bool buttonState = digitalRead(PUSH_BUTTON_EXTRA);
  if (buttonState == HIGH)
  {
    Serial.println("Extra boi");
  }
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

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  IMU.getCalibration(&system, &gyro, &accel, &mag);
 
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
 
  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}

