#include "Adafruit_BMP085_U.h"
#include "Adafruit_BNO055.h"
#include "OrientationControl.h"
#include "global_vars.h"
#include <Servo.h>

// Function Prototypes
void driveRobot(int speed); // + = forward, - = backward, 0-255 speed
void driveRobot(int leftSpeed, int rightSpeed); // + = forward, - = backward, 0-255 speed
void rotateRobotOnSpot(int speed); // + = CCW, - = CW, 0-255 speed

// IMU Declarations
Orientation orientation = Orientation();
sensors_event_t cur_orientation;

double integrationCounter = 0;
double speedMultiplier = 0.60;
// Servo Declaration
Servo sensorServo;
int angle = 90;

void setup()
{
  Serial.begin(115200);
  Serial.println("Fuck");
  delay(250);
  // Motor Controller Setup
  pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);
  pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);

  // Servo Setup
  sensorServo.attach(SERVO_SIGNAL);
  sensorServo.write(90);

  // IR Sensor Setup
  pinMode(IR_SIGNAL, INPUT);
  Serial.println("Hiya");
  // Push Button Setup
  pinMode(PUSH_BUTTON_START, INPUT);
  pinMode(PUSH_BUTTON_EXTRA, INPUT);

  //IMU Setup
  orientation.setupOrientation();
  
  // LED Setup
  pinMode(LED_IMU_STATE, OUTPUT);
}
void loop()
{
  // Once every calibration value is 3, we're good to go
  orientation.displayCalStatus();
  Serial.println("Hi");
  
  bool multiplier_latch = false;
  // Read Button


  delay(100);
  readIRSensor();
  bool startButtonState = digitalRead(PUSH_BUTTON_START);
  
  if (startButtonState == HIGH)
  {
    delay(500);
    
  orientation.setInitOrientation();
    
    while (true)
    {
      // Retrieve IMU Data
      orientation.printCurrentOrientation();
      setMotorSpeed();
      bool extraButtonState = digitalRead(PUSH_BUTTON_EXTRA);
      if (extraButtonState == HIGH && multiplier_latch == false)
      {
        //orientation.setInitOrientation();
        speedMultiplier = fmod((speedMultiplier + 0.2), 1.20)+0.6;
        multiplier_latch = true;
        integrationCounter = 0;
      }
      if (extraButtonState == LOW)
      {
        multiplier_latch = false;
      }
      delay(500);
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
int readIRSensor()
{
  double sensorVoltage = analogRead(IR_SIGNAL)/1024.00 * 5.00;
  double distance = 18.87*sensorVoltage*sensorVoltage - 96.319*sensorVoltage + 143.68; //cm
  Serial.print("IR Sensor Voltage: ");
  Serial.print(sensorVoltage);
  Serial.print("IR Sensor Distance: ");
  Serial.print(distance);
  Serial.println();
}
void setMotorSpeed()
{
    // correction multiplier
    double angleDelta = (orientation.getInitOrientation().orientation.x - orientation.getCurrentOrientation().orientation.x);
    // if angleDelta smaller than zero, we are too far right
    // if angleDelta largetr than zero, we are too far left
    if ( angleDelta > 270 )
      angleDelta = angleDelta - 360;
    else if ( angleDelta < -270 )
      angleDelta = angleDelta + 360;
    double positionMultiplier =  1 - 1 * abs(angleDelta) / 360;
    integrationCounter = integrationCounter + 0.25*angleDelta;
    if(integrationCounter > 270)
    {
      integrationCounter = 270;
    }
    else if(integrationCounter < -270)
    {
      integrationCounter = -270;
    }
    double integrationMultiplier = 1 - 0.1*abs(integrationCounter) / 360;
    Serial.print("angleDelta: ");
    Serial.print(angleDelta);
    Serial.println("");

    if (angleDelta > 0)
    {
      driveRobot(255 * speedMultiplier, 255 * speedMultiplier * positionMultiplier);
      //driveRobot(255 * speedMultiplier, 255 * speedMultiplier);
    }
    if (angleDelta < 0)
    {
      driveRobot(255 * speedMultiplier * positionMultiplier, 255 * speedMultiplier);
      //driveRobot(255 * speedMultiplier, 255 * speedMultiplier);
    }
}

