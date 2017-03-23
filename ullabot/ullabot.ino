#include "Adafruit_BMP085_U.h"
#include "Adafruit_BNO055.h"
#include "OrientationControl.h"
#include "global_vars.h"
#include <Servo.h>

// Function Prototypes
void driveRobot(int speed); // + = forward, - = backward, 0-255 speed
void driveRobot(int leftSpeed, int rightSpeed); // + = forward, - = backward, 0-255 speed
void rotateRobotOnSpot(int speed); // - = CCW, + = CW, 0-255 speed

// IMU Declarations
Orientation orientation = Orientation();
sensors_event_t cur_orientation;

double integrationCounter = 0;
double speedMultiplier = 1;
// Servo Declaration
Servo sensorServo;
double absolute_angle = 0;
double servo_angle = 90;

enum States
{
  Wall_approach_1, //0
  Wall_approach_2, //1
  Wall_up, // 2
  Wall_down, // 3
  Pole_finding, // 4
  Pole_approach, // 5
  Facing_left, //6
  Facing_right, // 7
  Facing_straight, // 8
  Pole_confirmation // 9
};
States state = Wall_down;
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
  sensorServo.write(servo_angle);

  // IR Sensor Setup
  pinMode(IR_SIGNAL, INPUT);
  Serial.println("Hiya");

  // Push Button Setup
  pinMode(PUSH_BUTTON_START, INPUT);
  pinMode(PUSH_BUTTON_EXTRA, INPUT);
  pinMode(PUSH_BUTTON_SOFT_RESET, INPUT);
  //IMU Setup
  orientation.setupOrientation();

  // LED Setup
  pinMode(LED_IMU_STATE, OUTPUT);
}
void loop()
{
  // Once every calibration value is 3, we're good to go
  orientation.displayCalStatus();
  //Serial.println("Hi");

  bool multiplier_latch = false;
  // Read Button

  setAbsSensorAngle(45);

  delay(100);
  readIRSensor();
  bool startButtonState = digitalRead(PUSH_BUTTON_START);

  if (startButtonState == HIGH)
  {
    delay(500);
    speedMultiplier = 1;
    orientation.setInitOrientation();
    servo_angle = 90;
    while (true)
    {
      //Angle Sensor
      Serial.print("State: ");
      Serial.print(state);
      Serial.println("");
      // Retrieve IMU Data
      orientation.printCurrentOrientation();
      switch (state) {
      case Wall_approach_1:
        orientation.setXOrientationOffset(-15);
        if (orientation.getXOrientationDelta() > -5)
        {
          state = Wall_approach_2;
        }
        break;
      case Wall_approach_2:
        orientation.setXOrientationOffset(10);
        integrationCounter = 0;
        if (orientation.getZOrientationDelta() > 70)
        {
          state = Wall_up;
        }
        break;
      case Wall_up:
        orientation.setXOrientationOffset(0);
        if (orientation.getZOrientationDelta() < -60)
        {
          state = Wall_down;
        }
        break;
      case Wall_down:
        if (orientation.getZOrientationDelta() > -30)
        {
          state = Pole_finding;
        }
        break;
      case Pole_finding:
        speedMultiplier = 0;
        orientation.setXOrientationOffset(0);
        if (orientation.getXOrientationDelta() > 30)
        {
          state = Facing_left;
        }
        else if (orientation.getXOrientationDelta() < -60)
        {
          servo_angle = 70;
          state = Facing_right;
        }
        else
        {
          servo_angle = -70;
          state = Facing_straight;
        }
        //If facing left, abort!!!! Turn on spot.
        //If facing right, probably do a shallow turn towards front
        //If facing front, go front with sensor facing right.
        //

        break;
      case Pole_approach:
        break;
      case Facing_left:
        speedMultiplier = 0.4;
        orientation.setXOrientationOffset(0);

        if (orientation.getXOrientationDelta() > 10)
        {
          rotateRobotOnSpot(-255 * speedMultiplier);
        }
        else
        {
          state = Pole_finding;
        }
        break;
      case Facing_right:
        speedMultiplier = 0.6;
        orientation.setXOrientationOffset(30);

        setAbsSensorAngle(0);
        break;
      case Facing_straight:
        speedMultiplier = 0.4;
        orientation.setXOrientationOffset(0);
        setAbsSensorAngle(80);
        if (orientation.getXOrientationDelta() > 30)
        {
          state = Facing_left;
        }
        if (checkForObstruction())
        {
          if (checkForObstruction())
          {
            state = Pole_confirmation;
            speedMultiplier = 0;
          }
        }
        break;
      case Pole_confirmation:
        // TODO: Ramp Ignoring code if its an issue
        // Stop car and recheck.
        setAbsSensorAngle(80);
        orientation.setXOrientationOffset(80);
        if (orientation.getXOrientationDelta() > 20)
        {
          speedMultiplier = 0.6;
          rotateRobotOnSpot(-255 * speedMultiplier);
        }
        else
        {
          speedMultiplier = 0;
          rotateRobotOnSpot(-255 * speedMultiplier);
        }

      }
      if (state != Facing_left && state != Pole_confirmation)
      {
        setMotorSpeed();
      }
      orientation.printOrientationDelta();
      bool extraButtonState = digitalRead(PUSH_BUTTON_EXTRA);
      if (extraButtonState == HIGH && multiplier_latch == false)
      {
        speedMultiplier = fmod((speedMultiplier + 0.2), 1.20);
        if (speedMultiplier == 0)
        {
          speedMultiplier = speedMultiplier + 0.6;
        }

        Serial.print("Speed: ");
        Serial.print(speedMultiplier);
        multiplier_latch = true;
        integrationCounter = 0;
      }
      if (extraButtonState == LOW)
      {
        multiplier_latch = false;
      }
      if (!digitalRead(PUSH_BUTTON_SOFT_RESET))
      {
        integrationCounter = 0;
        speedMultiplier = 0.0;
        setMotorSpeed();
        orientation.setInitOrientation();
        state = Wall_approach_1;
        break;
      }
      //delay(100);
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
  digitalWrite(LEFT_MOTOR_DIRECTION, (dir) ^ LEFT_MOTOR_FLIP);
  digitalWrite(RIGHT_MOTOR_DIRECTION, (!dir) ^ RIGHT_MOTOR_FLIP);
  analogWrite(LEFT_MOTOR_PWM, abs(speed));
  analogWrite(RIGHT_MOTOR_PWM, abs(speed));
}
int readIRSensor()
{
  double sensorVoltage = analogRead(IR_SIGNAL) / 1024.00 * 5.00;
  double distance = 18.87*sensorVoltage*sensorVoltage - 96.319*sensorVoltage + 143.68; //cm
  Serial.print("IR Sensor Voltage: ");
  Serial.print(sensorVoltage);
  Serial.print("IR Sensor Distance: ");
  Serial.print(distance);
  Serial.println();
  return distance;
}
void setMotorSpeed()
{
  // correction multiplier
  double angleDelta = orientation.getXOrientationDelta();
  // if angleDelta smaller than zero, we are too far right
  // if angleDelta largetr than zero, we are too far left
  if (angleDelta > 270)
    angleDelta = angleDelta - 360;
  else if (angleDelta < -270)
    angleDelta = angleDelta + 360;
  double positionMultiplier = 1 - 1 * abs(angleDelta) / 360;
  integrationCounter = integrationCounter + 0.1*angleDelta;
  if (integrationCounter > 270)
  {
    integrationCounter = 270;
  }
  else if (integrationCounter < -270)
  {
    integrationCounter = -270;
  }
  double integrationMultiplier = 1 - 0.1*abs(integrationCounter) / 360;


  if (angleDelta > 0)
  {
    driveRobot(255 * speedMultiplier, 255 * speedMultiplier * positionMultiplier * integrationMultiplier);
    //driveRobot(255 * speedMultiplier, 255 * speedMultiplier);
  }
  if (angleDelta < 0)
  {
    driveRobot(255 * speedMultiplier * positionMultiplier * integrationMultiplier, 255 * speedMultiplier);
    //driveRobot(255 * speedMultiplier, 255 * speedMultiplier);
  }
}
void setAbsSensorAngle(double angle)
{
  double cur_x_orientation = orientation.getCurrentOrientation().orientation.x - orientation.getInitOrientation().orientation.x + (servo_angle - 90);
  if (cur_x_orientation > 270)
    cur_x_orientation = cur_x_orientation - 360;
  else if (cur_x_orientation < -270)
    cur_x_orientation = cur_x_orientation + 360;
  double angle_delta = cur_x_orientation - angle;
  Serial.print("Sensor Orientation: ");
  Serial.print(angle_delta);
  Serial.println("");
  // if angleDelta smaller than zero, we are too far left
  // if angleDelta largetr than zero, we are too far right
  if (angle_delta < -2)
  {
    servo_angle = servo_angle + 3;
    if (servo_angle > 180)
    {
      servo_angle = 180;
    }
  }
  else if (angle_delta > 2)
  {
    servo_angle = servo_angle - 3;
    if (servo_angle < 0)
    {
      servo_angle = 0;
    }
  }
  sensorServo.write(servo_angle);
  absolute_angle = angle;
}
bool checkForObstruction()
{
  double distance = readIRSensor();
  if (distance < 80)
  {
    return true;
  }
  else
  {
    return false;
  }
}