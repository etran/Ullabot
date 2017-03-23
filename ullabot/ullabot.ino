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
double speedMultiplier = 0.80;
// Servo Declaration
Servo sensorServo;
int absolute_angle=0;


enum States
{
    Wall_approach_1,
    Wall_approach_2,
    Wall_up,
    Wall_down,
    Pole_finding,
    Pole_approach,
	Facing_left
};
States state = Wall_approach_1;
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
  //Serial.println("Hi");
  
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
        switch(state){
        case Wall_approach_1:
          orientation.setXOrientationOffset(-30);
          if(orientation.getXOrientationDelta() == 0)
		  {
			  state = Wall_approach_2;
		  }
          break;
        case Wall_approach_2:
          orientation.setXOrientationOffset(0);
		  if(orientation.getZOrientationDelta() < -70)
		  {
			  state = Wall_up;
		  }
          break;
        case Wall_up:
          if(orientation.getZOrientationDelta() > 60)
		  {
			  state = Wall_down;
		  }
          break;
        case Wall_down:
		  if(orientation.getZOrientationDelta() < 0)
		  {
			  state = Pole_finding;
		  }
          break;
        case Pole_finding:
		  speedMultiplier=0;
		  orientation.setXOrientationOffset(0);
		  if(orientation.getXOrientationDelta() < 0)
		  {
			  state = Facing_left;
		  }
		  else 
		  {
			  speedMultiplier = 0.1;
		  }
			  
		  //If facing left, abort!!!! Turn on spot.
		  //If facing right, probably do a shallow turn towards front
		  //If facing front, go front with sensor facing right.
		  //
		  
          break;
        case Pole_approach:
          break;
		case Facing_left:
		  orientation.setXOrientationOffset(0);
		  if(orientation.getXOrientationDelta() < -10)
		  {			  
			rotateRobotOnSpot(255 * speedMultiplier);
		  }
		  else if(orientation.getXOrientationDelta() > 10)
		  {
			  rotateRobotOnSpot(-255 * speedMultiplier);
		  }
		  else
		  {
			  state = Pole_finding;
		  }
		  break;
      }
	  if(state != Facing_left) 
	  {
		setMotorSpeed();
	  }		  
      bool extraButtonState = digitalRead(PUSH_BUTTON_EXTRA);
      if (extraButtonState == HIGH && multiplier_latch == false)
      {
        orientation.setInitOrientation();
        speedMultiplier = fmod((speedMultiplier + 0.2), 1.20);
		if(speedMultiplier == 0)
		{
			speedMultiplier = speedMultiplier +0.6;
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
  digitalWrite(LEFT_MOTOR_DIRECTION, (dir) ^ LEFT_MOTOR_FLIP);
  digitalWrite(RIGHT_MOTOR_DIRECTION, (!dir) ^ RIGHT_MOTOR_FLIP);
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
    integrationCounter = integrationCounter + 0.5*angleDelta;
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
      driveRobot(255 * speedMultiplier, 255 * speedMultiplier * positionMultiplier * integrationMultiplier);
      //driveRobot(255 * speedMultiplier, 255 * speedMultiplier);
    }
    if (angleDelta < 0)
    {
      driveRobot(255 * speedMultiplier * positionMultiplier * integrationMultiplier, 255 * speedMultiplier);
      //driveRobot(255 * speedMultiplier, 255 * speedMultiplier);
    }
}

