#include "OrientationControl.h"

// Constructor
Orientation::Orientation(void)
{
	double x_orientation_offset = 0;
	double y_orientation_offset = 0;
	double z_orientation_offset = 0;
}
void Orientation::setupOrientation()
{
	IMU = Adafruit_BNO055(55);
	IMU.begin();
	delay(1000);
	IMU.setExtCrystalUse(true);	
}
void Orientation::setInitOrientation()
{
	IMU.getEvent(&initOrientation);
}
sensors_event_t Orientation::getInitOrientation()
{
	return initOrientation;
}
sensors_event_t Orientation::getCurrentOrientation()
{
	IMU.getEvent(&imuEvent);
	return imuEvent;
}
void Orientation::printCurrentOrientation()
{
	IMU.getEvent(&imuEvent);
	Serial.print("X: ");
    Serial.print(imuEvent.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(imuEvent.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(imuEvent.orientation.z, 4);
    Serial.println("");
}
void Orientation::resetOrientationOffset()
{
	x_orientation_offset = 0;
	y_orientation_offset = 0;
	z_orientation_offset = 0;
}

void Orientation::displayCalStatus(void)
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
  if ( gyro == 3 && accel == 3 && mag == 3)
  {
    digitalWrite(LED_IMU_STATE, HIGH);
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

void Orientation::setXOrientationOffset(double offset)
{
	if(offset > 180)
	{
		offset = offset - 360;
	}
	if(offset < -180)
	{
		offset = offset + 360;
	}
	x_orientation_offset = offset;
}
void Orientation::setYOrientationOffset(double offset)
{
	if(offset > 180)
	{
		offset = offset - 360;
	}
	if(offset < -180)
	{
		offset = offset + 360;
	}
	y_orientation_offset = offset;
}
void Orientation::setZOrientationOffset(double offset)
{
	if(offset > 180)
	{
		offset = offset - 360;
	}
	if(offset < -180)
	{
		offset = offset + 360;
	}
	z_orientation_offset = offset;
}
void Orientation::addXOrientationOffset(double change)
{
	x_orientation_offset = x_orientation_offset + change;
	if(x_orientation_offset > 180)
	{
		x_orientation_offset = x_orientation_offset - 360;
	}
	if(x_orientation_offset < -180)
	{
		x_orientation_offset = x_orientation_offset + 360;
	}
}
void Orientation::addYOrientationOffset(double change)
{
	y_orientation_offset = y_orientation_offset + change;
	if(y_orientation_offset > 180)
	{
		y_orientation_offset = y_orientation_offset - 360;
	}
	if(y_orientation_offset < -180)
	{
		y_orientation_offset = y_orientation_offset + 360;
	}
}
void Orientation::addZOrientationOffset(double change)
{
	
	z_orientation_offset = z_orientation_offset + change;
	if(z_orientation_offset > 180)
	{
		z_orientation_offset = z_orientation_offset - 360;
	}
	if(z_orientation_offset < 180)
	{
		z_orientation_offset = z_orientation_offset + 360;
	}
}
double Orientation::getXOrientationDelta()
{
	return getInitOrientation().orientation.x - (getCurrentOrientation().orientation.x - x_orientation_offset);
}
double Orientation::getYOrientationDelta()
{
	return getInitOrientation().orientation.y - (getCurrentOrientation().orientation.y - y_orientation_offset);
}
double Orientation::getZOrientationDelta()
{
	return getInitOrientation().orientation.z - (getCurrentOrientation().orientation.z - z_orientation_offset);
}
