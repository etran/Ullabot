#include "OrientationControl.h"
#include <EEPROM.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
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
    Serial.println("Orientation Sensor Test"); Serial.println("");

    /* Initialise the sensor */
    if (!IMU.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    IMU.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

//        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        IMU.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }

    delay(1000);

    /* Display some basic information on this sensor */
  //  displaySensorDetails();

    /* Optional: Display current status */
    //displaySensorStatus();

    IMU.setExtCrystalUse(true);

    sensors_event_t event;
    IMU.getEvent(&event);
    if (foundCalib){
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!IMU.isFullyCalibrated())
        {
            IMU.getEvent(&event);
			Serial.print("X: ");
            Serial.print(event.orientation.x, 4);
            Serial.print("\tY: ");
            Serial.print(event.orientation.y, 4);
            Serial.print("\tZ: ");
            Serial.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            Serial.println("");
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }
    else
    {
        Serial.println("Please Calibrate Sensor: ");
        while (!IMU.isFullyCalibrated())
        {
            IMU.getEvent(&event);

            Serial.print("X: ");
            Serial.print(event.orientation.x, 4);
            Serial.print("\tY: ");
            Serial.print(event.orientation.y, 4);
            Serial.print("\tZ: ");
            Serial.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            Serial.println("");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    IMU.getSensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    IMU.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
    delay(500);
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
  if ( accel == 3 )
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
	double delta = getInitOrientation().orientation.x - (getCurrentOrientation().orientation.x - x_orientation_offset);
    if ( delta > 270 )
      delta = delta - 360;
    else if ( delta < -270 )
      delta = delta + 360;
	return delta;
}
double Orientation::getYOrientationDelta()
{
	return getInitOrientation().orientation.y - (getCurrentOrientation().orientation.y - y_orientation_offset);
}
double Orientation::getZOrientationDelta()
{
	return getInitOrientation().orientation.z - (getCurrentOrientation().orientation.z - z_orientation_offset);
}
void Orientation::printOrientationDelta()
{
	sensors_event_t cur_event = getCurrentOrientation();
	Serial.print("XangleDelta: ");
	Serial.print(getInitOrientation().orientation.x - (cur_event.orientation.x - x_orientation_offset));
	Serial.print("    ");	  
	Serial.print("ZangleDelta: ");
	Serial.print(getInitOrientation().orientation.z - (cur_event.orientation.z - z_orientation_offset));
	Serial.print("    ");	  
	Serial.print("YangleDelta: ");
	Serial.print(getInitOrientation().orientation.y - (cur_event.orientation.y - y_orientation_offset));
	Serial.println("");	  
}