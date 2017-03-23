#ifndef Orientation_h
#define Orientation_h
#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "Arduino.h"
#include "global_vars.h"

// Orientation and Distance Stuff
// Make it static? (singleton)
class Orientation{
	public:
		double x_orientation_offset;
		double y_orientation_offset;
		double z_orientation_offset;
		
		Orientation();
		void setupOrientation();
		void setInitOrientation();
		sensors_event_t getInitOrientation();
		sensors_event_t getCurrentOrientation();
		void resetOrientationOffset();
		void printCurrentOrientation();
		void displayCalStatus(void);
		
		void setXOrientationOffset(double);
		void setYOrientationOffset(double);
		void setZOrientationOffset(double);
		
		void addXOrientationOffset(double);
		void addYOrientationOffset(double);
		void addZOrientationOffset(double);
		
		double getXOrientationDelta();
		double getYOrientationDelta();
		double getZOrientationDelta();
	private:
		Adafruit_BNO055 IMU;
		sensors_event_t imuEvent;
		sensors_event_t initOrientation;
		
};

#endif
