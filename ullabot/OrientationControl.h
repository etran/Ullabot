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
		
		void setXOrientationOffset(int);
		void setYOrientationOffset(int);
		void setZOrientationOffset(int);
		
		void addXOrientationOffset(int);
		void addYOrientationOffset(int);
		void addZOrientationOffset(int);
    
	private:
		Adafruit_BNO055 IMU;
		sensors_event_t imuEvent;
		sensors_event_t initOrientation;
		
};

#endif
