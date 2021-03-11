// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT HEADER FILE.

// Include the motor shield library, sensors library and PID controller library.
#include "DualVNH5019MotorShield.h"
#include "PID.h"
#include "Sensors.h"

class Movement
{
	public:
	
	// Create objects.
	DualVNH5019MotorShield motorShield;
	PID pid;
	Sensors sensor;
	
	// Movement variables.
	int distsub;
	int distsubConstant;
	int tiltCount;
	boolean straightTransition;
	boolean loopSwitchCase;
	boolean lastCommand;
	
	// Movement functions.
	void init(void);
	void forwards();
	void rotate90left();
	void rotate90right();
	void rotate180();
	void stopIfReached();
	void stopIfRotated();
	void calibrate();
	void frontCalibrate();
	void frontWallCheckTilt();
	void rightCalibrate();
	void rightWallDistCheck();
	void stopIfFault();
	void readSensor();
	void printSensor();
	void right_tick_increment();
	void left_tick_increment();
};