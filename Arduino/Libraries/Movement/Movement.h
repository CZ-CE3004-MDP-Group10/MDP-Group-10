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
	int i;
	int limit;
	int rotateCount;
	float error;
	float error_margin;
	float perfDist;
	char previousCommand;
	bool straightTransition;
	bool calibrateRightRotate;
	//bool exitStuckLoop;
	//bool loopSwitchCase;
	//bool lastCommand;
	
	// Movement functions.
	void init(void);
	void forwards();
	void rotate90left();
	void rotate90right();
	void rotate180();
	void stopIfReached();
	void stopIfRotated();
	
	// Correction functions.
	void frontDistanceCheck();
	void rightDistanceCheck();
	void frontTiltCheck();
	void rightTiltCheck();
	
	// Other required functions.
	void stopIfFault();
	void readSensor();
	void printSensor();
	void right_tick_increment();
	void left_tick_increment();
};