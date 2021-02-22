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
	boolean straightTransition;
	//boolean rotateTransition;
	
	// Initialization of objects.
	void init(void);
	
	// Movement functions.
	void forwards();
	void rotate90left();
	void rotate90right();
	void rotate180();
	
	// Movement stopping functions.
	void stopIfReached();
	void stopIfRotated();
	void stopIfFault();
	
	// Read sensor values.
	void readSensor();
	
	// Interrupt functions to increment encoder ticks.
	void right_tick_increment();
	void left_tick_increment();
};