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
	boolean straightTransition;
	//boolean rotateTransition;
	boolean loopSwitchCase;
	
	// Movement functions.
	void init(void);
	void forwards();
	void rotate90left();
	void rotate90right();
	void rotate180();
	void stopIfReached();
	void stopIfRotated();
	void stopIfFault();
	void readSensor();
	void right_tick_increment();
	void left_tick_increment();
};