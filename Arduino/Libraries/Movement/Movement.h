#include "DualVNH5019MotorShield.h"
#include "PID.h"
#include "Sensors.h"

class Movement{
	
public:
	
	DualVNH5019MotorShield motorShield;
	PID pid;
	Sensors sensor;
	
	int distsub;
	boolean straightTransition;  // If the robot has to move straight after changing direction.
	boolean rotateTransition;    // If the robot has to rotate after changing direction.
	
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