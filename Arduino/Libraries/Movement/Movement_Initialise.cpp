// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT INITIALISATION FILE.

// Include the movement header file.
#include "Movement.h"

// Initialize motor shield, sensors and PID controller.
void Movement::init()
{
	motorShield.init();
	pid.init();
	sensor.init();
	
	// Determines if the robot is moving straight after a rotation.
	straightTransition = true;
	
	// Determines if the robot is rotating right during a right wall calibration only.
	calibrateRightRotate = false;
	
	// The following variables are only needed for fastest path, not exploration or image recognition.
	//loopSwitchCase = true;
	//lastCommand = false;
	
	// Correction parameters for robot calibration.
	rotateCount = 0;
	error = 0.0;
	error_margin = 0.0;
	perfDist = 0.0;
	limit = 14;
	previousCommand = ' ';
}
