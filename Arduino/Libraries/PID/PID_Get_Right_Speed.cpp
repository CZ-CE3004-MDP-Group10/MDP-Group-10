// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 PID RETURN RIGHT SPEED FILE.

// Include the PID controller header file.
#include "PID.h"

// Returns the commanded power for the right motor.
int PID::getRightSpeed()
{
	return right_speed;
}
