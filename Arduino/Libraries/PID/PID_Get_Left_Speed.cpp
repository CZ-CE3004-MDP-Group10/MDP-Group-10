// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 PID RETURN LEFT SPEED FILE.

// Include the PID controller header file.
#include "PID.h"

// Returns the commanded power for the left motor.
int PID::getLeftSpeed()
{
	return left_speed;
}
