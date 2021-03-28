// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT ROTATE 180 DEGREES BY THE LEFT FILE.

// Include the movement header file.
#include "Movement.h"

/*
// ROTATE 180 DEGREES ENDED UP NOT BEING USED FOR FASTEST PATH OR EXPLORATION.
// Rotate Left 180 Degrees.
void Movement::rotate180()
{
	pid.setZero();
	//Serial.println("First rotate 180 transition.");

	pid.M1_ticks_to_move = 733; //OK
	pid.M2_ticks_to_move = 555; //OK

	straightTransition = true;
	
	while(distsub > 0)
	{
		// Left motor is negative and right motor is positive.
		pid.control(-1,1);
		motorShield.setSpeeds(pid.getLeftSpeed(),pid.getRightSpeed());
		stopIfFault();
		delay(10);
		stopIfRotated();
	}
}
*/
