// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT STOP IF THERE IS A MOTOR FAULT FILE.

// Include the movement header file.
#include "Movement.h"

// Stops the motor if there is a fault, like a short circuit.
// Infinite loop stops the program from continuing.
// Disable this function if the robot constantly reports false alarm motor faults.
void Movement::stopIfFault()
{
	if (motorShield.getM1Fault())
	{
		Serial.println("Left motor fault.");
		//motorShield.setBrakes(400, 400);
		//while(1);
	}
	
	if (motorShield.getM2Fault())
	{
		Serial.println("Right motor fault.");
		//motorShield.setBrakes(400, 400);
		//while(1);
	}
}
