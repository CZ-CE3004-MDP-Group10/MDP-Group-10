// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT ROTATE 90 DEGREES LEFT FILE.

// Include the movement header file.
#include "Movement.h"

// Rotate Left 90 Degrees.
void Movement::rotate90left()
{
	pid.setZero();
	//Serial.println("First rotate left transition.");
		
	// Theoretically 398 ticks rotates the robot by approximately 90 degrees.
	// The ticks to move for each motor when rotating have to be individually adjusted.
	pid.M1_ticks_to_move = 347; //OK
	pid.M2_ticks_to_move = 307; //OK

	// Set the boolean variables to keep track of movement transitions.
	straightTransition = true;

	// While there are still steps to move.
	while(distsub > 0)
	{
		// Left motor is negative and right motor is positive.
		pid.control(-1,1);
		motorShield.setSpeeds(pid.getLeftSpeed(),pid.getRightSpeed());
		stopIfFault();
		delay(10);
		stopIfRotated();
	}
	
	// To check if the robot is stuck in an infinite loop, check if left and right rotations are executed simultaneously.
	
	if(previousCommand == 'R')
	{
		// Increment the counter that will determine if the robot is stuck in a loop.
		rotateCount++;
		
		// If the robot has rotated left and right 2 times consecutively.
		if(rotateCount >= 9)
		{
			// Set the boolean variable to exit the potential rotating loop that the robot may be stuck in.
			sensor.exitStuckLoop = true;
			
			// Reset the counter for being stuck in a loop.
			rotateCount = 0;
		}
	}
	else
	{
		// If the robot does not follow the consecutive left right sequence, reset the stuck loop counter.
		rotateCount = 0;
	}
	// Set the previous command for checking in the next command.
	previousCommand = 'L';
}
