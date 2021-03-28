// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT ROTATE 90 DEGREES RIGHT FILE.

// Include the movement header file.
#include "Movement.h"

// Rotate Right 90 Degrees.
void Movement::rotate90right()
{
	//Serial.println("Called rotate right");
	pid.setZero();
	//Serial.println("First rotate right transition.");
		
	pid.M1_ticks_to_move = 328; //OK
	pid.M2_ticks_to_move = 330; //OK
	
	// When rotating right during right wall calibration, right turn is observed to fall short.
	// Extra ticks specified here and triggered by boolean variable are to fix the issue.
	if(calibrateRightRotate)
	{
		pid.M1_ticks_to_move = 340; //OK
		pid.M2_ticks_to_move = 343; //OK
	}
	straightTransition = true;

	while(distsub > 0)
	{
		// Left motor is positive and right motor is negative.
		pid.control(1,-1);
		motorShield.setSpeeds(pid.getLeftSpeed(),pid.getRightSpeed());
		stopIfFault();
		delay(10);
		stopIfRotated();
	}
	
	// To check if the robot is stuck in an infinite loop, check if left and right rotations are executed simultaneously.
	
	if(previousCommand == 'L')
	{
		// Increment the counter that will determine if the robot is stuck in a loop.
		rotateCount++;
		
		// If the robot has rotated left and right 2 times consecutively.
		if(rotateCount >= 9)
		{
			sensor.exitStuckLoop = true;
			rotateCount = 0;
		}
	}
	else
	{
		// If the robot does not follow the consecutive left right sequence, reset the stuck loop counter.
		rotateCount = 0;
	}
	// Set the previous command for checking in the next command.
	previousCommand = 'R';
}
