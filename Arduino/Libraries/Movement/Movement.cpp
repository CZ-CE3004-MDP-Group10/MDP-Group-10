// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT CODE FILE.

// Include the movement header file.
#include "Movement.h"

// Initialize motor shield, sensors and PID controller.
void Movement::init()
{
	motorShield.init();
	pid.init();
	sensor.init();
	
	// Boolean variables determine movement transition state.
	straightTransition = true;
	
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

// Move Forwards.
void Movement::forwards()
{
// ########################################################
	// FOR FASTEST PATH ONLY:
	// The normal distsub value is used for decrementing the movement steps.
	// The constant value distsub is used for the switch case operation to determine the number
	// Of ticks that are required for steps 1 to 5.
	//distsubConstant = distsub;
// ########################################################
	
	// Reset the PID controller variables after each movement.
	pid.setZero();

	// Before moving forwards after a rotation or stationary stop, reset the differences in ticks.
	pid.M1_ticks_diff = 0;
	pid.M2_ticks_diff = 0;

	// If the robot has to move straight after changing direction.
	if(straightTransition and distsub == 1)
	{
		// Theoretically 298 ticks moves the robot forward by approximately 10cm.
		//Serial.println("First Straight Transition.");

		pid.M1_ticks_to_move = 235; //OK
		pid.M2_ticks_to_move = 235; //OK

		// Set boolean values to account for observed transition accuracy errors.
		straightTransition = false;
		
// ########################################################
		// FOR FASTEST PATH, UNCOMMENT THIS PORTION.
		// FOR EXPLORATION, COMMENT THIS PORTION.
		
		// If it is moving straight after a rotation transition, the switch case within the while
		// Loop body for forward movement below should not run at all.
		// The tick value set above will then move the robot one step forward.
		//loopSwitchCase = false;
// ########################################################
	}
	else
	{
		// If it is moving straight but not after a transition, iterate through the switch case
		// Statement with each decrement of distsub.
		//loopSwitchCase = true;
		
		pid.M1_ticks_to_move = 225 - pid.M1_ticks_diff; //OK
		pid.M2_ticks_to_move = 225 - pid.M2_ticks_diff; //OK
	}

	// Keep running while it is executing a command and has not reached the last step.
	while(distsub > 0)
	{
		// If it is not the first transition after rotation and distsub is not equal to 1.
		
// ########################################################
		// FOR FASTEST PATH, UNCOMMENT THIS PORTION.
		// FOR EXPLORATION, COMMENT THIS PORTION.
		/*
		if(loopSwitchCase)
		{
			// To account for discrepancies in the distance moving forward, we need to include a switch case
			// Statement here with 5 cases for 5 possible step distances when moving forwards up to 5 steps.
			switch(distsubConstant)
			{
				// When moving in a straight line, need to include correction for each number of steps.
				case 1: pid.M1_ticks_to_move = 220 - pid.M1_ticks_diff; //OK
						pid.M2_ticks_to_move = 220 - pid.M2_ticks_diff; //OK
						//Serial.println("Taking 1 step forward.");
						break;
					
				case 2: pid.M1_ticks_to_move = 256 - pid.M1_ticks_diff; //OK
						pid.M2_ticks_to_move = 266 - pid.M2_ticks_diff; //OK
						//Serial.println("Taking 2 steps forward.");
						break;
			
				case 3: pid.M1_ticks_to_move = 275 - pid.M1_ticks_diff; //OK
						pid.M2_ticks_to_move = 275 - pid.M2_ticks_diff; //OK
						//Serial.println("Taking 3 steps forward.");
						break;
			
				case 4: pid.M1_ticks_to_move = 279 - pid.M1_ticks_diff; //OK
						pid.M2_ticks_to_move = 280 - pid.M2_ticks_diff; //OK
						//Serial.println("Taking 4 steps forward.");
						break;
			
				case 5: pid.M1_ticks_to_move = 285 - pid.M1_ticks_diff; //OK
						pid.M2_ticks_to_move = 280 - pid.M2_ticks_diff; //OK
						//Serial.println("Taking 5 steps forward.");
						break;
						
				default: pid.M1_ticks_to_move = 0;
						 pid.M2_ticks_to_move = 0;
						 break;
			}
		}*/
// ########################################################
	
		// Both motors have positive speed values.
		pid.control(1,1);
		
		// Set the motor commanded power based on the number of ticks it should be moving.
		motorShield.setSpeeds(pid.getRightSpeed(),pid.getLeftSpeed());
		
		// Stop the motors if a fault is detected.
		stopIfFault();
		
// ########################################################
// FOR EXPLORATION, UNCOMMENT THIS PORTION.
// FOR FASTEST PATH, COMMENT THIS PORTION.
//*
		// Include a sampling time.
		delay(10);
			
		// Stop the motor movement if the required distance or step is reached.
		// Step distance and rotation angle are set at the top of this file.
		stopIfReached();
//*/
// ########################################################
// FOR FASTEST PATH, UNCOMMENT THIS PORTION.
// FOR EXPLORATION, COMMENT THIS PORTION.
/*
	
		// If the current forward command is the last command.
		// The robot will keep moving forwards as long as it doesn't detect a wall in front.
		if(lastCommand)
		{
			// Check the sensor readings to see if the robot is too close to the wall in front.
			sensor.readSensor();
			//Serial.println("Inside last command if.");
			
			if(sensor.distanceA0 < 10 or sensor.distanceA1 < 10 or sensor.distanceA2 < 10)
			{
				//Serial.println("Robot stopped last command.");
				
				// Stop the robot and reset the PID controller variables.
				motorShield.setBrakes(400, 400);
				distsub = 0;
				pid.M1_ticks_moved = 0;
				pid.M2_ticks_moved = 0;
				
				// Break out of this function.
				return;
			}
			// Otherwise continue moving forward at a slower pace.
			delay(10);
		}
		else
		{
			// Include a sampling time.
			delay(10);
			
			// Stop the motor movement if the required distance or step is reached.
			// Step distance and rotation angle are set at the top of this file.
			stopIfReached();
		}
//*/
// ########################################################
	}
	
	// Reset the rotation count if the robot moves straight next.
	rotateCount = 0;
}

// Rotate Left 90 Degrees.
void Movement::rotate90left()
{
	pid.setZero();
	//Serial.println("First rotate left transition.");
		
	// Theoretically 398 ticks rotates the robot by approximately 90 degrees.
	// The ticks to move for each motor when rotating have to be individually adjusted.
	pid.M1_ticks_to_move = 310; //OK
	pid.M2_ticks_to_move = 350; //OK

	// Set the boolean variables to keep track of movement transitions.
	straightTransition = true;

	// While there are still steps to move.
	while(distsub > 0)
	{
		// Left motor is negative and right motor is positive.
		pid.control(-1,1);
		motorShield.setSpeeds(pid.getRightSpeed(),pid.getLeftSpeed());
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
			// Move forward by a small amount to get the sensor out of the stuck zone.
			pid.setZero();
			pid.M1_ticks_diff = 0;
			pid.M2_ticks_diff = 0;
			pid.M1_ticks_to_move = 25; //OK
			pid.M2_ticks_to_move = 25; //OK
			distsub = 1;
			straightTransition = false;
			
			while(distsub > 0)
			{
				pid.control(1,1);
				motorShield.setSpeeds(pid.getRightSpeed(),pid.getLeftSpeed());
				stopIfFault();
				delay(10);
				stopIfReached();
			}
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

// Rotate Right 90 Degrees.
void Movement::rotate90right()
{
	//Serial.println("Called rotate right");
	pid.setZero();
	//Serial.println("First rotate right transition.");
		
	pid.M1_ticks_to_move = 333; //OK
	pid.M2_ticks_to_move = 333; //OK

	straightTransition = true;

	while(distsub > 0)
	{
		// Left motor is positive and right motor is negative.
		pid.control(1,-1);
		motorShield.setSpeeds(pid.getRightSpeed(),pid.getLeftSpeed());
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
			// Move forward by a small amount to get the sensor out of the stuck zone.
			pid.setZero();
			pid.M1_ticks_diff = 0;
			pid.M2_ticks_diff = 0;
			pid.M1_ticks_to_move = 25; //OK
			pid.M2_ticks_to_move = 25; //OK
			distsub = 1;
			straightTransition = false;
			
			while(distsub > 0)
			{
				pid.control(1,1);
				motorShield.setSpeeds(pid.getRightSpeed(),pid.getLeftSpeed());
				stopIfFault();
				delay(10);
				stopIfReached();
			}
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
	previousCommand = 'R';
}

// Rotate Left 180 Degrees.
void Movement::rotate180()
{
	pid.setZero();
	//Serial.println("First rotate 180 transition.");

	pid.M1_ticks_to_move = 555; //OK
	pid.M2_ticks_to_move = 733; //OK

	straightTransition = true;
	
	while(distsub > 0)
	{
		// Left motor is negative and right motor is positive.
		pid.control(-1,1);
		motorShield.setSpeeds(pid.getRightSpeed(),pid.getLeftSpeed());
		stopIfFault();
		delay(10);
		stopIfRotated();
	}
}

// Stop the robot after it has moved the required straight distance.
void Movement::stopIfReached()
{
	// Once the desired number of ticks to move the required distance has been reached.
	if(pid.M1_ticks_moved > pid.M1_ticks_to_move and pid.M2_ticks_moved > pid.M2_ticks_to_move)
	{
		//Serial.print("Ticks to move M1: "); Serial.print(M1_ticks_to_move);
		//Serial.print(", Ticks moved M1: "); Serial.println(pid.M1_ticks_moved);
		//Serial.print(", Ticks to move M2: "); Serial.print(M2_ticks_to_move);
		//Serial.print(", Ticks moved M2: "); Serial.println(pid.M2_ticks_moved);

		// Error difference for how many ticks the current step exceeded by.
		pid.M1_ticks_diff = pid.M1_ticks_moved - pid.M1_ticks_to_move;
		pid.M2_ticks_diff = pid.M2_ticks_moved - pid.M2_ticks_to_move;
		
		// Count the total number of ticks over a period of time.
		//Total_M1_moved += M1_ticks_moved;
		//Total_M2_moved += M2_ticks_moved;
		  
		//Serial.print("R ticks moved : "); Serial.print(M1_ticks_moved);
		//Serial.print(", L ticks moved : "); Serial.println(M2_ticks_moved);
		//Serial.print(", Total right ticks moved : "); Serial.print(Total_M1_moved);
		//Serial.print(", Total left ticks moved : "); Serial.println(Total_M2_moved);

		// Decrement the number of steps left to travel.
		distsub--;
		
// ########################################################
		// FOR FASTEST PATH, UNCOMMENT THIS PORTION.
		// FOR EXPLORATION, COMMENT THIS PORTION.
		
		/*sensor.readSensor();
		
		// If the robot moves too close to an obstacle in front when moving forwards,
		// Set the 'distsub' to zero, apply the brakes and perform subsequent corrections.
		if(sensor.distanceA0 < 10 or sensor.distanceA1 < 10 or sensor.distanceA2 < 10)
		{
			pid.M1_ticks_moved = pid.M1_ticks_to_move;
			pid.M2_ticks_moved = pid.M1_ticks_to_move;
			distsub = 0;
		}*/
// ########################################################

		// The robot should only apply the brakes when it has finished the last step, 
		// Before stopping and waiting for a command.
		if(distsub <= 0)
		{
		  // Set the brakes on both motors to bring the robot to a stop.
		  //motorShield.setM1Brake(400);
		  //motorShield.setM2Brake(400);
		  motorShield.setBrakes(400, 400);
		}
		// Reset the tick counters.
		pid.M1_ticks_moved = 0;
		pid.M2_ticks_moved = 0;
	}
}

// Stop the robot after it has rotated by the required angle.
void Movement::stopIfRotated()
{
	// Once the number of ticks moved by both wheels exceed the ticks to be moved for each wheel.
	if(pid.M1_ticks_moved > pid.M1_ticks_to_move and pid.M2_ticks_moved > pid.M2_ticks_to_move)
	{
		//Serial.print("Ticks to move M1: "); Serial.print(M1_ticks_to_move);
		//Serial.print(", Ticks moved M1: "); Serial.println(pid.M1_ticks_moved);
		//Serial.print(", Ticks to move M2: "); Serial.print(M2_ticks_to_move);
		//Serial.print(", Ticks moved M2: "); Serial.println(pid.M2_ticks_moved);

		// NOTE: ERROR IN TICKS DIFFERENCE IS NOT NEEDED HERE AS EACH ROTATION IS INDEPENDENT.
		  
		// Count the total number of ticks over a period of time.
		//Total_M1_moved += M1_ticks_moved;
		//Total_M2_moved += M2_ticks_moved;
		  
		//Serial.print("R ticks moved : "); Serial.print(M1_ticks_moved);
		//Serial.print(", L ticks moved : "); Serial.print(M2_ticks_moved);
		//Serial.print(", Total right ticks moved : "); Serial.print(Total_M1_moved);
		//Serial.print(", Total left ticks moved : "); Serial.println(Total_M2_moved);

		// Decrement the number of steps left to travel.
		distsub--;

		if(distsub == 0)
		{
		  motorShield.setBrakes(400, 400);
		}
		// Reset the tick counters.
		pid.M1_ticks_moved = 0;
		pid.M2_ticks_moved = 0;
	}
}

// Stops the motor if there is a fault, like a short circuit.
// Infinite loop stops the program from continuing.
void Movement::stopIfFault()
{
	if (motorShield.getM1Fault())
	{
		//Serial.println("Left motor fault.");
		//while(1);
	}
	if (motorShield.getM2Fault())
	{
		//Serial.println("Right motor fault.");
		//while(1);
	}
}

// Read the sensor distance values when the robot stops moving.
void Movement::readSensor()
{
	sensor.readSensor();
}

// Print the sensor values in terms of distance in steps from obstacle.
void Movement::printSensor()
{
	sensor.print();
}

// Function to increment the encoder's right ticks.
void Movement::right_tick_increment()
{
	pid.right_ticks_increment();
}

// Function to increment the encoder's left ticks.
void Movement::left_tick_increment()
{
	pid.left_ticks_increment();
}