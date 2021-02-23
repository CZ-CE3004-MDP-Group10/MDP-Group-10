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
	//rotateTransition = true;
	loopSwitchCase = true;
}

// Move Forwards.
void Movement::forwards()
{
	// The normal distsub value is used for decrementing the movement steps.
	// The constant value distsub is used for the switch case operation to determine the number
	// Of ticks that are required for steps 1 to 5.
	distsubConstant = distsub;
	
	// Reset the PID controller variables after each movement.
	pid.setZero();

	// Before moving forwards after a rotation or stationary stop, reset the differences in ticks.
	pid.M1_ticks_diff = 0;
	pid.M2_ticks_diff = 0;

	// If the robot has to move straight after changing direction.
	if(straightTransition and distsub == 1)
	{
		// Theoretically 298 ticks moves the robot forward by approximately 10cm.
		Serial.println("First Straight Transition.");

		pid.M1_ticks_to_move = 221; //OK
		pid.M2_ticks_to_move = 221; //OK

		// Set boolean values to account for observed transition accuracy errors.
		straightTransition = false;
		//rotateTransition = true;
		
		// If it is moving straight after a rotation transition, the switch case within the while
		// Loop body for forward movement below should not run at all.
		// The tick value set above will then move the robot one step forward.
		loopSwitchCase = false;
	}
	else
	{
		// If it is moving straight but not after a transition, iterate through the switch case
		// Statement with each decrement of distsub.
		loopSwitchCase = true;
	}

	// Keep running while it is executing a command and has not reached the last step.
	while(distsub > 0)
	{
		// If it is not the first transition after rotation and distsub is not equal to 1.
		if(loopSwitchCase)
		{
			// To account for discrepancies in the distance moving forward, we need to include a switch case
			// Statement here with 5 cases for 5 possible step distances when moving forwards up to 5 steps.
			switch(distsubConstant)
			{
				// When moving in a straight line, need to include correction for each number of steps.
				case 1: pid.M1_ticks_to_move = 273 - pid.M1_ticks_diff; //OK
						pid.M2_ticks_to_move = 273 - pid.M2_ticks_diff; //OK
						Serial.println("Taking 1 step forward.");
						break;
					
				case 2: pid.M1_ticks_to_move = 273 - pid.M1_ticks_diff; //TO ADJUST
						pid.M2_ticks_to_move = 273 - pid.M2_ticks_diff; //TO ADJUST
						Serial.println("Taking 2 steps forward.");
						break;
			
				case 3: pid.M1_ticks_to_move = 273 - pid.M1_ticks_diff; //TO ADJUST
						pid.M2_ticks_to_move = 273 - pid.M2_ticks_diff; //TO ADJUST
						Serial.println("Taking 3 steps forward.");
						break;
			
				case 4: pid.M1_ticks_to_move = 273 - pid.M1_ticks_diff; //TO ADJUST
						pid.M2_ticks_to_move = 273 - pid.M2_ticks_diff; //TO ADJUST
						Serial.println("Taking 4 steps forward.");
						break;
			
				case 5: pid.M1_ticks_to_move = 273 - pid.M1_ticks_diff; //TO ADJUST
						pid.M2_ticks_to_move = 273 - pid.M2_ticks_diff; //TO ADJUST
						Serial.println("Taking 5 steps forward.");
						break;
						
				default: pid.M1_ticks_to_move = 0;
						 pid.M2_ticks_to_move = 0;
						 break;
			}
		}
	
		// Both motors have positive speed values.
		pid.control(1,1);
		
		// Set the motor commanded power based on the number of ticks it should be moving.
		motorShield.setSpeeds(pid.getRightSpeed(),pid.getLeftSpeed());
		
		// Stop the motors if a fault is detected.
		stopIfFault();
		
		// Include a sampling time.
		delay(10);
		
		// Stop the motor movement if the required distance or step is reached.
		// Step distance and rotation angle are set at the top of this file.
		stopIfReached();
	}
}

// Rotate Left 90 Degrees.
void Movement::rotate90left()
{
	Serial.println("First rotate left transition.");
	pid.setZero();

	//if(rotateTransition)
	//{
	// Theoretically 398 ticks rotates the robot by approximately 90 degrees.
	// The ticks to move for each motor when rotating have to be individually adjusted.
	pid.M1_ticks_to_move = 320; //OK
	pid.M2_ticks_to_move = 325; //OK

	//rotateTransition = false;
	straightTransition = true;
	//}
	
	// Include this portion only if the robot will make 2 consecutive 90 degree left turns.
	/*else
	{
		pid.M1_ticks_to_move = 300; //OK
		pid.M2_ticks_to_move = 330; //OK
	}*/

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

// Rotate Right 90 Degrees.
void Movement::rotate90right()
{
	Serial.println("First rotate right transition.");
	pid.setZero();

	//if(rotateTransition)
	//{
	pid.M1_ticks_to_move = 305; //OK
	pid.M2_ticks_to_move = 335; //OK

	//rotateTransition = false;
	straightTransition = true;
	//}
	
	// Include this portion only if the robot will make 2 consecutive 90 degree right turns.
	/*else
	{
		// Need to add offsets to tick values when rotating right.
		pid.M1_ticks_to_move = 320; //OK
		pid.M2_ticks_to_move = 330; //OK
	}*/

	while(distsub > 0)
	{
		// Left motor is positive and right motor is negative.
		pid.control(1,-1);
		motorShield.setSpeeds(pid.getRightSpeed(),pid.getLeftSpeed());
		stopIfFault();
		delay(10);
		stopIfRotated();
	}
}

// Rotate Left 180 Degrees.
void Movement::rotate180()
{
	Serial.println("First rotate 180 transition.");
	pid.setZero();

	pid.M1_ticks_to_move = 565; //OK
	pid.M2_ticks_to_move = 740; //OK

	//rotateTransition = false;
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
		Serial.print(", Ticks moved M1: "); Serial.println(pid.M1_ticks_moved);
		//Serial.print(", Ticks to move M2: "); Serial.print(M2_ticks_to_move);
		Serial.print(", Ticks moved M2: "); Serial.println(pid.M2_ticks_moved);

		// Error difference for how many ticks the current step exceeded by.
		pid.M1_ticks_diff = pid.M1_ticks_moved - pid.M1_ticks_to_move;
		pid.M2_ticks_diff = pid.M2_ticks_moved - pid.M2_ticks_to_move;
		  
		// For debugging, two master counters count the total number of ticks moved through
		// the entire motor operation to check for tick discrepancies over time.
		//Total_M1_moved += M1_ticks_moved;
		//Total_M2_moved += M2_ticks_moved;
		  
		//Serial.print("R ticks moved : "); Serial.print(M1_ticks_moved);
		//Serial.print(", L ticks moved : "); Serial.println(M2_ticks_moved);
		//Serial.print(", Total right ticks moved : "); Serial.print(Total_M1_moved);
		//Serial.print(", Total left ticks moved : "); Serial.println(Total_M2_moved);

		// Decrement the number of steps left to travel.
		distsub--;

		// The robot should only apply the brakes when it has finished the last step.
		// The robot should only stop and wait for a command after its last step.
		if(distsub == 0)
		{
		  // Set the brakes on both motors to bring the robot to a stop.
		  //motorShield.setM1Brake(400);
		  //motorShield.setM2Brake(400);
		  motorShield.setBrakes(400, 400);

		  // When the robot stops moving after its last step, read in sensor data.
		  sensor.readSensor();
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
		Serial.print(", Ticks moved M1: "); Serial.println(pid.M1_ticks_moved);
		//Serial.print(", Ticks to move M2: "); Serial.print(M2_ticks_to_move);
		Serial.print(", Ticks moved M2: "); Serial.println(pid.M2_ticks_moved);

		// NOTE: ERROR IN TICKS DIFFERENCE IS NOT NEEDED HERE AS EACH ROTATION IS INDEPENDENT.
		  
		// For debugging, two master counters count the total number of ticks moved through
		// the entire motor operation to check for tick discrepancies over time.
		//Total_M1_moved += M1_ticks_moved;
		//Total_M2_moved += M2_ticks_moved;
		  
		//Serial.print("R ticks moved : "); Serial.print(M1_ticks_moved);
		//Serial.print(", L ticks moved : "); Serial.print(M2_ticks_moved);
		//Serial.print(", Total right ticks moved : "); Serial.print(Total_M1_moved);
		//Serial.print(", Total left ticks moved : "); Serial.println(Total_M2_moved);

		// Decrement the number of steps left to travel.
		distsub--;

		// The robot should only apply the brakes when it has finished the last step.
		if(distsub == 0)
		{
		  // Set the brakes on both motors simultaneously to bring the robot to a stop.
		  // Syntax: motorShield.setBrakes(M1 right motor, M2 left motor);
		  motorShield.setBrakes(400, 400);

		  // When the robot stops moving after its last step, read in sensor data.
		  sensor.readSensor();
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
		Serial.println("Left motor fault.");
		while(1);
	}
	if (motorShield.getM2Fault())
	{
		Serial.println("Right motor fault.");
		while(1);
	}
}


void Movement::readSensor()
{
	sensor.readSensor();
}

void Movement::right_tick_increment()
{
	pid.right_ticks_increment();
}

void Movement::left_tick_increment()
{
	pid.left_ticks_increment();
}