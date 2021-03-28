// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT FORWARDS IN STEPS OF 10CM FILE.

// Include the movement header file.
#include "Movement.h"

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

		pid.M1_ticks_to_move = 225; //OK
		pid.M2_ticks_to_move = 225; //OK

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
		
		pid.M1_ticks_to_move = 227 - pid.M1_ticks_diff; //OK
		pid.M2_ticks_to_move = 227 - pid.M2_ticks_diff; //OK
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
					
				case 2: pid.M1_ticks_to_move = 265 - pid.M1_ticks_diff; //OK
						pid.M2_ticks_to_move = 255 - pid.M2_ticks_diff; //OK
						//Serial.println("Taking 2 steps forward.");
						break;
			
				case 3: pid.M1_ticks_to_move = 275 - pid.M1_ticks_diff; //OK
						pid.M2_ticks_to_move = 275 - pid.M2_ticks_diff; //OK
						//Serial.println("Taking 3 steps forward.");
						break;
			
				case 4: pid.M1_ticks_to_move = 280 - pid.M1_ticks_diff; //OK
						pid.M2_ticks_to_move = 280 - pid.M2_ticks_diff; //OK
						//Serial.println("Taking 4 steps forward.");
						break;
			
				case 5: pid.M1_ticks_to_move = 280 - pid.M1_ticks_diff; //OK
						pid.M2_ticks_to_move = 285 - pid.M2_ticks_diff; //OK
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
		motorShield.setSpeeds(pid.getLeftSpeed(),pid.getRightSpeed());
		
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
