#include "Movement.h"

void Movement::init()
{
	motorShield.init();
	pid.init();
	sensor.init();
	straightTransition = true;
	rotateTransition = true;
}

void Movement::forwards()
{
	// Reset the PID parameters when changing direction.
	pid.E1_prev_error = 0;
	pid.E2_prev_error = 0;

	// Optimal sum of square errors for E1 and E2 are 110 and 95 respectively.
	pid.E1_sum_error = 110;
	pid.E2_sum_error = 95;

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
		rotateTransition = true;
	}
	else
	{
		// To account for discrepancies in the distance moving forward, may need to include a switch case
		// Statement here with 5 cases for 5 possible step distances when moving forwards up to 5 steps.

		// When moving in a straight line for more than 1 step, need to include correction for subsequent steps.
		pid.M1_ticks_to_move = 273 - pid.M1_ticks_diff; //OK
		pid.M2_ticks_to_move = 273 - pid.M2_ticks_diff; //OK
	}

	// Keep running while it is executing a command and has not reached the last step.
	while(distsub > 0)
	{     
		// Both motors have positive speed values.
		pid.control(1,1);
		motorShield.setSpeeds(pid.getRightTicks(),pid.getLeftTicks());
		stopIfFault();
		delay(10);
		
		// Stop the motor movement if the required distance or step is reached.
		// Step distance and rotation angle are set at the top of this file.
		stopIfReached();
	}

	// Reset the tick counters here as there would have been alight movement between the time
	// When the tick counters are last reset in the PID function and when the brakes are applied.
	pid.right_ticks = 0;
	pid.left_ticks = 0;
	
}

void Movement::rotate90left()
{
	Serial.println("First rotate left transition.");
   
	pid.setZero();

	if(rotateTransition)
	{
		// Theoretically 398 ticks rotates the robot by approximately 90 degrees.
		// The ticks to move for each motor when rotating have to be individually adjusted.
		pid.M1_ticks_to_move = 320; //OK
		pid.M2_ticks_to_move = 325; //OK

		rotateTransition = false;
		straightTransition = true;
	}
	else
	{
		pid.M1_ticks_to_move = 300; //OK
		pid.M2_ticks_to_move = 330; //OK
	}

	while(distsub > 0)
	{
		// Left motor is negative and right motor is positive.
		pid.control(-1,1);
		motorShield.setSpeeds(pid.getRightTicks(),pid.getLeftTicks());
		stopIfFault();
		delay(10);

		// When rotating, the number of ticks for both motors is observed to fluctuate
		// Anywhere from 5 to 15 ticks more than the set value at the top of this file.
		stopIfRotated();
	}

	//delay(1000);
}

// Rotating right 90 degrees.
void Movement::rotate90right()
{
	Serial.println("First rotate right transition.");
  
	pid.setZero();

	if(rotateTransition)
	{
		pid.M1_ticks_to_move = 305; //OK
		pid.M2_ticks_to_move = 335; //OK

		rotateTransition = false;
		straightTransition = true;
	}
	else
	{
		// Need to add offsets to tick values when rotating right.
		pid.M1_ticks_to_move = 320; //OK
		pid.M2_ticks_to_move = 330; //OK
	}

	while(distsub > 0)
	{
		// Left motor is positive and right motor is negative.
		pid.control(1,-1);
		motorShield.setSpeeds(pid.getRightTicks(),pid.getLeftTicks());
		stopIfFault();
		delay(10);
		
		stopIfRotated();
	}

	//delay(1000);
}

// Rotating left 180 degrees.
void Movement::rotate180()
{
	Serial.println("First rotate 180 transition.");
  
	pid.setZero();

	pid.M1_ticks_to_move = 565; //OK
	pid.M2_ticks_to_move = 740; //OK

	rotateTransition = false;
	straightTransition = true;

	while(distsub > 0)
	{
		// Left motor is negative and right motor is positive.
		pid.control(-1,1);
		motorShield.setSpeeds(pid.getRightTicks(),pid.getLeftTicks());
		stopIfFault();
		delay(10);
		
		stopIfRotated();
	}

	//delay(1000);
}

// If the robot is meant to go straight.
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

		  // The robot needs to wait for another input command before continuing.
		  //waitingInput = true;

		  // When the robot stops moving after its last step, read in sensor data.
		  sensor.readSensor();
		}

		// Reset the tick counters.
		pid.M1_ticks_moved = 0;
		pid.M2_ticks_moved = 0;
	}
	// Provide a sampline time before the next PID iteration.
	//delay(5);
}

void Movement::stopIfRotated()
{
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
		// The robot should only stop and wait for a command after its last step.
		if(distsub == 0)
		{
		  // Set the brakes on both motors simultaneously to bring the robot to a stop.
		  // Syntax: motorShield.setBrakes(M1 right motor, M2 left motor);
		  motorShield.setBrakes(400, 400);

		  // The robot needs to wait for another input command before continuing.
		  //waitingInput = true;

		  // When the robot stops moving after its last step, read in sensor data.
		  sensor.readSensor();
		}

		// Reset the tick counters.
		pid.M1_ticks_moved = 0;
		pid.M2_ticks_moved = 0;
	}
	// Provide a sampline time before the next PID iteration.
	//delay(5);
}

// Stops the motor if there is a fault, like a short circuit.
void Movement::stopIfFault()
{
	if (motorShield.getM1Fault())
	{
		Serial.println("Left motor fault.");

		// Infinite loop stops the program from continuing.
		while(1);
	}
	if (motorShield.getM2Fault())
	{
		Serial.println("Right motor fault.");

		// Infinite loop stops the program from continuing.
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