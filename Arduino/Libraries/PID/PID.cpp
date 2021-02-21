#include "PID.h"

void PID::init()
{
	right_speed = 0.0;
	left_speed = 0.0;
	
	// Tick counters.
	right_ticks = 0;
	left_ticks = 0;
	M1_ticks_moved = 0;
	M2_ticks_moved = 0;
	M1_ticks_diff = 0;
	M2_ticks_diff = 0;
	Total_M1_moved = 0;
	Total_M2_moved = 0;
	M1_ticks_to_move = 0;    // Individual number of ticks to move is specified in each scenario.
	M2_ticks_to_move = 0;
	M1_setpoint_ticks = 10;    // Number of ticks before each iteration of PID controller, also controls motor speed.
	M2_setpoint_ticks = 10;    // A larger value here corresponds to the robot moving faster.

	// Proportional Integral Derivative (PID) controller.
	M1_ticks_PID = 0;
	M2_ticks_PID = 0;
	E1_error_ticks = 0;
	E2_error_ticks = 0;
	E1_prev_error = 0;
	E2_prev_error = 0;
	E1_sum_error = 0;
	E2_sum_error = 0;
	KP = 0.2;        // Adjust for proportional component.
	KD = 0.3;        // Adjust for derivative component.
	KI = 7;          // Adjust for integral component.
}

// Increment the number of ticks at the rising edge of the right motor encoder square wave.
void PID::right_ticks_increment()
{
	right_ticks++;
}

// Increment the number of ticks at the rising edge of the left motor encoder square wave.
void PID::left_ticks_increment()
{
	left_ticks++;
}

// Input multipliers determine the rotation direction of each wheel.
void PID::control(int right_mul , int left_mul)
{
	// Sum up the total number of ticks per iteration of PID computation.
	M1_ticks_moved += right_ticks;
	M2_ticks_moved += left_ticks;

	// Determine the error difference in the number of ticks.
	E1_error_ticks = M1_setpoint_ticks - right_ticks;
	E2_error_ticks = M2_setpoint_ticks - left_ticks;

	// NOTE: PID CALCULATION 'KP', 'KI', 'KD' VALUES NEED TO BE MANUALLY TUNED FOR EACH MOTOR.
  
	// Perform PID calculation for the new motor speed for the right motor.
	M1_ticks_PID = right_ticks + (E2_error_ticks * KP) + (E2_prev_error * KD) + (E2_sum_error * KI * 1.19);

	// Perform PID calculation for the new motor speed for the left motor.
	M2_ticks_PID = left_ticks + (E1_error_ticks * KP * 0.95) + (E1_prev_error * (KD + 0.2)) + (E1_sum_error * KI * 0.98);

	// For debugging.
	//Serial.print(", Right (M1) PID ticks: "); Serial.print(M1_ticks_PID);
	//Serial.print(", Left (M2) PID ticks: "); Serial.println(M2_ticks_PID);

	// Convert the adjusted ticks to the new motor power using the functions below.
	right_speed = right_ticks_to_power(M1_ticks_PID) * right_mul;
	left_speed = left_ticks_to_power(M2_ticks_PID) * left_mul;
	
	// Set both motors speed and direction with the multiplier simultaneously.
	//Syntax: motorShield.setSpeeds(M1Speed (right), M2Speed (left));
	//motorShield.setSpeeds(right_speed * right_mul, left_speed * left_mul);
	//stopIfFault();

	// For debugging.
	//Serial.print(", R power: "); Serial.print(right_speed);
	//Serial.print(", L power: "); Serial.println(left_speed);

	// Reset the tick count for the next iteration after the PID computation is performed.
	right_ticks = 0;
	left_ticks = 0;

	// Store the last error value for the next computation.
	E1_prev_error = E1_error_ticks;
	E2_prev_error = E2_error_ticks;

	// Store the sum of all errors. Errors with negative values are subtracted from the sum.
	E1_sum_error += E1_error_ticks;
	E2_sum_error += E2_error_ticks;

	//Serial.print("M1 Error: "); Serial.print(E1_error_ticks); Serial.print(", M2 error: "); Serial.print(E2_error_ticks);
	//Serial.print(", M1 prev error: "); Serial.print(E1_prev_error); Serial.print(", M2 prev error: "); Serial.print(E2_prev_error);
	//Serial.print(", M1 sum error: "); Serial.print(E1_sum_error); Serial.print(", M2 sum error: "); Serial.print(E2_sum_error);
	//Serial.println();

	// Perform the PID computation again after a sampling time.
	//delay(10);
}

int PID::getRightTicks()
{
	return right_ticks;
}

int PID::getLeftTicks()
{
	return left_ticks;
}

// Based on obtained data of ticks moved vs set power for right motor,
// Convert right motor ticks to commanded power.
double PID::right_ticks_to_power(double right_ticks)
{
  return (right_ticks * 0.22748) + 20.06839;
}

// Based on obtained data of ticks moved vs set power for left motor,
// Convert left motor ticks to commanded power.
double PID::left_ticks_to_power(double left_ticks)
{
  return (left_ticks * 0.221907) + 31.70289;
}

void PID::setZero()
{
	E1_prev_error = 0;
	E2_prev_error = 0;

	E1_sum_error = 110;
	E2_sum_error = 95;

	right_ticks = 0;
	left_ticks = 0;
}

/*void PID::stopIfFault()
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
}*/