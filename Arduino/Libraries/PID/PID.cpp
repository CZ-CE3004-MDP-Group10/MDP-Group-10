// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 PID CONTROLLER CODE FILE.

// Include the PID controller header file.
#include "PID.h"

// Initialize the PID controller.
void PID::init()
{
	// Motor M1 - Encoder E1 - Left Motor.
	// Motor M2 - Encoder E2 - Right Motor.
	
	// Motor commanded power.
	left_speed = 0.0;
	right_speed = 0.0;
	
	// Tick counters.
	left_ticks = 0;
	right_ticks = 0;
	M1_ticks_moved = 0;
	M2_ticks_moved = 0;
	M1_ticks_diff = 0;
	M2_ticks_diff = 0;
	Total_M1_moved = 0;
	Total_M2_moved = 0;
	
	// Individual number of ticks to move is specified in each scenario.
	M1_ticks_to_move = 0;
	M2_ticks_to_move = 0;
	
	// Number of ticks before each iteration of PID controller, also controls motor speed.
	// A larger value here corresponds to the robot moving faster, but can result in loss of accuracy.
	M1_setpoint_ticks = 9;
	M2_setpoint_ticks = 9;
	
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

// Increment the number of ticks at the rising edge of the left motor encoder square wave.
void PID::left_ticks_increment()
{
	left_ticks++;
}

// Increment the number of ticks at the rising edge of the right motor encoder square wave.
void PID::right_ticks_increment()
{
	right_ticks++;
}

// PID controller - Input multipliers determine the rotation direction of each wheel.
void PID::control(int left_mul , int right_mul)
{
	// Sum up the total number of ticks per iteration of PID computation.
	M1_ticks_moved += left_ticks;
	M2_ticks_moved += right_ticks;

	// Determine the error difference in the number of ticks.
	E1_error_ticks = M1_setpoint_ticks - left_ticks;
	E2_error_ticks = M2_setpoint_ticks - right_ticks;

	// NOTE: PID CALCULATION 'KP', 'KI', 'KD' VALUES NEED TO BE MANUALLY TUNED FOR EACH MOTOR.
  
	// Perform PID calculation for the new motor speed for the right motor.
	M1_ticks_PID = left_ticks + (E1_error_ticks * KP) + (E1_prev_error * KD) + (E1_sum_error * KI * 1.19);

	// Perform PID calculation for the new motor speed for the left motor.
	M2_ticks_PID = right_ticks + (E2_error_ticks * KP * 0.95) + (E2_prev_error * (KD + 0.2)) + (E2_sum_error * KI * 0.98);

	//Serial.print(", Left (M1) PID ticks: "); Serial.print(M1_ticks_PID);
	//Serial.print(", Right (M2) PID ticks: "); Serial.println(M2_ticks_PID);

	// Convert the adjusted ticks to the new motor power using the functions below.
	left_speed = left_ticks_to_power(M1_ticks_PID) * left_mul;
	right_speed = right_ticks_to_power(M2_ticks_PID) * right_mul;

	//Serial.print(", L power: "); Serial.println(left_speed);
	//Serial.print(", R power: "); Serial.print(right_speed);

	// Reset the tick count for the next iteration after the PID computation is performed.
	left_ticks = 0;
	right_ticks = 0;

	// Store the last error value for the next computation.
	E1_prev_error = E1_error_ticks;
	E2_prev_error = E2_error_ticks;

	// Store the sum of all errors. Errors with negative values are subtracted from the sum.
	E1_sum_error += E1_error_ticks;
	E2_sum_error += E2_error_ticks;

	//Serial.print("M1 Left Error: "); Serial.print(E1_error_ticks); Serial.print(", M2 Right error: "); Serial.print(E2_error_ticks);
	//Serial.print(", M1 prev error: "); Serial.print(E1_prev_error); Serial.print(", M2 prev error: "); Serial.print(E2_prev_error);
	//Serial.print(", M1 sum error: "); Serial.print(E1_sum_error); Serial.print(", M2 sum error: "); Serial.println(E2_sum_error);
}

// Returns the commanded power for the right motor.
int PID::getRightSpeed()
{
	return right_speed;
}

// Returns the commanded power for the left motor.
int PID::getLeftSpeed()
{
	return left_speed;
}

// Convert right motor ticks to commanded power.
double PID::right_ticks_to_power(double right_ticks)
{
  return (right_ticks * 0.221907) + 31.70289;
}

// Convert left motor ticks to commanded power.
double PID::left_ticks_to_power(double left_ticks)
{
  return (left_ticks * 0.22748) + 20.06839;
}

// Reset the PID controller values after each movement.
void PID::setZero()
{
	// Reset the PID parameters when changing direction.
	E1_prev_error = 0;
	E2_prev_error = 0;

	// Optimal sum of square errors for E1 and E2 are 110 and 95 respectively.
	E1_sum_error = 95;
	E2_sum_error = 110;

	// Reset the tick counters as there would have been slight movement between the time
	// When the tick counters are last reset in the PID function and when the brakes are applied.
	left_ticks = 0;
	right_ticks = 0;
}