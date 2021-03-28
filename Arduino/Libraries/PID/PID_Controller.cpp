// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 PID CONTROLLER FILE.

// Include the PID controller header file.
#include "PID.h"

// PID controller - Input multipliers determine the rotation direction of each wheel.
void PID::control(int left_mul , int right_mul)
{
	// Sum up the total number of ticks per iteration of PID computation.
	M1_ticks_moved += left_ticks;
	M2_ticks_moved += right_ticks;

	// Determine the error difference in the number of ticks.
	E1_error_ticks = M1_setpoint_ticks - left_ticks;
	E2_error_ticks = M2_setpoint_ticks - right_ticks;
  
	// Perform PID calculation for the new motor speed for the right motor.
	M1_ticks_PID = left_ticks + 
	(E1_error_ticks * Left_Proportion) + 
	(E1_prev_error * Left_Derivative) + 
	(E1_sum_error * Left_Integral);

	// Perform PID calculation for the new motor speed for the left motor.
	M2_ticks_PID = right_ticks + 
	(E2_error_ticks * Right_Proportion) + 
	(E2_prev_error * Right_Derivative) + 
	(E2_sum_error * Right_Integral);

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
