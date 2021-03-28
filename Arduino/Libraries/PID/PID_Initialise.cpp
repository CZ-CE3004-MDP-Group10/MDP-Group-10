// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 PID INITIALISATION FILE.

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
	
	// Adjust the proportional, integral and derivative components for left and right motor.
	// These values require offsets for fine tuning each motor's accuracy.
	Left_Proportion = 0.2;
	Left_Integral = 0.3 * 1.19;
	Left_Derivative = 7;
	
	Right_Proportion = 0.2 * 0.95;
	Right_Integral = 0.3 * 0.98;
	Right_Derivative = 7 + 0.2;
}
