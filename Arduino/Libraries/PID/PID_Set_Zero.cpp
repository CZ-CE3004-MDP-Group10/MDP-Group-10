// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 PID RESET VARIABLES FILE.

// Include the PID controller header file.
#include "PID.h"

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
