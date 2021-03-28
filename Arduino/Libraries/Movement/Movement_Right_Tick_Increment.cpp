// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT RIGHT TICK INCREMENT FILE.

// Include the movement header file.
#include "Movement.h"

// Function to increment the encoder's right ticks.
void Movement::right_tick_increment()
{
	pid.right_ticks_increment();
}
