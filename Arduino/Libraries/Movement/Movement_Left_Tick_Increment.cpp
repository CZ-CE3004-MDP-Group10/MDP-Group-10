// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT LEFT TICK INCREMENT FILE.

// Include the movement header file.
#include "Movement.h"

// Function to increment the encoder's left ticks.
void Movement::left_tick_increment()
{
	pid.left_ticks_increment();
}