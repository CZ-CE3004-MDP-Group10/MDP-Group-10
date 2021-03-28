// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 PID INCREMENT LEFT TICKS FILE.

// Include the PID controller header file.
#include "PID.h"

// Increment the number of ticks at the rising edge of the left motor encoder square wave.
void PID::left_ticks_increment()
{
	left_ticks++;
}
