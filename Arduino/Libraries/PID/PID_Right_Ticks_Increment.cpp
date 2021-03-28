// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 PID INCREMENT RIGHT TICKS FILE.

// Include the PID controller header file.
#include "PID.h"

// Increment the number of ticks at the rising edge of the right motor encoder square wave.
void PID::right_ticks_increment()
{
	right_ticks++;
}
