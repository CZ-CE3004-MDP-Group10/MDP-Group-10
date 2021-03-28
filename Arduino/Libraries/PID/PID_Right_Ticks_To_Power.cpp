// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 PID CONVERT RIGHT TICKS TO POWER FILE.

// Include the PID controller header file.
#include "PID.h"

// Convert right motor ticks to commanded power.
double PID::right_ticks_to_power(double right_ticks)
{
  return (right_ticks * 0.221907) + 31.70289;
}
