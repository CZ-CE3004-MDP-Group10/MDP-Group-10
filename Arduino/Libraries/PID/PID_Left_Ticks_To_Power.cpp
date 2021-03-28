// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 PID CONVERT LEFT TICKS TO POWER FILE.

// Include the PID controller header file.
#include "PID.h"

// Convert left motor ticks to commanded power.
double PID::left_ticks_to_power(double left_ticks)
{
  return (left_ticks * 0.22748) + 20.06839;
}
