// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 SENSORS CALCULATE OBSTACLE STEP DISTANCE FROM RIGHT FILE.

// Include the sensors header file.
#include "Sensors.h"

// Convert distance into steps of 10cm from obstacle block for short range infrared sensor for the right sensors.
double Sensors::calculateStepRight(double dist)
{
	// IF THE OBSTACLE NEXT TO THE ROBOT IS TREATED AS ONE STEP AWAY.
	// Obstacle is 1 step away.
	if(dist < 15) {return 1;}
	
	// Obstacle is 2 steps away.
	else if(dist >= 15 and dist < 24) {return 2;}
	
	// Too far to be detected.
	else if(dist >= 24) {return 0;}
}
