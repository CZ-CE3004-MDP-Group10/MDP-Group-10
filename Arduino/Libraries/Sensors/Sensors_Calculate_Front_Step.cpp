// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 SENSORS CALCULATE OBSTACLE STEP DISTANCE FROM FRONT FILE.

// Include the sensors header file.
#include "Sensors.h"

// Convert distance into steps of 10cm from obstacle block for short range infrared sensor for the front sensors.
double Sensors::calculateStepFront(double dist)
{
	// IF THE OBSTACLE NEXT TO THE ROBOT IS TREATED AS ONE STEP AWAY.
	// Obstacle is 1 step away.
	if(dist < 12) {return 1;}
	
	// Obstacle is 2 steps away.
	else if(dist >= 12 and dist < 25) {return 2;}
	
	// Too far to be detected.
	else if(dist >= 25) {return 0;}
}
