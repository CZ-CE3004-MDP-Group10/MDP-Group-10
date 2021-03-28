// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 SENSORS CALCULATE OBSTACLE STEP DISTANCE FOR LONG RANGE FILE.

// Include the sensors header file.
#include "Sensors.h"

// Convert distance into steps of 10cm from obstacle block for long range infrared sensor.
double Sensors::calculateStepLong(double dist)
{	
	// IF THE OBSTACLE NEXT TO THE ROBOT IS TREATED AS ONE STEP AWAY.
	// Obstacle is 1 step away.
	if(dist < 24) {return 1;}
	
	// Obstacle is 2 steps away.
	else if( dist >= 24 and dist < 33.5) {return 2;}
	
	// Obstacle is 3 steps away.
	else if(dist >= 33.5 and dist < 42) {return 3;}
	
	// Obstacle is 4 steps away.
	else if(dist >= 42 and dist < 45.5) {return 4;}
	
	// Obstacle is 5 steps away.
	else if(dist >= 45.5 and dist < 47.5) {return 5;}
	
	// Too far to be detected.
	else if(dist >= 47.5) {return 0;}
}
