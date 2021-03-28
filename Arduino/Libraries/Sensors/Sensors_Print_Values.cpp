// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 SENSORS PRINT VALUES IN STEPS OR CM FILE.

// Include the sensors header file.
#include "Sensors.h"

// Print and send the string over to algorithm.
void Sensors::print()
{	
	// Calculate the distance of the obstacle from the sensor in steps.
	obstacleA0 = calculateStepFront(distanceA0);
	obstacleA1 = calculateStepFront(distanceA1);
	obstacleA2 = calculateStepFront(distanceA2);
	obstacleA3 = calculateStepRight(distanceA3);
	obstacleA4 = calculateStepRight(distanceA4);
	obstacleA5 = calculateStepLong(distanceA5);
	
	if(exitStuckLoop)
	{
		//Serial.println("Detected multiple consecutive left and right rotations. Breaking out of loop.");
		
		// Return a modified version of the obstacle step detection string that marks the right rear sensor and 
		// Front right sensor as having an obstacle one step away, so as to break out of a possible rotational loop.
		Serial.println("ALG|" + String(obstacleA5) + "," + String(obstacleA0) + "," + String(obstacleA1) + ",1," + String(obstacleA3) + ",1");
		
		// Reset the boolean that breaks the robot out of a stuck loop.
		exitStuckLoop = false;
		return;
	}
	else
	{
		// Return the results to algorithm in terms of steps from obstacle.
		Serial.println("ALG|" + String(obstacleA5) + "," + String(obstacleA0) + "," + String(obstacleA1) + "," + String(obstacleA2) + "," + String(obstacleA3) + "," + String(obstacleA4));
		
		// Return the results to algorithm in terms of distance from obstacle.
		//Serial.println("ALG|" + String(distanceA5) + "," + String(distanceA0) + "," + String(distanceA1) + "," + String(distanceA2) + "," + String(distanceA3) + "," + String(distanceA4));
	}
}