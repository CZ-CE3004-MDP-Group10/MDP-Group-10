// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 CORRECTION OF DISTANCE FROM OBSTACLE IN FRONT FILE.

// Include the movement header file.
#include "Movement.h"

// Check and correct distance from front using front sensors.
void Movement::frontDistanceCheck()
{
	error = 0;
	error_margin = 0.1;
	
	// Specify the distance of the robot's front sensors from the obstacle or wall in front of it.
	perfDist = 8.5;
	
	// Read in the sensor values.
	sensor.readSensor();
	
	// If the distance between the frontmost left and right sensors is greater than a specific threshold,
	// The robot is possibly facing a staircase shaped obstacle and should not calibrate.
	if(abs(sensor.distanceA0 - sensor.distanceA2) > 6)
	{
		return;
	}
	
	// Comparing front left and front right sensor values.
	if(sensor.distanceA0 < limit and sensor.distanceA2 < limit)
	{
		error = (sensor.distanceA0 + sensor.distanceA2) / 2 - perfDist;
		
		//Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2); Serial.print(" Error: ");Serial.println(error); 
		
		for(i = 0; i < 100; i++)
		{
			if(abs(error) > error_margin and sensor.distanceA0 < limit and sensor.distanceA2 < limit)
			{
				//Serial.println("Comparing A0 AND A2.");
					
				// If the robot front is too close to an obstacle.
				if(error < 0)
				{
					//Serial.println("Too close to front. Moving backwards for correction.");
					motorShield.setSpeeds(-110,-100);
				}
				// If the robot front is too far from an obstacle.
				else if(error > 0)
				{
					//Serial.println("Too far from front. Moving forwards for correction.");
					motorShield.setSpeeds(110,100);
				}
					
				// Read in the sensor values.
				sensor.readSensor();
				
				// Check the sensor values for debugging.
				//Serial.println(sensor.distanceA0);
				//Serial.println(sensor.distanceA2);
				
				error = (sensor.distanceA0 + sensor.distanceA2) / 2 - perfDist;
			}
		}
		// Stop the robot once it is at approxiately the right distance from the front of the wall.
		motorShield.setBrakes(400, 400);
		delay(500);
	}
}
