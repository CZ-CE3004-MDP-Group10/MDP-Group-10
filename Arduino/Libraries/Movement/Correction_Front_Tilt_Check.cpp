// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 CORRECTION OF TILT ANGLE USING FRONT SENSORS FILE.

// Include the movement header file.
#include "Movement.h"

// Check and correct tilt angle using front sensors.
void Movement::frontTiltCheck()
{
	//Serial.println("frontWallCheckTilt called.");
	error = 0;
	error_margin = 0.1;
	
	sensor.readSensor();
	
	// If the distance between the frontmost left and right sensors is greater than a specific threshold,
	// The robot is possibly facing a staircase shaped obstacle and should not calibrate.
	if(abs(sensor.distanceA0 - sensor.distanceA2) > 4)
	{
		return;
	}
	
	// Comparing front left and front right sensor values.
	if(sensor.distanceA0 < 15 and sensor.distanceA2 < 15)
	{
		error = (sensor.distanceA0 + 1.2) - (sensor.distanceA2 - 0);
		
		//Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0 + 1.2); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2 - 0); Serial.print(" Error: ");Serial.println(error); 
		
		for(i = 0; i < 30; i++)
		{
			if(abs(error) > error_margin and (sensor.distanceA0) < 20 and sensor.distanceA2 < 20)
			{
				// If the robot is tilted left.
				if(error > 0)
				{
					//Serial.println("A0, A2, Tilted left. Tilting right for correction.");
					motorShield.setSpeeds(80,-80);
					delay(50);
					motorShield.setBrakes(400, 400);
					delay(100);
					
					sensor.readSensor();
					error = (sensor.distanceA0 + 1.2) - (sensor.distanceA2 - 0);
				}
				
				// If the robot is tilted right.
				else if(error < 0)
				{
					//Serial.println("A0, A2, Tilted right. Tilting left for correction.");
					motorShield.setSpeeds(-80,80);
					delay(50);
					motorShield.setBrakes(400, 400);
					delay(100);
					
					sensor.readSensor();
					error = (sensor.distanceA0 + 1.2) - (sensor.distanceA2 - 0);
				}
				
				//Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0 + 1.2); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2 - 0); Serial.print(" Error: ");Serial.println(error); 
			}
		}
		motorShield.setBrakes(400, 400);
		delay(500);
	}
}
