// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 CORRECTION CODE FILE.

// Include the movement header file.
#include "Movement.h"

// Check and correct distance from front using front sensors.
void Movement::frontCalibrate()
{
	error = 0;
	error_margin = 0.1;
	perfDist = 9;
	
	//Serial.println("frontDistanceCheck called.");
	
	// Read in the sensor values.
	sensor.readSensor();
	
	// Comparing front left and front right sensor values.
	if(sensor.distanceA0 < 25 and sensor.distanceA2 < 25)
	{
		error = (sensor.distanceA0 + sensor.distanceA2) / 2 - perfDist;
		
		Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2); Serial.print(" Error: ");Serial.println(error); 
		
		//for(int i = 0; i < tiltCount; i++)
		//{
			while(abs(error) > error_margin and sensor.distanceA0 < 25 and sensor.distanceA2 < 25)
			{
				Serial.println("Comparing A0 AND A2.");
				
				// If the robot front is too close to an obstacle.
				if(error < 0)
				{
					Serial.println("Too close to front. Moving backwards for correction.");
					motorShield.setSpeeds(-110,-100);
				}
				// If the robot front is too far from an obstacle.
				else if(error > 0)
				{
					Serial.println("Too far from front. Moving forwards for correction.");
					motorShield.setSpeeds(110,100);
				}
				
				// Read in the sensor values.
				sensor.readSensor();
				error = (sensor.distanceA0 + sensor.distanceA2) / 2 - perfDist;
			}
		//}
		motorShield.setBrakes(400, 400);
		delay(500);
		
		// Perform the tilt check using front sensors after adjusting the front distance from an obstacle,
		// To account for the sensor blind spot.
		frontWallCheckTilt();
		delay(500);
	}
}

// Check and correct tilt angle using right side sensors.
void Movement::rightCalibrate()
{
	//Serial.println("rightWallCheckTilt called.");
	
	error = 0;
	error_margin = 0.1;
	
	// Read in the sensor values.
	sensor.readSensor();
	
	if(sensor.distanceA3 < 25 and sensor.distanceA4 < 25)
	{
		error = (sensor.distanceA3 - 0.25) - (sensor.distanceA4 + 0.4);
		
		Serial.print("Right front sensor: "); Serial.print(sensor.distanceA3 - 0.25); Serial.print(", Right back sensor: "); Serial.print(sensor.distanceA4 + 0.4); Serial.print(" Error: ");Serial.println(error); 
		
		for(int i = 0; i < tiltCount; i++)
		{
			if(abs(error) > error_margin and sensor.distanceA3 < 35 and sensor.distanceA4 < 35)
			{
				// Perform adjustments if the values are different.
				// Tilt must be significant enough for sensors to detect at least 1cm difference.
				
				// If the robot is tilted left.
				if(error > 0)
				{
					Serial.println("A4, A5, Tilted left. Tilting right for correction.");
					motorShield.setSpeeds(100,-100);
					delay(50);
					motorShield.setBrakes(400, 400);
					delay(100);
					sensor.readSensor();
					error = (sensor.distanceA3 - 0.25) - (sensor.distanceA4 + 0.1);
				}
				
				// If the robot is tilted right.
				else if(error < 0)
				{
					Serial.println("A4, A5, Tilted right. Tilting left for correction.");
					motorShield.setSpeeds(-120,120);
					delay(50);
					motorShield.setBrakes(400, 400);
					delay(100);
					sensor.readSensor();
					error = (sensor.distanceA3 - 0.25) - (sensor.distanceA4 + 0.4);
				}
				
				Serial.print("Right front sensor: "); Serial.print(sensor.distanceA3 - 0.25); Serial.print(", Right back sensor: "); Serial.println(sensor.distanceA4 + 0.4); Serial.print(" Error: ");Serial.println(error);
			}
		}
		// Set the brakes once the correct tilt correction angle has been reached.
		motorShield.setBrakes(400, 400);
	}
}

void Movement::rightWallDistCheck()
{
	// First calibrate by right, then by front.
	rightCalibrate();
	delay(100);
	frontCalibrate();
	delay(100);
	
	// Read sensor values and determine if the right side of the robot is too far or close to the wall.
	sensor.readSensor();
	
	if((sensor.distanceA3 < 7 and sensor.distanceA4 < 7) or (sensor.distanceA3 > 14 and sensor.distanceA4 > 14 and sensor.distanceA3 < 20 and sensor.distanceA4 < 20))
	{
		// ---------------------------------------------------------
		// CHECKING BOTH FRONT AND RIGHT, FRONT FIRST THEN RIGHT.
		Serial.println("Right side too close or far.");
		
		// Rotate the robot right.
		distsub = 1;
		rotate90right();
		delay(100);
		
		// Calibrate by front of the robot, which would now be facing right.
		frontCalibrate();
		delay(100);
		
		// Rotate the robot left.
		distsub = 1;
		rotate90left();
		delay(100);
		
		// Perform right side calibration.
		rightCalibrate();
	}
}

// Check and correct tilt angle using front sensors.
void Movement::frontWallCheckTilt()
{
	//Serial.println("frontWallCheckTilt called.");
	error = 0;
	error_margin = 0.1;
	
	sensor.readSensor();
	
	// Comparing front left and front right sensor values.
	if(sensor.distanceA0 < 35 and sensor.distanceA2 < 35)
	{
		error = (sensor.distanceA0 + 1.6) - (sensor.distanceA2);
		
		Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0 + 1.6); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2); Serial.print(" Error: ");Serial.println(error); 
		
		for(int i = 0; i < tiltCount; i++)
		{
			if(abs(error) > error_margin and (sensor.distanceA0) < 25 and sensor.distanceA2 < 25)
			{
				if(error > 0 )
				{
					Serial.println("A0, A2, Tilted left. Tilting right for correction.");
					motorShield.setSpeeds(100,-100);
					delay(50);
					motorShield.setBrakes(400, 400);
					delay(100);
					sensor.readSensor();
					error = (sensor.distanceA0 + 1.6) - (sensor.distanceA2);
				}
				
				else if(error < 0 )
				{
					Serial.println("A0, A2, Tilted right. Tilting left for correction.");
					motorShield.setSpeeds(-100,100);
					delay(50);
					motorShield.setBrakes(400, 400);
					delay(100);
					sensor.readSensor();
					error = (sensor.distanceA0 + 1.6) - (sensor.distanceA2);
				}
				
				Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0 + 1.6); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2); Serial.print(" Error: ");Serial.println(error); 
			}
		}
		motorShield.setBrakes(400, 400);
		delay(500);
	}
}
