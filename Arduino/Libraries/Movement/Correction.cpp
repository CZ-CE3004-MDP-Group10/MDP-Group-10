// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 CORRECTION CODE FILE.

// Include the movement header file.
#include "Movement.h"

// Check and correct distance from front using front sensors.
void Movement::frontCalibrate()
{
	float error = 0;
	float error_margin = 0.1;
	float perfDist = 12;
	
	Serial.println("frontDistanceCheck called.");
	
	// Read in the sensor values.
	sensor.readSensor();
	
	// Perform check on right side to see if it is too close to or far from the wall.
	if((sensor.distanceA3 < 7 and sensor.distanceA4 < 7) or (sensor.distanceA3 > 13 and sensor.distanceA4 < 13))
	{
// ---------------------------------------------------------
// CHECKING BOTH FRONT AND RIGHT, FRONT FIRST THEN RIGHT.
		Serial.println("Right side too close or far.");
		
		// Rotate the robot right.
		rotate90right();
		
		// Perform front calibration distance check.
		if(sensor.distanceA0 < 25 and sensor.distanceA2 < 25)
		{
			error = (sensor.distanceA0 + sensor.distanceA2) / 2 - perfDist;
			
			Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2); Serial.print(" Error: ");Serial.println(error); 
			
			while(abs(error) > error_margin and sensor.distanceA0 < 25 and sensor.distanceA2 < 25)
			{
				Serial.println("Comparing A0 AND A2.");
				
				if(error < 0)
				{
					Serial.println("Too close to front. Moving backwards for correction.");
					motorShield.setSpeeds(-100,-100);
				}
				else if(error > 0)
				{
					Serial.println("Too far from front. Moving forwards for correction.");
					motorShield.setSpeeds(100,100);
				}
				sensor.readSensor();
				error = (sensor.distanceA0 + sensor.distanceA2) / 2 - perfDist;
			}
			motorShield.setBrakes(400, 400);
			delay(500);
			
			frontWallCheckTilt();
			delay(500);
			
			motorShield.setSpeeds(95, 100);
			delay(500);
			motorShield.setBrakes(400,400);
		}
		
		// Rotate the robot left.
		rotate90left();
		
		// Perform right side calibration.
		rightWallCheckTilt();
	}
// ---------------------------------------------------------
// CHECKING FRONT ONLY.
	else
	{
		// Comparing front left and front right sensor values.
		if(sensor.distanceA0 < 25 and sensor.distanceA2 < 25)
		{
			error = (sensor.distanceA0 + sensor.distanceA2) / 2 - perfDist;
			
			Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2); Serial.print(" Error: ");Serial.println(error); 
			
			while(abs(error) > error_margin and sensor.distanceA0 < 25 and sensor.distanceA2 < 25)
			{
				Serial.println("Comparing A0 AND A2.");
				
				// If the robot front is too close to an obstacle.
				if(error < 0)
				{
					Serial.println("Too close to front. Moving backwards for correction.");
					motorShield.setSpeeds(-100,-100);
				}
				// If the robot front is too far from an obstacle.
				else if(error > 0)
				{
					Serial.println("Too far from front. Moving forwards for correction.");
					motorShield.setSpeeds(100,100);
				}
				
				// Read in the sensor values.
				sensor.readSensor();
				error = (sensor.distanceA0 + sensor.distanceA2) / 2 - perfDist;
			}
			motorShield.setBrakes(400, 400);
			delay(500);
			
			// Perform the tilt check using front sensors after adjusting the front distance from an obstacle,
			// To account for the sensor blind spot.
			frontWallCheckTilt();
			delay(500);
			
			// Move the robot forwards half a step after fixing tilt errors.
			motorShield.setSpeeds(95, 100);
			delay(500);
			motorShield.setBrakes(400,400);
		}
	}
}

// Check and correct tilt angle using right side sensors.
void Movement::rightCalibrate()
{
	Serial.println("rightWallCheckTilt called.");
	float error = 0;
	float error_margin = 0.1;
	
	// Read in the sensor values.
	sensor.readSensor();
	
	if(sensor.distanceA3 < 25 and sensor.distanceA4 < 25)
	{
		error = (sensor.distanceA3 - 0.25) - (sensor.distanceA4 + 0.4);
		
		Serial.print("Right front sensor: "); Serial.print(sensor.distanceA3 - 0.25); Serial.print(", Right back sensor: "); Serial.print(sensor.distanceA4 + 0.4); Serial.print(" Error: ");Serial.println(error); 
		
		while(abs(error) > error_margin and sensor.distanceA3 < 35 and sensor.distanceA4 < 35)
		{
			// Perform adjustments if the values are different.
			// Tilt must be significant enough for sensors to detect at least 1cm difference.
			
			// If the robot is tilted left.
			if(error > 0)
			{
				Serial.println("A4, A5, Tilted left. Tilting right for correction.");
				motorShield.setSpeeds(120,-120);
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
		// Set the brakes once the correct tilt correction angle has been reached.
		motorShield.setBrakes(400, 400);
	}
}

// Check and correct tilt angle using front sensors.
void Movement::frontWallCheckTilt()
{
	Serial.println("frontWallCheckTilt called.");
	float error = 0;
	float error_margin = 0.1;
	
	sensor.readSensor();
	
	// Comparing front left and front right sensor values.
	if(sensor.distanceA0 < 35 and sensor.distanceA2 < 35)
	{
		error = (sensor.distanceA0 + 0.5) - (sensor.distanceA2);
		
		Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0 + 0.5); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2); Serial.print(" Error: ");Serial.println(error); 
		
		while(abs(error) > error_margin and (sensor.distanceA0) < 35 and sensor.distanceA2 < 35)
		{
			if(error > 0 )
			{
				Serial.println("A0, A2, Tilted left. Tilting right for correction.");
				motorShield.setSpeeds(100,-100);
				delay(100);
				motorShield.setBrakes(400, 400);
				sensor.readSensor();
				delay(100);
				error = (sensor.distanceA0 + 0.5) - (sensor.distanceA2);
			}
			
			else if(error < 0 )
			{
				Serial.println("A0, A2, Tilted right. Tilting left for correction.");
				motorShield.setSpeeds(-100,100);
				delay(100);
				motorShield.setBrakes(400, 400);
				delay(100);
				sensor.readSensor();
				error = (sensor.distanceA0 + 0.5) - (sensor.distanceA2);
			}
			
			Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0 + 0.5); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2); Serial.print(" Error: ");Serial.println(error); 
		}
		motorShield.setBrakes(400, 400);
		delay(500);
	}
}
