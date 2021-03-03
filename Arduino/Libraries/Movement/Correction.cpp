// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 CORRECTION CODE FILE.

// Include the movement header file.
#include "Movement.h"

// Calibrate the robot using the right side sensors, followed by the front sensors.
void Movement::calibrate()
{
	Serial.println("Right side calibration called.");
	rightWallCheckTilt();
	frontObstacleCheck();
	//rightWallCheckTilt();
}

// Calibrate the robot using only the front sensors.
void Movement::frontObstacleCheck()
{
	Serial.println("Front calibration called.");
	frontWallCheckTilt();
	//frontDistanceCheck();
	frontWallCheckTilt();
}

void Movement::frontDistanceCheck()
{
	Serial.println("Proceeding to calibrate the distance from front.");
	float error = 0;
	float error_margin = 0.1;
	float perfDist = 12;
	
	Serial.println("Front distance check function called.");
	
	// Read in the sensor values.
	sensor.readSensor();
	
	// Comparing front left and front right sensor values.
	if(sensor.distanceA0 < 25 and sensor.distanceA2 < 25)
	{
		error = (sensor.distanceA0 + sensor.distanceA2) / 2 - perfDist;
		
		Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2); Serial.print(" Error: ");Serial.println(error); 
		
		while(abs(error) > error_margin and sensor.distanceA0 < 25 and sensor.distanceA2 < 25)
		{
			Serial.println("Comparing front left and front right.");
			
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
			// Once the robot front has reached the correct distance from the obstacle.
			else
			{
				Serial.println("Acceptable distance from front.");
				motorShield.setBrakes(400, 400);
			}
			
			// Read in the sensor values.
			sensor.readSensor();
			error = (sensor.distanceA0 + sensor.distanceA2) / 2 - perfDist;
		}
	}
	
	// Comparing front left and front middle sensors.
	else if(sensor.distanceA0 < 25 and sensor.distanceA1 < 25)
	{
		error = (sensor.distanceA0 + sensor.distanceA1) / 2 - perfDist;
		
		Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0); Serial.print(", Middle Front sensor: "); Serial.print(sensor.distanceA1); Serial.print(" Error: ");Serial.println(error); 
		
		while(abs(error) > error_margin and sensor.distanceA0 < 25 and sensor.distanceA1 < 25)
		{
			Serial.println("Comparing front left and front middle.");
			
			// Compare the front left and front right sensors.
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
			else
			{
				Serial.println("Acceptable distance from front.");
				motorShield.setBrakes(400, 400);
			}
			
			// Read in the sensor values.
			sensor.readSensor();
			error = (sensor.distanceA0 + sensor.distanceA1) / 2 - perfDist;
		}
	}
	
	// Comparing front middle and front right sensors.
	else if(sensor.distanceA1 < 25 and sensor.distanceA2 < 25)
	{
		Serial.println("Comparing front middle and front right.");
		
		error = (sensor.distanceA1 + sensor.distanceA2) / 2 - perfDist;
		
		Serial.print("Middle Front sensor: "); Serial.print(sensor.distanceA1); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2); Serial.print(" Error: ");Serial.println(error); 
		
		while(abs(error) > error_margin and sensor.distanceA1 < 25 and sensor.distanceA2 < 25)
		{
			// Compare the front left and front right sensors.
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
			else
			{
				Serial.println("Acceptable distance from front.");
				motorShield.setBrakes(400, 400);
			}
			
			// Read in the sensor values.
			sensor.readSensor();
			error = (sensor.distanceA1 + sensor.distanceA2) / 2 - perfDist;
		}
	}
}

// Right wall hugging - Check if the right mounted sensor values are equal, perform corrections if not.
void Movement::rightWallCheckTilt()
{
	float error = 0;
	float error_margin = 0.1;
	
	// Read in the sensor values.
	sensor.readSensor();
	error = sensor.distanceA3 - sensor.distanceA4;
	
	Serial.print("Right front sensor: "); Serial.print(sensor.distanceA3); Serial.print(", Right back sensor: "); Serial.print(sensor.distanceA4); Serial.print(" Error: ");Serial.println(error); 
	
	while(abs(error) > error_margin and sensor.distanceA3 < 35 and sensor.distanceA4 < 35)
	{
		// Perform adjustments if the values are different.
		// Tilt must be significant enough for sensors to detect at least 1cm difference.
		
		// If the robot is tilted left.
		if(error > 0)
		{
			Serial.println("Tilted left. Tilting right for correction.");
			motorShield.setSpeeds(100,-100);
		}
		
		// If the robot is tilted right.
		else if(error < 0)
		{
			Serial.println("Tilted right. Tilting left for correction.");
			motorShield.setSpeeds(-100,100);
		}
		
		// If no tilt is detected.
		else
		{
			Serial.println("No tilt detected.");
			motorShield.setBrakes(400, 400);
			
		}
		
		// Read in the sensor values.
		sensor.readSensor();
		error = sensor.distanceA3 - sensor.distanceA4;
		
		Serial.print("Right front sensor: "); Serial.print(sensor.distanceA3); Serial.print(", Right back sensor: "); Serial.println(sensor.distanceA4);
	}
}

void Movement::frontWallCheckTilt()
{
	Serial.println("frontWallCheckTilt called.");
	float error = 0;
	float error_margin = 0.2;
	
	// Read in the sensor values.
	sensor.readSensor();
	
	// Comparing front left and front right sensor values.
	if(sensor.distanceA0 < 35 and sensor.distanceA2 < 35)
	{
		error = sensor.distanceA0 - sensor.distanceA2;
		
		Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2); Serial.print(" Error: ");Serial.println(error); 
		
		while(abs(error) > error_margin and sensor.distanceA0  < 35 and sensor.distanceA2 < 35)
		{
			if(error > 0 )
			{
				Serial.println("Tilted left. Tilting right for correction.");
				motorShield.setSpeeds(100,-100);
			}
			
			// If the robot is tilted right.
			else if(error < 0 )
			{
				Serial.println("Tilted right. Tilting left for correction.");
				motorShield.setSpeeds(-100,100);
			}
			
			// If no tilt is detected.
			else
			{
				Serial.println("No tilt detected.");
				motorShield.setBrakes(400, 400);
			}
			
			// Read in the sensor values.
			sensor.readSensor();
			error = sensor.distanceA0 - sensor.distanceA2;
		}
	}
	// Comparing front left and front middle sensors.
	else if(sensor.distanceA0 < 35 and sensor.distanceA1 < 35)
	{
		error = sensor.distanceA0 - sensor.distanceA1;
		
		Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0); Serial.print(", Middle Front sensor: "); Serial.print(sensor.distanceA1); Serial.print(" Error: ");Serial.println(error); 
		
		while(abs(error) > error_margin and sensor.distanceA0  < 35 and sensor.distanceA1 < 35)
		{
			if(error > 0 )
			{
				Serial.println("Tilted left. Tilting right for correction.");
				motorShield.setSpeeds(100,-100);
			}
			
			// If the robot is tilted right.
			else if(error < 0 )
			{
				Serial.println("Tilted right. Tilting left for correction.");
				motorShield.setSpeeds(-100,100);
			}
			
			// If no tilt is detected.
			else
			{
				Serial.println("No tilt detected.");
			}
			
			// Read in the sensor values.
			sensor.readSensor();
			error = sensor.distanceA0 - sensor.distanceA1;
		}
	}
	// Comparing front middle and front right sensors.
	else if(sensor.distanceA1 < 35 and sensor.distanceA2 < 35)
	{
		error = sensor.distanceA1 - sensor.distanceA2;
		
		Serial.print("Middle Front sensor: "); Serial.print(sensor.distanceA1); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2); Serial.print(" Error: ");Serial.println(error); 
		
		while(abs(error) > error_margin and sensor.distanceA1 < 35 and sensor.distanceA2 < 35)
		{
			if(error > 0 )
			{
				Serial.println("Tilted left. Tilting right for correction.");
				motorShield.setSpeeds(100,-100);
			}
			
			// If the robot is tilted right.
			else if(error < 0 )
			{
				Serial.println("Tilted right. Tilting left for correction.");
				motorShield.setSpeeds(-100,100);
			}
			
			// If no tilt is detected.
			else
			{
				Serial.println("No tilt detected.");
			}
			
			// Read in the sensor values.
			sensor.readSensor();
			error = sensor.distanceA1 - sensor.distanceA2;
		}
	}
}
