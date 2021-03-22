// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 CORRECTION CODE FILE.

// Include the movement header file.
#include "Movement.h"

// Check and correct distance from front using front sensors.
void Movement::frontDistanceCheck()
{
	error = 0;
	error_margin = 0.1;
	perfDist = 8;
	
	// Read in the sensor values.
	sensor.readSensor();
	
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
				error = (sensor.distanceA0 + sensor.distanceA2) / 2 - perfDist;
			}
		}
		// Stop the robot once it is at approxiately the right distance from the front of the wall.
		motorShield.setBrakes(400, 400);
		delay(500);
	}
}

// ---------------------------------------------------------------------------------------------
// Check if the robot is the correct distance from the right side wall.
void Movement::rightDistanceCheck()
{
	// Align the robot with the right wall.
	rightTiltCheck();
	sensor.readSensor();
	
	// This limit value determines if the robot should move closer towards the right side wall,
	// When it is more than one step out of position away from the wall.
	limit = 24;
		
	// Rotate the robot right.
	distsub = 1;
	
	// Need to fix issue with right rotation during right wall calibration falling short.
	calibrateRightRotate = true;
	rotate90right();
	calibrateRightRotate = false;
	delay(50);
			
	// Calibrate by front of the robot, which would now be facing right.
	frontDistanceCheck();
	delay(50);
			
	// Rotate the robot left.
	distsub = 1;
	rotate90left();
	delay(50);
			
	// Perform right side calibration.
	rightTiltCheck();
		
	// Reset the limit value so that if the robot is called to calibrate by the front, and there is supposed
	// To be one empty step space ahead of it, it will not move forward into that empty space and misalign
	// Itself with the map on the algorithm and the android.
	limit = 14;
}

// ---------------------------------------------------------------------------------------------
// Check and correct tilt angle using front sensors.
void Movement::frontTiltCheck()
{
	//Serial.println("frontWallCheckTilt called.");
	error = 0;
	error_margin = 0.1;
	
	sensor.readSensor();
	
	// If there is at least 10cm difference between the sensors,
	// The robot may be facing a staircase obstacle and should not calibrate.
	if(abs(sensor.distanceA0 - sensor.distanceA2) > 10)
	{
		return;
	}
	
	// Comparing front left and front right sensor values.
	if(sensor.distanceA0 < 15 and sensor.distanceA2 < 15)
	{
		error = (sensor.distanceA0 + 1) - (sensor.distanceA2 - 0);
		
		Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0 + 0.4); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2 - 0.4); Serial.print(" Error: ");Serial.println(error); 
		
		for(i = 0; i < 12; i++)
		{
			if(abs(error) > error_margin and (sensor.distanceA0) < 20 and sensor.distanceA2 < 20)
			{
				if(error > 0)
				{
					Serial.println("A0, A2, Tilted left. Tilting right for correction.");
					motorShield.setSpeeds(100,-100);
					delay(50);
					motorShield.setBrakes(400, 400);
					delay(100);
					
					sensor.readSensor();
					error = (sensor.distanceA0 + 1) - (sensor.distanceA2 - 0);
				}
				
				else if(error < 0)
				{
					Serial.println("A0, A2, Tilted right. Tilting left for correction.");
					motorShield.setSpeeds(-100,100);
					delay(50);
					motorShield.setBrakes(400, 400);
					delay(100);
					
					sensor.readSensor();
					error = (sensor.distanceA0 + 1) - (sensor.distanceA2 - 0);
				}
				
				Serial.print("Left Front sensor: "); Serial.print(sensor.distanceA0 + 0.4); Serial.print(", Right Front sensor: "); Serial.print(sensor.distanceA2 - 0.4); Serial.print(" Error: ");Serial.println(error); 
			}
		}
		motorShield.setBrakes(400, 400);
		delay(500);
	}
}

// ---------------------------------------------------------------------------------------------
// Check and correct tilt angle using right side sensors.
void Movement::rightTiltCheck()
{	
	error = 0;
	error_margin = 0.1;
	
	// Read in the sensor values.
	sensor.readSensor();
	
	// If there is at least 10cm difference between the sensors,
	// The robot may be facing a staircase obstacle and should not calibrate.
	if(abs(sensor.distanceA3 - sensor.distanceA4) > 10)
	{
		return;
	}
	
	if(sensor.distanceA3 < 20 and sensor.distanceA4 < 20)
	{
		error = (sensor.distanceA3 - 0.1) - (sensor.distanceA4 + 0.4);
		
		Serial.print("Right front sensor: "); Serial.print(sensor.distanceA3 - 0.1); Serial.print(", Right back sensor: "); Serial.print(sensor.distanceA4 + 0.4); Serial.print(" Error: ");Serial.println(error); 
		
		// Perform only 10 iterations to prevent endless tilting left and right.
		// Number of iterations must be sufficient to cater for the maximum tilt error during run.
		for(i = 0; i < 12; i++)
		{
			if(abs(error) > error_margin and sensor.distanceA3 < 20 and sensor.distanceA4 < 20)
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
					error = (sensor.distanceA3 - 0.1) - (sensor.distanceA4 + 0.1);
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
					error = (sensor.distanceA3 - 0.1) - (sensor.distanceA4 + 0.4);
				}
				
				Serial.print("Right front sensor: "); Serial.print(sensor.distanceA3 - 0.1); Serial.print(", Right back sensor: "); Serial.println(sensor.distanceA4 + 0.4); Serial.print(" Error: ");Serial.println(error);
			}
		}
		// Set the brakes once the correct tilt correction angle has been reached.
		motorShield.setBrakes(400, 400);
	}
}
