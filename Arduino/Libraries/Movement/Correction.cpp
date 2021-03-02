// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 CORRECTION CODE FILE.

// Include the movement header file.
#include "Movement.h"

// Move forwards a little if the robot is too far from the stopping point in front.
void Movement::forwardsLittle()
{
	pid.setZero();	
	Serial.println("Moving forwards for correction.");
	pid.M1_ticks_diff = 0;
	pid.M2_ticks_diff = 0;
	
	// Using a much smaller ticks value.
	pid.M1_ticks_to_move = 1; //OK
	pid.M2_ticks_to_move = 1; //OK
	
	straightTransition = false;
	rotateTransition = true;
	distsub = 1;
	
	pid.control(1,1);
	motorShield.setSpeeds(pid.getRightSpeed() * 0.5,pid.getLeftSpeed() * 0.5);
		
	//stopIfFault();
	delay(10);
	stopIfReached();
}

// Move backwards a little if the robot is too close to an obstacle in front.
void Movement::backwards()
{
	pid.setZero();
	Serial.println("Moving backwards for correction.");
	pid.M1_ticks_diff = 0;
	pid.M2_ticks_diff = 0;
	
	pid.M1_ticks_to_move = 1; //OK
	pid.M2_ticks_to_move = 1; //OK
	
	straightTransition = false;
	rotateTransition = true;
	distsub = 1;
	
	pid.control(-1,-1);
	motorShield.setSpeeds(pid.getRightSpeed() * 0.5,pid.getLeftSpeed() * 0.5);
		
	//stopIfFault();
	delay(10);
	stopIfReached();
}

// Rotate 3 degrees to the left if the robot is detected to be tilted right.
void Movement::rotate3left()
{
	pid.setZero();
	Serial.println("Tilting left for correction.");
	pid.M1_ticks_to_move = 1; //TO ADJUST
	pid.M2_ticks_to_move = 1; //TO ADJUST

	rotateTransition = false;
	straightTransition = true;
	distsub = 1;
	
	while(distsub > 0)
	{
		// Left motor is negative and right motor is positive.
		pid.control(-1,1);
		motorShield.setSpeeds(pid.getRightSpeed() * 0.5,pid.getLeftSpeed() * 0.5);
		stopIfFault();
		delay(5);
		stopIfRotated();
	}
}

// Rotate 3 degrees to the right if the robot is detected to be tilted left.
void Movement::rotate3right()
{
	pid.setZero();
	Serial.println("Tilting right for correction.");
	pid.M1_ticks_to_move = 1; //TO ADJUST
	pid.M2_ticks_to_move = 1; //TO ADJUST

	rotateTransition = false;
	straightTransition = true;
	distsub = 1;
	
	while(distsub > 0)
	{
		// Left motor is positive and right motor is negative.
		pid.control(1,-1);
		motorShield.setSpeeds(pid.getRightSpeed() * 0.5,pid.getLeftSpeed() * 0.5);
		stopIfFault();
		delay(5);
		stopIfRotated();
	}
}

void Movement::calibrate()
{
	Serial.println("Right side calibration called.");
	rightWallCheckTilt();
	frontObstacleCheck();
	//rightWallCheckTilt();
}

// Check if the robot is too close to the front, perform corrections if it is.
void Movement::frontObstacleCheck()
{
	Serial.println("Front calibration called.");
	frontWallCheckTilt();
	//frontDistanceCheck();
	//frontWallCheckTilt();
}

void Movement::frontDistanceCheck()
{
	Serial.println("Proceeding to calibrate the distance from front.");
	float error = 0;
	float error_margin = 0.1;
	float perfDist = 11;
	
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
			
			// Compare the front left and front right sensors.
			if(error < 0)
			{
				Serial.println("Too close to front.");
				
				// Wait a short while between the two movements.
				delay(100);
				backwards();
			}
			else if(error > 0)
			{
				Serial.println("Too far from front.");
				delay(100);
				forwardsLittle();
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
				Serial.println("Too close to front.");
				
				// Wait a short while between the two movements.
				delay(100);
				backwards();
			}
			else if(error > 0)
			{
				Serial.println("Too far from front.");
				delay(100);
				forwardsLittle();
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
				Serial.println("Too close to front.");
				
				// Wait a short while between the two movements.
				delay(100);
				backwards();
			}
			else if(error > 0)
			{
				Serial.println("Too far from front.");
				delay(100);
				forwardsLittle();
			}
			
			// Read in the sensor values.
			sensor.readSensor();
			error = (sensor.distanceA1 + sensor.distanceA2) / 2 - perfDist;
		}
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
				Serial.println("Tilted left.");
				delay(100);
				rotate3right();
			}
			
			// If the robot is tilted right.
			else if(error < 0 )
			{
				Serial.println("Tilted right.");
				delay(100);
				rotate3left();
			}
			
			// If no tilt is detected.
			else
			{
				Serial.println("No tilt detected.");
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
				Serial.println("Tilted left.");
				delay(100);
				rotate3right();
			}
			
			// If the robot is tilted right.
			else if(error < 0 )
			{
				Serial.println("Tilted right.");
				delay(100);
				rotate3left();
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
				Serial.println("Tilted left.");
				delay(100);
				rotate3right();
			}
			
			// If the robot is tilted right.
			else if(error < 0 )
			{
				Serial.println("Tilted right.");
				delay(100);
				rotate3left();
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

// Perform right wall hugging.
// Check if the analog values of right mounted sensors are equal, perform corrections if not.
void Movement::rightWallCheckTilt()
{
	float error = 0;
	float error_margin = 0.2;
	
	// Read in the sensor values.
	sensor.readSensor();
	error = sensor.distanceA3 - (sensor.distanceA4 - 0.8);
	
	Serial.print("Right front sensor: "); Serial.print(sensor.distanceA3); Serial.print(", Right back sensor: "); Serial.print(sensor.distanceA4); Serial.print(" Error: ");Serial.println(error); 
	
	while(abs(error) > error_margin and sensor.distanceA3 < 35 and (sensor.distanceA4) < 35)
	{
		// Perform adjustments if the values are different.
		// Tilt must be significant enough for sensors to detect at least 1cm difference.
		
		// If the robot is tilted left.
		if(error > 0)
		{
			Serial.println("Tilted left.");
			delay(100);
			rotate3right();
		}
		
		// If the robot is tilted right.
		else if(error < 0)
		{
			Serial.println("Tilted right.");
			delay(100);
			rotate3left();
		}
		
		// If no tilt is detected.
		else
		{
			Serial.println("No tilt detected.");
		}
		
		// Read in the sensor values.
		sensor.readSensor();
		error = sensor.distanceA3 - (sensor.distanceA4 - 0.8);
		
		Serial.print("Right front sensor: "); Serial.print(sensor.distanceA3); Serial.print(", Right back sensor: "); Serial.println(sensor.distanceA4);
	}
	delay(250);
}