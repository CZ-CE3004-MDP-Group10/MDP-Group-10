// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 CORRECTION OF TILT ANGLE USING RIGHT SENSORS FILE.

// Include the movement header file.
#include "Movement.h"

// Check and correct tilt angle using right side sensors.
void Movement::rightTiltCheck()
{	
	error = 0;
	error_margin = 0.1;
	
	// Read in the sensor values.
	sensor.readSensor();
	
	// If the distance between the right side front and back sensors is greater than a specific threshold,
	// The robot is possibly facing a staircase shaped obstacle on the right and should not calibrate.
	if(abs(sensor.distanceA3 - sensor.distanceA4) > 4)
	{
		return;
	}
	
	if(sensor.distanceA3 < 20 and sensor.distanceA4 < 20)
	{
		error = (sensor.distanceA3 - 0) - (sensor.distanceA4 + 0.2);
		
		//Serial.print("Right front sensor: "); Serial.print(sensor.distanceA3 - 0); Serial.print(", Right back sensor: "); Serial.print(sensor.distanceA4 + 0); Serial.print(" Error: ");Serial.println(error); 
		
		// Perform only 10 iterations to prevent endless tilting left and right.
		// Number of iterations must be sufficient to cater for the maximum tilt error during run.
		for(i = 0; i < 30; i++)
		{
			if(abs(error) > error_margin and sensor.distanceA3 < 20 and sensor.distanceA4 < 20)
			{
				// Perform adjustments if the values are different.
				// Tilt must be significant enough for sensors to detect at least 1cm difference.
				
				// If the robot is tilted left.
				if(error > 0)
				{
					//Serial.println("A4, A5, Tilted left. Tilting right for correction.");
					motorShield.setSpeeds(80,-80);
					delay(50);
					motorShield.setBrakes(400, 400);
					delay(100);
					sensor.readSensor();
					error = (sensor.distanceA3 - 0) - (sensor.distanceA4 + 0.2);
				}
				
				// If the robot is tilted right.
				else if(error < 0)
				{
					//Serial.println("A4, A5, Tilted right. Tilting left for correction.");
					motorShield.setSpeeds(-80,80);
					delay(50);
					motorShield.setBrakes(400, 400);
					delay(100);
					sensor.readSensor();
					error = (sensor.distanceA3 - 0) - (sensor.distanceA4 + 0.2);
				}
				
				//Serial.print("Right front sensor: "); Serial.print(sensor.distanceA3 - 0); Serial.print(", Right back sensor: "); Serial.println(sensor.distanceA4 + 0); Serial.print(" Error: ");Serial.println(error);
			}
		}
		// Set the brakes once the correct tilt correction angle has been reached.
		motorShield.setBrakes(400, 400);
	}
}
