// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 SENSORS CODE FILE.

// Include the sensors header file.
#include "Sensors.h"

// Define sensor variables.
void Sensors::init()
{
	//count = 0;
	
	// Float variables to hold the voltage value of the sensor outputs.
	sensorA0_avg = 0.0;		// Proximity Sensor 1 on board - FRONT LEFT SENSOR.
	sensorA1_avg = 0.0;		// Proximity Sensor 2 on board - FRONT MIDDLE SENSOR.
	sensorA2_avg = 0.0;		// Proximity Sensor 3 on board - FRONT RIGHT SENSOR.
	sensorA3_avg = 0.0;		// Proximity Sensor 4 on board - RIGHT SIDE FRONT SENSOR.
	sensorA4_avg = 0.0;		// Proximity Sensor 5 on board - RIGHT SIDE BACK SENSOR.
	sensorA5_avg = 0.0;		// Proximity Sensor 6 on board - FACING LEFT LONG RANGE SENSOR.

	// Float variables to hold the converted distance based on sensor analog value.
	distanceA0 = 0.0;
	distanceA1 = 0.0;
	distanceA2 = 0.0;
	distanceA3 = 0.0;
	distanceA4 = 0.0;
	distanceA5 = 0.0;
	
	// Integer variables to hold the obstacle distance away in steps.
	obstacleA0 = 0;
	obstacleA1 = 0;
	obstacleA2 = 0;
	obstacleA3 = 0;
	obstacleA4 = 0;
	obstacleA5 = 0;
}

// Read and obtain the average of all sensor's analog values.
void Sensors::readSensor()
{
	sensorA0_avg = 0;
	sensorA1_avg = 0;
	sensorA2_avg = 0;
	sensorA3_avg = 0;
	sensorA4_avg = 0;
	sensorA5_avg = 0;
	
	// Take 20 readings from each sensor.
	for(int count = 0; count < 20; count++)
	{
		// Read the analog value of each sensor and accumulate them for averaging.
		sensorA0_list[count] = analogRead(A0);
		sensorA1_list[count] = analogRead(A1);
		sensorA2_list[count] = analogRead(A2);
		sensorA3_list[count] = analogRead(A3);
		sensorA4_list[count] = analogRead(A4);
		sensorA5_list[count] = analogRead(A5);
	}
	
	// Take only the middle 10 readings out of the 20 samples for each sensor.
	for(int i = 5; i < 15; i++)
	{
		sensorA0_avg += sensorA0_list[i];
		sensorA1_avg += sensorA1_list[i];
		sensorA2_avg += sensorA2_list[i];
		sensorA3_avg += sensorA3_list[i];
		sensorA4_avg += sensorA4_list[i];
		sensorA5_avg += sensorA5_list[i];
	}
  
	// Obtain the average of the values of each sensor.
	sensorA0_avg /= 10;
	sensorA1_avg /= 10;
	sensorA2_avg /= 10;
	sensorA3_avg /= 10;
	sensorA4_avg /= 10;
	sensorA5_avg /= 10;
	
	// Perform analog values to distance in cm conversion.
	doOffsets();
}

// Calculate the distance of obstacles from each sensor based on the analog values.
void Sensors::doOffsets()
{
	// EQUATIONS TO CONVERT THE ANALOG VALUES INTO CENTIMETERS:

	// SENSOR 1 CALCULATION AND OFFSETS:   READINGS DIFFERENT FROM EXCEL
	distanceA0 = -9.1042 + (8155.745 / (sensorA0_avg + 22.11502));
	if(distanceA0 < 15) {distanceA0 += 2;}
	else if(distanceA0 > 15 and distanceA0 < 30) {distanceA0 -= 1;}
	else if(distanceA0 > 30 and distanceA0 < 45) {distanceA0 -= 2;}
	else if(distanceA0 > 50 and distanceA0 < 55) {distanceA0 += 1;}
	else if(distanceA0 > 65 and distanceA0 < 70) {distanceA0 += 2;}

	// SENSOR 2 CALCULATION AND OFFSETS:
	distanceA1 = -3.53939 + (5891.966 / (sensorA1_avg - 11.84241));
	if(distanceA1 < 10) {distanceA1 += 1;}
	else if(distanceA1 > 15 and distanceA1 < 20) {distanceA1 += 1;}

	// SENSOR 3 CALCULATION AND OFFSETS:
	distanceA2 = 1.41294 + (4269.218 / (sensorA2_avg - 28.92149));
	if(distanceA2 > 10 and distanceA2 < 40) {distanceA2 += 1;}
	else if(distanceA2 > 55 and distanceA2 < 60) {distanceA2 += 1;}
	else if(distanceA2 > 65) {distanceA2 += 2;}
  
	// SENSOR 4 CALCULATION AND OFFSETS:
	distanceA3 = 0.252644 + (4894.633 / (sensorA3_avg - 26.90775));
	if(distanceA3 > 70) {distanceA3 -= 3;}
	else if(distanceA3 > 80) {distanceA3 -= 2;}

	// SENSOR 5 CALCULATION AND OFFSETS:
	distanceA4 = 0.404528 + (5267.347 / (sensorA4_avg - 7.79982));
	if(distanceA4 < 13) {distanceA4 -= 1;}
	else if(distanceA4 > 50 and distanceA4 < 70) {distanceA4 -= 2;}

	// SENSOR 6 CALCULATION AND OFFSETS:
	distanceA5 = -3.3012 + (12806.428 / (sensorA5_avg - 9.81909));
	if(distanceA5 < 25) {distanceA5 -= 1;}
	else if(distanceA5 > 25 and distanceA5 < 45) {distanceA5 += 10;}
	else if(distanceA5 > 75 and distanceA5 < 80) {distanceA5 -= 10;}
	
	// If the distance result is less than zero due to no obstacle being within the sensor's
	// Blind spot distance, treat it as the obstacle being 100cm away from the sensor to avoid
	// Getting a negative value distance output.
	if(distanceA0 < 0) { distanceA0 = 100; }
	if(distanceA1 < 0) { distanceA1 = 100; }
	if(distanceA2 < 0) { distanceA2 = 100; }
	if(distanceA3 < 0) { distanceA3 = 100; }
	if(distanceA4 < 0) { distanceA4 = 100; }
	if(distanceA5 < 0) { distanceA5 = 100; }
}

// Convert distance into steps of 10cm from obstacle block for short range infrared sensor.
double Sensors::calculateDist1(double dist)
{
	// IF THE OBSTACLE NEXT TO ROBOT IS TREATED AS WITHIN BLIND SPOT.
	/*
	// Within blind spot.
	if(dist < 13) {return -1;}
	
	// Too far to be detected.
	else if(dist >= 35) {return 0;}
	
	// Obstacle is 1 step away.
	else if(dist >= 13 and dist < 25) {return 1;}
	
	// Obstacle is 2 steps away.
	else if(dist >= 25 and dist < 35) {return 2;}
	*/

	// IF THE OBSTACLE NEXT TO THE ROBOT IS TREATED AS ONE STEP AWAY.
	// Obstacle is 1 step away.
	if(dist < 13) {return 1;}
	
	// Too far to be detected.
	else if(dist >= 25) {return 0;}
	
	// Obstacle is 2 steps away.
	else if(dist >= 13 and dist < 25) {return 2;}
}

// Convert distance into steps of 10cm from obstacle block for long range infrared sensor.
double Sensors::calculateDist2(double dist)
{
	// IF THE OBSTACLE NEXT TO ROBOT IS TREATED AS WITHIN BLIND SPOT.
	/*
	// Within blind spot.
	if(dist < 25) {return -1;}
	
	// Too far to be detected.
	else if(dist >= 50) {return 0;}
	
	// Obstacle is 1 step away.
	else if( dist >= 25 and dist < 35) {return 1;}
	
	// Obstacle is 2 steps away.
	else if(dist >= 35 and dist < 38) {return 2;}
	
	// Obstacle is 3 steps away.
	else if(dist >= 38 and dist < 41) {return 3;}
	
	// Obstacle is 4 steps away.
	else if(dist >= 41 and dist < 43) {return 4;}
	
	// Obstacle is 5 steps away.
	else if(dist >= 43 and dist < 50) {return 5;}
	*/
	
	// IF THE OBSTACLE NEXT TO THE ROBOT IS TREATED AS ONE STEP AWAY.
	// Obstacle is 1 step away.
	if(dist < 25) {return 1;}
	
	// Too far to be detected.
	else if(dist >= 43) {return 0;}
	
	// Obstacle is 2 steps away.
	else if( dist >= 25 and dist < 35) {return 2;}
	
	// Obstacle is 3 steps away.
	else if(dist >= 35 and dist < 38) {return 3;}
	
	// Obstacle is 4 steps away.
	else if(dist >= 38 and dist < 41) {return 4;}
	
	// Obstacle is 5 steps away.
	else if(dist >= 41 and dist < 43) {return 5;}
}

// Print and send the string over to algorithm.
void Sensors::print()
{
	// Calculate the distance of the obstacle from the sensor in steps.
	obstacleA0 = calculateDist1(distanceA0);
	obstacleA1 = calculateDist1(distanceA1);
	obstacleA2 = calculateDist1(distanceA2);
	obstacleA3 = calculateDist1(distanceA3);
	obstacleA4 = calculateDist1(distanceA4);
	obstacleA5 = calculateDist2(distanceA5);
	
	// Return the results to algorithm in terms of steps from obstacle.
	Serial.println("ALG|" + String(obstacleA5) + "," + String(obstacleA0) + "," + String(obstacleA1) + "," + String(obstacleA2) + "," + String(obstacleA3) + "," + String(obstacleA4));
	
	// Return the results to algorithm in terms of distance from obstacle.
	//Serial.println("ALG|" + String(distanceA5) + "," + String(distanceA0) + "," + String(distanceA1) + "," + String(distanceA2) + "," + String(distanceA3) + "," + String(distanceA4));
}