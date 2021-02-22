// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 SENSORS CODE FILE.

// Include the sensors header file.
#include "Sensors.h"

// Define sensor variables.
void Sensors::init()
{
	sensorA0 = 0;   // Proximity Sensor 1 on board - FRONT LEFT SENSOR.
	sensorA1 = 0;   // Proximity Sensor 2 on board - FRONT MIDDLE SENSOR.
	sensorA2 = 0;   // Proximity Sensor 3 on board - FRONT RIGHT SENSOR.
	sensorA3 = 0;   // Proximity Sensor 4 on board - RIGHT SIDE FRONT SENSOR.
	sensorA4 = 0;   // Proximity Sensor 5 on board - RIGHT SIDE BACK SENSOR.
	sensorA5 = 0;   // Proximity Sensor 6 on board - FACING LEFT LONG RANGE SENSOR.
	count = 0;
	
	// Float variables to hold the voltage value of the sensor outputs.
	sensorA0_avg = 0.0;
	sensorA1_avg = 0.0;
	sensorA2_avg = 0.0;
	sensorA3_avg = 0.0;
	sensorA4_avg = 0.0;
	sensorA5_avg = 0.0;

	// Float variables to hold the converted distance based on sensor analog value.
	distanceA0 = 0.0;    // FRONT LEFT SENSOR.
	distanceA1 = 0.0;    // FRONT MIDDLE SENSOR.
	distanceA2 = 0.0;    // FRONT RIGHT SENSOR.
	distanceA3 = 0.0;    // RIGHT SIDE FRONT SENSOR.
	distanceA4 = 0.0;    // RIGHT SIDE BACK SENSOR.
	distanceA5 = 0.0;    // FACING LEFT LONG RANGE SENSOR.
}

// Read and obtain the average of all sensor's analog values.
void Sensors::readSensor()
{
	for(count = 0; count < 20; count++)
	{
		// Read the analog value of each sensor and accumulate them for averaging.
		sensorA0 = analogRead(A0);
		sensorA0_avg += sensorA0;
		sensorA1 = analogRead(A1);
		sensorA1_avg += sensorA1;
		sensorA2 = analogRead(A2);
		sensorA2_avg += sensorA2;
		sensorA3 = analogRead(A3);
		sensorA3_avg += sensorA3;
		sensorA4 = analogRead(A4);
		sensorA4_avg += sensorA4;
		sensorA5 = analogRead(A5);
		sensorA5_avg += sensorA5;
	}
  
	// Obtain the average of the values of each sensor.
	sensorA0_avg /= 20;
	sensorA1_avg /= 20;
	sensorA2_avg /= 20;
	sensorA3_avg /= 20;
	sensorA4_avg /= 20;
	sensorA5_avg /= 20;
	
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
	else if(distanceA5 > 25 and distanceA5 < 45) {distanceA5 += 1;}
	else if(distanceA5 > 75 and distanceA5 < 80) {distanceA5 -= 1;}

	// If the distance result is less than zero due to no obstacle being within the sensor's
	// Blind spot distance, treat it as the obstacle being 100cm away from the sensor to avoid
	// Getting a negative value distance output.
	if(distanceA0 < 0) { distanceA0 = 100; }
	if(distanceA1 < 0) { distanceA1 = 100; }
	if(distanceA2 < 0) { distanceA2 = 100; }
	if(distanceA3 < 0) { distanceA3 = 100; }
	if(distanceA4 < 0) { distanceA4 = 100; }
	if(distanceA5 < 0) { distanceA5 = 100; }

	print();	
	reset();
}

// Reset the sensor values to their defaults.
void Sensors::reset()
{
	// Reset the counter and variables.
	count = 0;
	sensorA0_avg = 0;
	sensorA1_avg = 0;
	sensorA2_avg = 0;
	sensorA3_avg = 0;
	sensorA4_avg = 0;
	sensorA5_avg = 0;
}

// Print and send the string over to algorithm.
void Sensors::print()
{
	Serial.println("ALG|" + String(distanceA0) + "," + String(distanceA1) + "," + String(distanceA2)
      + "," + String(distanceA3) + "," + String(distanceA4) + "," + String(distanceA5));
}