// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 SENSORS CALCULATE OBSTACLE DISTANCE FROM SENSOR IN CM FILE.

// Include the sensors header file.
#include "Sensors.h"

// Calculate the distance of obstacles from each sensor based on the analog values.
void Sensors::calculateDistance()
{
	// EQUATIONS TO CONVERT THE ANALOG VALUES INTO CENTIMETERS:

	// SENSOR 1 CALCULATION AND OFFSETS:
	distanceA0 = -9.1042 + (8155.745 / (sensorA0_avg + 22.11502));
	if(distanceA0 < 15) {distanceA0 += 2;}
	else if(distanceA0 > 15 and distanceA0 < 30) {distanceA0 -= 1;}
	else if(distanceA0 > 30 and distanceA0 < 45) {distanceA0 -= 2;}
	else if(distanceA0 > 50 and distanceA0 < 55) {distanceA0 += 1;}
	else if(distanceA0 > 65 and distanceA0 < 70) {distanceA0 += 2;}

	// SENSOR 2 CALCULATION AND OFFSETS:
	distanceA1 = 1.41294 + (4269.218 / (sensorA1_avg - 28.92149));
	if(distanceA1 > 10 and distanceA1 < 40) {distanceA1 += 1;}
	else if(distanceA1 > 55 and distanceA1 < 60) {distanceA1 += 1;}
	else if(distanceA1 > 65) {distanceA1 += 2;}

	// SENSOR 3 CALCULATION AND OFFSETS:
	distanceA2 = -3.53939 + (5891.966 / (sensorA2_avg - 11.84241));
	if(distanceA2 < 10) {distanceA2 += 1;}
	else if(distanceA2 > 15 and distanceA2 < 20) {distanceA2 += 1;}
  
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
	
	//Serial.print("Front Left Analog A0: " ); Serial.println(sensorA0_avg);
	//Serial.print("Front Middle Analog A1: " ); Serial.println(sensorA1_avg);
	//Serial.print("Front Right Analog A2: " ); Serial.println(sensorA2_avg);
	//Serial.print("Right Front Analog A3: " ); Serial.println(sensorA3_avg);
	//Serial.print("Right Back Analog A4: " ); Serial.println(sensorA4_avg);
	//Serial.print("Left Long Analog A5: " ); Serial.println(sensorA5_avg);
	
	// If the distance result is less than zero due to no obstacle being within the sensor's
	// Blind spot distance, treat it as the obstacle being 100cm away from the sensor to avoid
	// Getting a negative value distance output.
	if(distanceA0 < 0) { distanceA0 = 100; }
	if(distanceA1 < 0) { distanceA1 = 100; }
	if(distanceA2 < 0) { distanceA2 = 100; }
	if(distanceA3 < 0) { distanceA3 = 100; }
	if(distanceA4 < 0) { distanceA4 = 100; }
	if(distanceA5 < 0) { distanceA5 = 100; }
	
	//Serial.print("Front Left Distance A0: " ); Serial.println(sensorA0_avg);
	//Serial.print("Front Middle Distance A1: " ); Serial.println(sensorA1_avg);
	//Serial.print("Front Right Distance A2: " ); Serial.println(sensorA2_avg);
	//Serial.print("Right Front Distance A3: " ); Serial.println(sensorA3_avg);
	//Serial.print("Right Back Distance A4: " ); Serial.println(sensorA4_avg);
	//Serial.print("Left Long Distance A5: " ); Serial.println(sensorA5_avg);
}
