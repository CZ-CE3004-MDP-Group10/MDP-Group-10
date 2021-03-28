// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 SENSORS READ ANALOG VALUES FROM SENSORS FILE.

// Include the sensors header file.
#include "Sensors.h"

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
	
	// Sort the sensor analog readings from lowest to highest.
	sortReadings(sensorA0_list);
	sortReadings(sensorA1_list);
	sortReadings(sensorA2_list);
	sortReadings(sensorA3_list);
	sortReadings(sensorA4_list);
	sortReadings(sensorA5_list);
	
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
	calculateDistance();
}
