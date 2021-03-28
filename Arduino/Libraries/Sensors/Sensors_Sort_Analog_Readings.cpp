// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 SENSORS SORTING OF SENSOR ANALOG READINGS FILE.

// Include the sensors header file.
#include "Sensors.h"

// For each sensor's array of 20 captured readings, first arrange them from highest to lowest.
void Sensors::sortReadings(double *list)
{
	for(i = 0; i < 19; i++)
	{
		for(j = 0; j < 20 - i - 1; j++)
		{
			if(list[j] > list[j+1]) 
			{
				temp = list[i];
				list[i] = list[i + 1];
				list[i + 1] = temp;
			}
		}
	}
}
