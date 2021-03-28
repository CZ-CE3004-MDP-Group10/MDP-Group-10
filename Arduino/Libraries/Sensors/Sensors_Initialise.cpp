// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 SENSORS INITIALISATION FILE.

// Include the sensors header file.
#include "Sensors.h"

// Define sensor variables.
void Sensors::init()
{
	temp = 0;
	
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
	
	// Boolean to determine if robot should break out of stuck loop.
	exitStuckLoop = false;
}
