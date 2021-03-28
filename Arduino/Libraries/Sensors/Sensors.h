// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 SENSORS HEADER FILE.

// Include Arduino library for printing debugging statements and reading of analog pin values.
#include <Arduino.h>

class Sensors
{
	public:
	
	// Stores the sensor analog values.
	// Voltage values range: 0V to 5V, Analog integer values range: 0 to 1023.
	double sensorA0_avg;	// Proximity Sensor 1 on board - FRONT LEFT SENSOR.
	double sensorA1_avg;	// Proximity Sensor 2 on board - FRONT MIDDLE SENSOR.
	double sensorA2_avg;	// Proximity Sensor 3 on board - FRONT RIGHT SENSOR.
	double sensorA3_avg;	// Proximity Sensor 4 on board - RIGHT SIDE FRONT SENSOR.
	double sensorA4_avg;	// Proximity Sensor 5 on board - RIGHT SIDE BACK SENSOR.
	double sensorA5_avg;	// Proximity Sensor 6 on board - FACING LEFT LONG RANGE SENSOR.

	// Arrays to hold a collection of 20 sensor analog values captured at a time.
	double sensorA0_list[20];
	double sensorA1_list[20];
	double sensorA2_list[20];
	double sensorA3_list[20];
	double sensorA4_list[20];
	double sensorA5_list[20];
	int temp;
	int i;
	int j;

	// Stores the converted distance based on sensor analog value.
	double distanceA0;
	double distanceA1;
	double distanceA2;
	double distanceA3;
	double distanceA4;
	double distanceA5;
	
	// Integer variables to hold the obstacle distance away in steps.
	int obstacleA0 = 0;
	int obstacleA1 = 0;
	int obstacleA2 = 0;
	int obstacleA3 = 0;
	int obstacleA4 = 0;
	int obstacleA5 = 0;
	
	// Boolean to determine if robot should break out of stuck loop.
	boolean exitStuckLoop;
	
	// Functions for sensors.
	void init(void);
	void readSensor(void);
	void sortReadings(double *list);
	void calculateDistance(void);
	void print(void);
	double calculateStepFront(double dist);
	double calculateStepRight(double dist);
	double calculateStepLong(double dist);
};