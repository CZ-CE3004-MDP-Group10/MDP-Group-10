#include "DualVNH5019MotorShield.h"

class Sensors{

public:

	// Integer variables to hold sensor analog values.
	// Voltage values range: 0V to 5V, Analog integer values range: 0 to 1023.
	int sensorA0;   // Proximity Sensor 1 on board - FRONT LEFT SENSOR.
	int sensorA1;   // Proximity Sensor 2 on board - FRONT MIDDLE SENSOR.
	int sensorA2;   // Proximity Sensor 3 on board - FRONT RIGHT SENSOR.
	int sensorA3;   // Proximity Sensor 4 on board - RIGHT SIDE FRONT SENSOR.
	int sensorA4;   // Proximity Sensor 5 on board - RIGHT SIDE BACK SENSOR.
	int sensorA5;   // Proximity Sensor 6 on board - FACING LEFT LONG RANGE SENSOR.
	int count;
	
	// Float variables to hold the voltage value of the sensor outputs.
	double sensorA0_avg;
	double sensorA1_avg;
	double sensorA2_avg;
	double sensorA3_avg;
	double sensorA4_avg;
	double sensorA5_avg;

	// Float variables to hold the converted distance based on sensor analog value.
	double distanceA0;    // FRONT LEFT SENSOR.
	double distanceA1;    // FRONT MIDDLE SENSOR.
	double distanceA2;    // FRONT RIGHT SENSOR.
	double distanceA3;    // RIGHT SIDE FRONT SENSOR.
	double distanceA4;    // RIGHT SIDE BACK SENSOR.
	double distanceA5;    // FACING LEFT LONG RANGE SENSOR.
	
	void init(void);
	void readSensor(void);
	void doOffsets(void);
	void reset(void);
	void print(void);
	
};