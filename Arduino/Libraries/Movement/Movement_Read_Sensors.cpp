// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT CALL SENSOR OBJECT TO READ SENSOR VALUES FILE.

// Include the movement header file.
#include "Movement.h"

// Read the sensor distance values when the robot stops moving.
void Movement::readSensor()
{
	sensor.readSensor();
}
