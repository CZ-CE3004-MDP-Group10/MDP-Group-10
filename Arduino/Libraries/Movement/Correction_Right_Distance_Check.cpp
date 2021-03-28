// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 CORRECTION OF DISTANCE FROM OBSTACLE ON RIGHT FILE.

// Include the movement header file.
#include "Movement.h"

// Check if the robot is the correct distance from the right side wall.
void Movement::rightDistanceCheck()
{
	// Align the robot with the right wall.
	rightTiltCheck();
	sensor.readSensor();
	
	// This limit value determines if the robot should move closer towards the right side wall,
	// When it is more than one step out of position away from the wall.
	limit = 24;
		
	// Rotate the robot right.
	distsub = 1;
	
	// Need to fix issue with right rotation during right wall calibration falling short.
	calibrateRightRotate = true;
	rotate90right();
	calibrateRightRotate = false;
	delay(50);
			
	// Calibrate by front of the robot, which would now be facing right.
	frontDistanceCheck();
	delay(50);
			
	// Rotate the robot left.
	distsub = 1;
	rotate90left();
	delay(50);
			
	// Perform right side calibration.
	rightTiltCheck();
		
	// Reset the limit value so that if the robot is called to calibrate by the front, and there is supposed
	// To be one empty step space ahead of it, it will not move forward into that empty space and misalign
	// Itself with the map on the algorithm and the android.
	limit = 14;
}
