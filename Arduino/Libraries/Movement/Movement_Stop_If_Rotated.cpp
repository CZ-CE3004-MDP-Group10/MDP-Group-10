// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT STOP IF ROTATION ANGLE HAS BEEN REACHED FILE.

// Include the movement header file.
#include "Movement.h"

// Stop the robot after it has rotated by the required angle.
void Movement::stopIfRotated()
{
	// Once the number of ticks moved by both wheels exceed the ticks to be moved for each wheel.
	if(pid.M1_ticks_moved > pid.M1_ticks_to_move and pid.M2_ticks_moved > pid.M2_ticks_to_move)
	{
		//Serial.print("Ticks to move M1: "); Serial.print(pid.M1_ticks_to_move);
		//Serial.print(", Ticks moved M1: "); Serial.println(pid.M1_ticks_moved);
		//Serial.print(", Ticks to move M2: "); Serial.print(pid.M2_ticks_to_move);
		//Serial.print(", Ticks moved M2: "); Serial.println(pid.M2_ticks_moved);

		// NOTE: ERROR IN TICKS DIFFERENCE IS NOT NEEDED HERE AS EACH ROTATION IS INDEPENDENT.
		  
		// Count the total number of ticks over a period of time.
		//Total_M1_moved += M1_ticks_moved;
		//Total_M2_moved += M2_ticks_moved;
		  
		//Serial.print(", Left ticks moved : "); Serial.print(pid.M1_ticks_moved);
		//Serial.print(", Right ticks moved : "); Serial.print(pid.M2_ticks_moved);
		//Serial.print(", Total left ticks moved : "); Serial.print(pid.Total_M1_moved);
		//Serial.print(", Total right ticks moved : "); Serial.println(pid.Total_M2_moved);

		// Decrement the number of steps left to travel.
		distsub--;

		if(distsub <= 0)
		{
		  motorShield.setBrakes(400, 400);
		}
		// Reset the tick counters.
		pid.M1_ticks_moved = 0;
		pid.M2_ticks_moved = 0;
	}
}
