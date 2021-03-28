// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 MOVEMENT STOP IF FORWARD DISTANCE HAS BEEN REACHED FILE.

// Include the movement header file.
#include "Movement.h"

// Stop the robot after it has moved the required straight distance.
void Movement::stopIfReached()
{
	// Once the desired number of ticks to move the required distance has been reached.
	if(pid.M1_ticks_moved > pid.M1_ticks_to_move and pid.M2_ticks_moved > pid.M2_ticks_to_move)
	{
		// Robot movement variables are held in the PID controller object created during initialization.
		
		//Serial.print("Ticks to move M1: "); Serial.print(pid.M1_ticks_to_move);
		//Serial.print(", Ticks moved M1: "); Serial.println(pid.M1_ticks_moved);
		//Serial.print(", Ticks to move M2: "); Serial.print(pid.M2_ticks_to_move);
		//Serial.print(", Ticks moved M2: "); Serial.println(pid.M2_ticks_moved);

		// Error difference for how many ticks the current step exceeded by.
		//pid.M1_ticks_diff = pid.M1_ticks_moved - pid.M1_ticks_to_move;
		//pid.M2_ticks_diff = pid.M2_ticks_moved - pid.M2_ticks_to_move;
		
		// Count the total number of ticks over a period of time.
		//Total_M1_moved += M1_ticks_moved;
		//Total_M2_moved += M2_ticks_moved;
		  
		//Serial.print(", Left ticks moved : "); Serial.print(pid.M1_ticks_moved);
		//Serial.print(", Right ticks moved : "); Serial.println(pid.M2_ticks_moved);
		//Serial.print(", Total left ticks moved : "); Serial.print(pid.Total_M1_moved);
		//Serial.print(", Total right ticks moved : "); Serial.println(pid.Total_M2_moved);
		
// ########################################################
		// FOR FASTEST PATH, UNCOMMENT THIS PORTION.
		// FOR EXPLORATION, COMMENT THIS PORTION.
		
		/*sensor.readSensor();
		
		// If the robot moves too close to an obstacle in front when moving forwards,
		// Set the 'distsub' to zero, apply the brakes and perform subsequent corrections.
		if(sensor.distanceA0 < 10 or sensor.distanceA1 < 10 or sensor.distanceA2 < 10)
		{
			pid.M1_ticks_moved = pid.M1_ticks_to_move;
			pid.M2_ticks_moved = pid.M2_ticks_to_move;
			distsub = 0;
		}*/
// ########################################################

		// Decrement the number of steps left to travel.
		distsub--;

		// The robot should only apply the brakes when it has finished the last step, 
		// Before stopping and waiting for a command.
		if(distsub <= 0)
		{
		  // Set the brakes on both motors to bring the robot to a stop.
		  motorShield.setM1Brake(400);
		  
		  // Delay is added here intentionally to make the left motor brake before the right motor,
		  // Due to observation that the robot tended to shift to the right after moving straight.
		  delay(3);
		  motorShield.setM2Brake(400);
		  //motorShield.setBrakes(400, 400);
		}
		// Reset the tick counters.
		pid.M1_ticks_moved = 0;
		pid.M2_ticks_moved = 0;
	}
}
