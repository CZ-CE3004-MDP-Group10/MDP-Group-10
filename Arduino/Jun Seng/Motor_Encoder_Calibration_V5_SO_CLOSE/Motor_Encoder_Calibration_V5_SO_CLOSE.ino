// VERSION 2 - IMPLEMENT THE PID CONTROLLER.
// Include the required libraries.
#include "DualVNH5019MotorShield.h"
#include "EnableInterrupt.h"

// Create an object for the motor shield.
DualVNH5019MotorShield motorShield;

// Motor 1 = Right.
// Motor 2 = Left.

#define encoder_M1_A 3   // Right motor output 1.
#define encoder_M1_B 5   // Right motor output 2.
#define encoder_M2_A 11  // Left motor output 1.
#define encoder_M2_B 13  // Left motor output 2.

// Motor commanded power variables, set the initial motor speed here.
// Note that right motor is slightly faster than left motor.
double right_speed = 200.0;
double left_speed = 250.0;

// Tick count variables.
int right_ticks;
int left_ticks;
double M1_ticks_moved = 0;
double M2_ticks_moved = 0;
double Total_M1_moved = 0;
double Total_M2_moved = 0;
double ticks_to_move = 245;     // Gives approximately 10cm.
double M1_setpoint_ticks = 5;   // Number of ticks before each iteration of PID controller.
double M2_setpoint_ticks = 5;   // Number of ticks before each iteration of PID controller.

// PID function variables.
double right_ticks_PID = 0.0;
double left_ticks_PID = 0.0;
double M1_ticks_PID = 0;
double M2_ticks_PID = 0;
double E1_error_ticks = 0;
double E2_error_ticks = 0;
double E1_prev_error = 0;
double E2_prev_error = 0;
double E1_sum_error = 0;
double E2_sum_error = 0;
double KP = 0.2;        // Adjust for proportional component.
double KD = 0.3;        // Adjust for derivative component.
double KI = 7;          // Adjust for integral component.

// PID control law values.
/*
double right_KP = 0.2;
double right_KD = 0.01;
double right_KI = 0.05;
double left_KP = 0.2;
double left_KD = 0.01;
double left_KI = 0.05;

double right_K1 = right_KP + right_KI + right_KD;
double right_K2 = -right_KP - (2 * right_KD);
double right_K3 = right_KD;
double left_K1 = left_KP + left_KI + left_KD;
double left_K2 = -left_KP - (2 * left_KD);
double left_K3 = left_KD;
*/

// ----------------------------------------------------------------------------------
// Setup code runs once at startup.
void setup()
{
  //Send and receive serial monitor console data at rate of 115200 baud.
  Serial.begin(115200);

  // Initialize the motor shield driver object.
  motorShield.init();

  // Set the encoder pins as input.
  pinMode(encoder_M1_A, INPUT);
  pinMode(encoder_M1_B, INPUT);
  pinMode(encoder_M2_A, INPUT);
  pinMode(encoder_M2_B, INPUT);

  // Enable interrupts for each encoder input pin 1 on the rising edge.
  // Function 'right_tick_increment" is called on rising edge of input 'encoder_M1_A'.
  enableInterrupt(encoder_M1_A, right_tick_increment, RISING);
  enableInterrupt(encoder_M2_A, left_tick_increment, RISING);

  // Set the motor initial speed if desired. - May be redundant.
  //motorShield.setM1Speed(100);    // Right motor.
  //motorShield.setM2Speed(150);     // Left motor.
  //stopIfFault();
  //delay(200);
}

// Looping code runs continuously.
void loop()
{
  // Sum up the total number of ticks per iteration of PID computation.
  M1_ticks_moved += right_ticks;
  M2_ticks_moved += left_ticks;

  // Two master counters count the total number of ticks moved through
  // the entire motor operation to check for discrepancies over time.
  
  // Once the desired number of ticks to move the required distance (10cm) has been reached.
  if(M1_ticks_moved > ticks_to_move and M2_ticks_moved > ticks_to_move)
  {
    Serial.print("R ticks moved : "); Serial.print(M1_ticks_moved);
    Serial.print(", L ticks moved : "); Serial.print(M2_ticks_moved);
    Serial.print(", Total right ticks moved : "); Serial.print(Total_M1_moved);
    Serial.print(", Total left ticks moved : "); Serial.println(Total_M2_moved);

    Total_M1_moved += M1_ticks_moved;
    Total_M2_moved += M2_ticks_moved;
    
    // Reset the tick counters.
    M1_ticks_moved = 0;
    M2_ticks_moved = 0;

    // Stop the robot movement, braking is more effective then setting the speed to 0.
    motorShield.setBrakes(400,400);
    delay(500);
  }

  // Determine the error difference in the number of ticks.
  E1_error_ticks = M1_setpoint_ticks - right_ticks;
  E2_error_ticks = M2_setpoint_ticks - left_ticks;

  // Perform PID calculation for the new motor speed.
  M1_ticks_PID = right_ticks + (E1_error_ticks * KP * 0.95) + (E1_prev_error * (KD + 0.2)) + (E1_sum_error * KI);
  M2_ticks_PID = left_ticks + (E2_error_ticks * KP) + (E2_prev_error * KD) + (E2_sum_error * KI);

  // Convert the adjusted ticks to the new motor power.
  right_speed = right_ticks_to_power(M2_ticks_PID);
  left_speed = left_ticks_to_power(M1_ticks_PID);
  
  // Set the new motor speed.
  motorShield.setM1Speed(right_speed);    // Right motor.
  motorShield.setM2Speed(left_speed);     // Left motor.
  stopIfFault();

  //Serial.print(", R power: "); Serial.print(right_speed);
  //Serial.print(", L power: "); Serial.println(left_speed);

  // Reset the tick count for the next iteration after the PID has been performed.
  right_ticks = 0;
  left_ticks = 0;

  // Store the last error value for the next computation.
  E1_prev_error = E1_error_ticks;
  E2_prev_error = E2_error_ticks;

  // Store the sum of all errors.
  // If an error is a negative value, it is subtracted from the sum.
  E1_sum_error += E1_error_ticks;
  E2_sum_error += E2_error_ticks;

  // Perform the PID computation again after a sampling time.
  delay(10);
}

// ----------------------------------------------------------------------------------
// Increment the number of ticks at the rising edge of the right motor encoder square wave.
void right_tick_increment()
{
  right_ticks++;
}

// Increment the number of ticks at the rising edge of the left motor encoder square wave.
void left_tick_increment()
{
  left_ticks++;
}

// Based on obtained data of ticks moved vs set power for right motor.
double right_ticks_to_power(double right_ticks)
{
  return (right_ticks * 0.22748) + 20.06839;
}

// Based on obtained data of ticks moved vs set power for left motor.
double left_ticks_to_power(double left_ticks)
{
  return (left_ticks * 0.221907) + 31.70289;
}

// ----------------------------------------------------------------------------------
// Stops the motor if there is a fault, like a short circuit.
// Infinite loop stops the program from continuing.
void stopIfFault()
{
  if (motorShield.getM1Fault())
  {
    Serial.println("Left motor fault.");
    while(1);
  }
  if (motorShield.getM2Fault())
  {
    Serial.println("Right motor fault.");
    while(1);
  }
}
