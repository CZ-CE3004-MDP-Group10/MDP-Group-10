// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 ARDUINO FILE.

// LIBRARIES.***********************************************************************************************
#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include <SharpIR.h>

// DEFINITIONS.*********************************************************************************************

// Motor M1 = Encoder E1 = Right.
// Motor M2 = Encoder E2 = Left.

// Motor encoder inputs
#define encoder_M1_A 3   // Right motor output 1.
#define encoder_M1_B 5   // Right motor output 2. Not used.
#define encoder_M2_A 11  // Left motor output 1.
#define encoder_M2_B 13  // Left motor output 2. Not used.

// Inrared sensor input pins.
#define PS1_pin A0
#define PS2_pin A1
#define PS3_pin A2
#define PS4_pin A3
#define PS5_pin A4
#define PS6_pin A5

// Infrared sensor models.
#define large_model 20150
#define small_model 1080

// VARIABLES.***********************************************************************************************

// Motor commanded power, set the initial motor speed here.
// Note that right motor is slightly faster than left motor by default.
double right_speed = 200.0;
double left_speed = 250.0;

// Tick counters.
int right_ticks;
int left_ticks;
double M1_ticks_moved = 0;
double M2_ticks_moved = 0;
double Total_M1_moved = 0;
double Total_M2_moved = 0;
double ticks_to_move = 245;     // Gives approximately 10cm.
double M1_setpoint_ticks = 5;   // Number of ticks before each iteration of PID controller.
double M2_setpoint_ticks = 5;   // Number of ticks before each iteration of PID controller.

// Proportional Integral Derivative (PID) controller.
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

// Second method PID control law variables - Not used.
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

// Sensor values for distance from obstacles.
int B_Left_distance;
int B_Right_distance;
int Left_distance;
int Right_distance;
int F_Left_distance;
int F_Right_distance;

// Boolean variables indicate if there are obstacles surrounding the robot.
boolean obstacle_front = false;
boolean obstacle_back = false;
boolean obstacle_left = false;
boolean obstacle_right = false;

// For movement control when receiving commands for navigating around obstacles.
boolean waitingInput = true;
char readChar = ' ';
boolean backwards = false;
int backwards_count = 0;

// CREATE OBJECTS.******************************************************************************************

// Create an object for the motor shield.
DualVNH5019MotorShield motorShield;

// Create a new instance of the SharpIR class for each small and large sensor.
SharpIR PS1 = SharpIR(PS1_pin, small_model);  // Back left sensor.
SharpIR PS2 = SharpIR(PS2_pin, small_model);  // Back right sensor.
SharpIR PS3 = SharpIR(PS3_pin, small_model);  // Left sensor.
SharpIR PS4 = SharpIR(PS4_pin, small_model);  // Right sensor.
SharpIR PS5 = SharpIR(PS5_pin, large_model);  // Front left sensor.
SharpIR PS6 = SharpIR(PS6_pin, large_model);  // Front right sensor.

// SETUP - RUNS ONCE.***************************************************************************************
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

  // Get the initial readings from each sensor for any obstacles detected and their distances.
  B_Left_distance = PS1.distance() + 2;     // Need to +2cm for improved accuracy.
  B_Right_distance = PS2.distance() + 1;    // Need to +1cm for improved accuracy.
  Left_distance = PS3.distance() + 2;       // Need to +2cm for improved accuracy.
  Right_distance = PS4.distance() + 2;      // Need to +2cm for improved accuracy.
  F_Left_distance = PS5.distance() + 1;     // Need to +1cm +more as distance increases for improved accuracy.
  F_Right_distance = PS6.distance() + 2;    // Need to +2cm +more as distance increases for improved accuracy.
}

// LOOPING - RUNS CONTINUOUSLY.*****************************************************************************

void loop()
{
  // METHOD 1 - READ FROM SERIAL.***************************************************************************
  /*
   // Check if data has been received at the serial link (USB).
   while(waitingInput and Serial.available() > 0)
  {
    // Read up to the entire string that is passed in.
    String data = Serial.readStringUntil("\n");

    // Capture the first character in a variable, remaining characters are ignored.
    readChar = data.charAt(0);

    // Robot has received a command and does not need to wait for further input.
    waitingInput = false;

    // If the command is to move forwards or backwards, set the tick value.
    if(readChar == 'w' or readChar == 's')
    {
    // Optimal value: 283.
      ticks_to_move = 283;
    }

    // If the command is to rotate left or right by 90 degrees, set the tick value.
    if(readChar == 'a' or readChar == 'd')
    {
    // Optimal value: 393.
      ticks_to_move = 393;
    }
  }
  */ 
  // METHOD 2 - AUTO DETECT OBSTACLES AND STEER AROUND THEM.************************************************

  // If the robot is not commanded externally, it must wait for input from
  // Its sensors to navigate on its own.
  while(waitingInput)
  {
    delay(500);

    // If the robot is required to reverse.
    if(backwards == true)
    {
      // If there are no obstacles on its left or right.
      if(Left_distance > 10 or Right_distance > 10)
      {
        // Reverse by only one step.
        backwards_count += 1;
        readChar = 's';
      }

      // If there are obstacles on both its left and right.
      if(backwards_count > 2)
      {
        // Check if the robot's left is clear to move.
        if(Left_distance > 10)
        {
          readChar = 'a';
        }

        // Otherwise check if the robot's right is clear to move.
        else if ( Right_distance > 10)
        {
          readChar = 'd';
        }

        // Reset the reversing parameters.
        backwards_count = 0;
        backwards = false;  
      }
    }

    // If the robot is not required to reverse.
    else
    {
      // While there are no obstacles detected in front.
      if(F_Left_distance > 20 and F_Right_distance > 20)
      {
        readChar = 'w';
      }

      // Otherwise if obstacles are detected in front.
      else
      {
        // Check if the robot's left is clear to move.
        if(Left_distance > 10)
        {
          readChar = 'a';
        }

        // Otherwise check if the robot's right is clear to move.
        else if(Right_distance > 10)
        {
          readChar = 'd';
        }

        // Otherwise, the robot is surrounded by obstacles on front,
        // Left and right, and must reverse.
        else if(Left_distance < 10 && Right_distance < 10)
        {
          readChar = 's';
          backwards = true;
        }
      }

      // If the command is to move forwards or backwards, set the tick value.
      if(readChar == 'w' or readChar == 's')
      {
        // Optimal value: 283.
        ticks_to_move = 283;
      }

      // If the command is to rotate left or right by 90 degrees, set the tick value.
      if(readChar == 'a' or readChar == 'd')
      {
        // Optimal value: 393.
        ticks_to_move = 393;
      }

      // The robot has received a command input from the sensors and does not have
      // To wait for further input.
      waitingInput = false;
    }
  }

  // If an input has already been given, it needs to be executed here.
  if ( !waitingInput )
  {
    // Read the input command given.
    switch(readChar)
    {
      // Move forward. Both motors have positive speed values.
      case 'w': PID(1,1);
                break;

      // Move backwards. Both motors have negative speed values.
      case 's': PID(-1,-1);
                break;

      // Rotate to the left. Left motor is negative and right motor is positive.
      case 'a': PID(-1,1);
                break;

      // Rotate to the right. Left motor is positive and right motor is negative.
      case 'd': PID(1,-1);
                break;
    }
    
    // Once the desired number of ticks to move the required distance (10cm) has been reached.
    if(M1_ticks_moved > ticks_to_move and M2_ticks_moved > ticks_to_move)
    {
      // For debugging, two master counters count the total number of ticks moved through
      // the entire motor operation to check for tick discrepancies over time.
      Total_M1_moved += M1_ticks_moved;
      Total_M2_moved += M2_ticks_moved;
      
      //Serial.print("R ticks moved : "); Serial.print(M1_ticks_moved);
      //Serial.print(", L ticks moved : "); Serial.print(M2_ticks_moved);
      Serial.print(", Total right ticks moved : "); Serial.print(Total_M1_moved);
      Serial.print(", Total left ticks moved : "); Serial.println(Total_M2_moved);
      
      // Reset the tick counters.
      M1_ticks_moved = 0;
      M2_ticks_moved = 0;
  
      // Stop the robot movement, braking is more effective then setting the speed to 0.
      motorShield.setBrakes(400,400);

      // When the robot stops moving, read in sensor data.
      // May need to add in buffer distance to each measured value to reduce collision likelihood.
      B_Left_distance = PS1.distance() + 2;     // Need to +2cm for improved accuracy.
      B_Right_distance = PS2.distance() + 1;    // Need to +1cm for improved accuracy.
      Left_distance = PS3.distance() + 2;       // Need to +2cm for improved accuracy.
      Right_distance = PS4.distance() + 2;      // Need to +2cm for improved accuracy.
      F_Left_distance = PS5.distance() + 1;     // Need to +1cm +more as distance increases for improved accuracy.
      F_Right_distance = PS6.distance() + 2;    // Need to +2cm +more as distance increases for improved accuracy.

      // Once finished executing the current iteration of command, the robot
      // Waits for either another command input from the serial link or its sensors.
      waitingInput = true;
    }
  }
}

// PID CONTROLLER.*****************************************************************************************

// Input multipliers determine the rotation direction of each wheel.
void PID(int right_mul , int left_mul)
{
  // Sum up the total number of ticks per iteration of PID computation.
  M1_ticks_moved += right_ticks;
  M2_ticks_moved += left_ticks;

  // Determine the error difference in the number of ticks.
  E1_error_ticks = M1_setpoint_ticks - right_ticks;
  E2_error_ticks = M2_setpoint_ticks - left_ticks;

  // NOTE: PID CALCULATION 'KP', 'KI', 'KD' VALUES NEED TO BE MANUALLY TUNED FOR EACH MOTOR.
  
  // Perform PID calculation for the new motor speed for the right motor.
  M1_ticks_PID = right_ticks + (E2_error_ticks * KP) + (E2_prev_error * KD) + (E2_sum_error * KI * 1.19);

  // Perform PID calculation for the new motor speed for the left motor.
  M2_ticks_PID = left_ticks + (E1_error_ticks * KP * 0.95) + (E1_prev_error * (KD + 0.2)) + (E1_sum_error * KI * 0.98);

  // For debugging.
  //Serial.print(", Right (M1) PID ticks: "); Serial.print(M1_ticks_PID);
  //Serial.print(", Left (M2) PID ticks: "); Serial.println(M2_ticks_PID);

  // Convert the adjusted ticks to the new motor power using the functions below.
  right_speed = right_ticks_to_power(M1_ticks_PID);
  left_speed = left_ticks_to_power(M2_ticks_PID);
  
  // Set the new motor speed and direction with the multiplier.
  motorShield.setM1Speed(right_speed * right_mul);    // Right motor.
  motorShield.setM2Speed(left_speed * left_mul);     // Left motor.
  stopIfFault();

  // For debugging.
  //Serial.print(", R power: "); Serial.print(right_speed);
  //Serial.print(", L power: "); Serial.println(left_speed);

  // Reset the tick count for the next iteration after the PID computation is performed.
  right_ticks = 0;
  left_ticks = 0;

  // Store the last error value for the next computation.
  E1_prev_error = E1_error_ticks;
  E2_prev_error = E2_error_ticks;

  // Store the sum of all errors. Errors with negative values are subtracted from the sum.
  E1_sum_error += E1_error_ticks;
  E2_sum_error += E2_error_ticks;

  // Perform the PID computation again after a sampling time.
  delay(10);
}

// INTERRUPT, FAULT STOP AND MOTOR TICKS TO POWER FUNCTIONS.************************************************

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

// Based on obtained data of ticks moved vs set power for right motor,
// Convert right motor ticks to commanded power.
double right_ticks_to_power(double right_ticks)
{
  return (right_ticks * 0.22748) + 20.06839;
}

// Based on obtained data of ticks moved vs set power for left motor,
// Convert left motor ticks to commanded power.
double left_ticks_to_power(double left_ticks)
{
  return (left_ticks * 0.221907) + 31.70289;
}

// Stops the motor if there is a fault, like a short circuit.
void stopIfFault()
{
  if (motorShield.getM1Fault())
  {
    Serial.println("Left motor fault.");

    // Infinite loop stops the program from continuing.
    while(1);
  }
  if (motorShield.getM2Fault())
  {
    Serial.println("Right motor fault.");

    // Infinite loop stops the program from continuing.
    while(1);
  }
}
