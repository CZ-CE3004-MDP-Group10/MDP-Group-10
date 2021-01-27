// Include the required libraries.
#include "DualVNH5019MotorShield.h"
#include "EnableInterrupt.h"
#include <SharpIR.h>

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

// -----------------------------------------------------------------------------------
// SHARP IR SENSOR LIBRARY CALIBRATION.

// Define IR sensor input pins.
#define PS1_pin A0
#define PS2_pin A1
#define PS3_pin A2
#define PS4_pin A3
#define PS5_pin A4
#define PS6_pin A5

// Define IR sensor models, both large and small are used.
#define large_model 20150
#define small_model 1080

// Create variable to store the distance, all in centimetres.
int B_Left_distance;
int B_Right_distance;
int Left_distance;
int Right_distance;
int F_Left_distance;
int F_Right_distance;

// Boolean variables to indicate if there are obstacles surrounding the robot.
boolean obstacle_front = false;
boolean obstacle_back = false;
boolean obstacle_left = false;
boolean obstacle_right = false;

// Create a new instance of the SharpIR class for each small and large sensor.
SharpIR PS1 = SharpIR(PS1_pin, small_model);  // Back left sensor.
SharpIR PS2 = SharpIR(PS2_pin, small_model);  // Back right sensor.
SharpIR PS3 = SharpIR(PS3_pin, small_model);  // Left sensor.
SharpIR PS4 = SharpIR(PS4_pin, small_model);  // Right sensor.
SharpIR PS5 = SharpIR(PS5_pin, large_model);  // Front left sensor.
SharpIR PS6 = SharpIR(PS6_pin, large_model);  // Front right sensor.



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

  B_Left_distance = PS1.distance() + 2;    // Need to +2cm for improved accuracy.
  B_Right_distance = PS2.distance() + 1;    // Need to +1cm for improved accuracy.
  Left_distance = PS3.distance() + 2;    // Need to +2cm for improved accuracy.
  Right_distance = PS4.distance() + 2;    // Need to +2cm for improved accuracy.
  F_Left_distance = PS5.distance() + 1;    // Need to +1cm +more as distance increases for improved accuracy.
  F_Right_distance = PS6.distance() + 2;    // Need to +2cm +more as distance increases for improved accuracy.

  
  // Set the motor initial speed if desired. - May be redundant.
  //motorShield.setM1Speed(100);    // Right motor.
  //motorShield.setM2Speed(150);     // Left motor.
  //stopIfFault();
  //delay(200);
}

boolean waitingInput = true;
char readChar = '0';

boolean backwards = false;
int backwards_count = 0;

int front_back_ticks = 245;
int left_right_ticks = 380;

int left_mul = 0;
int right_mul = 0;

// Looping code runs continuously.
void loop()
{
  //************************* Read from Serial ********************************* 
  
   while(waitingInput and Serial.available() > 0)
  {
    String data = Serial.readStringUntil("\n");
    readChar = data.charAt(0);

    waitingInput = false;

    if(readChar == 'w' or readChar == 's')
    {
      ticks_to_move = front_back_ticks;
    }
    if(readChar == 'a' or readChar == 'd')
    {
      ticks_to_move = left_right_ticks;
    }
    
  }
  
  //************************* Auto detect and steer v1 *********************************
  /*
  while(waitingInput)
  {
    delay(500);
    
    if(F_Left_distance > 20 and F_Right_distance > 20)
    {
      readChar = 'w';
    }
    else
    {
      if(Left_distance > 40)
      {
        readChar = 'a';
      }
      else if(Right_distance > 40)
      {
        readChar = 'd';
      }
      else if(Left_distance < 40 && Right_distance < 40)
      {
        readChar = 's';
        backwards = true;
      }
    }
    
     if(readChar == 'w' or readChar == 's')
    {
      ticks_to_move = 245;
    }
    if(readChar == 'a' or readChar == 'd')
    {
      ticks_to_move = 400;
    }

    waitingInput = false;
    
  }
  */

  //************************* Auto detect and steer v2 *********************************
  /*
  while(waitingInput)
  {
    delay(500);
    if(backwards == true)
    {
      if(Left_distance > 40 or Right_distance > 40)
      {
        backwards_count += 1;
        readChar = 's';
      }
      
      if(backwards_count > 2)
      {

        if(Left_distance > 40)
        {
          readChar = 'a';
        }
        else if ( Right_distance > 40)
        {
          readChar = 'd';
        }
  
        backwards_count = 0;
        backwards = false;  
      }
      
    }
    else
    {
      if(F_Left_distance > 20 and F_Right_distance > 20)
      {
        readChar = 'w';
      }
      else
      {
        if(Left_distance > 40)
        {
          readChar = 'a';
        }
        else if(Right_distance > 40)
        {
          readChar = 'd';
        }
        else if(Left_distance < 40 && Right_distance < 40)
        {
          readChar = 's';
          backwards = true;
        }
      }
      
       if(readChar == 'w' or readChar == 's')
      {
        ticks_to_move = front_back_ticks;
      }
      if(readChar == 'a' or readChar == 'd')
      {
        ticks_to_move = left_right_ticks;
      }
  
      waitingInput = false;
    }
  }
  */

  
  

  if ( !waitingInput )
  {
    switch(readChar)
    {
      case 'w': setMul(1,1);
                break;
      
      case 's': setMul(-1,-1);
                break;

      case 'a': setMul(-1,1);
                break;

      case 'd': setMul(1,-1);
                break;
    }

    /*
    if(M1_ticks_moved > ticks_to_move)
    {
      right_mul = 0;
      motorShield.setBrakes(0,400);
      Serial.print("Hit here");
    }
    if(M2_ticks_moved > ticks_to_move)
    {
      left_mul = 0;
      motorShield.setBrakes(400,0);
      Serial.print("Hit there");
    }*/

    PID();
  
    // Once the desired number of ticks to move the required distance (10cm) has been reached.
    if(M1_ticks_moved > ticks_to_move and M2_ticks_moved > ticks_to_move)
    {
      Total_M1_moved += M1_ticks_moved;
      Total_M2_moved += M2_ticks_moved;
      
      Serial.print("R ticks moved : "); Serial.print(M1_ticks_moved);
      Serial.print(", L ticks moved : "); Serial.print(M2_ticks_moved);
      Serial.print(", Total right ticks moved : "); Serial.print(Total_M1_moved);
      Serial.print(", Total left ticks moved : "); Serial.println(Total_M2_moved);
      
      // Reset the tick counters.
      M1_ticks_moved = 0;
      M2_ticks_moved = 0;

      waitingInput = true;
  
      // Stop the robot movement, braking is more effective then setting the speed to 0.
      motorShield.setBrakes(400,400);

      // Check sensor when stopped 
      // May wish to add in buffer distance to each measured value to reduce collision likelihood.
      B_Left_distance = PS1.distance() + 2;    // Need to +2cm for improved accuracy.
      B_Right_distance = PS2.distance() + 1;    // Need to +1cm for improved accuracy.
      Left_distance = PS3.distance() + 2;    // Need to +2cm for improved accuracy.
      Right_distance = PS4.distance() + 2;    // Need to +2cm for improved accuracy.
      F_Left_distance = PS5.distance() + 1;    // Need to +1cm +more as distance increases for improved accuracy.
      F_Right_distance = PS6.distance() + 2;    // Need to +2cm +more as distance increases for improved accuracy.
      
    }
  }
}

void setMul(int right , int left)
{
  right_mul = right;
  left_mul = left;
}

void PID()//int right_mul , int left_mul)
{
  // Sum up the total number of ticks per iteration of PID computation.
  M1_ticks_moved += right_ticks;
  M2_ticks_moved += left_ticks;

  // Two master counters count the total number of ticks moved through
  // the entire motor operation to check for discrepancies over time.
  Serial.print("R ticks moved : "); Serial.print(M1_ticks_moved);
  Serial.print(", L ticks moved : "); Serial.print(M2_ticks_moved);

  // Determine the error difference in the number of ticks.
  E1_error_ticks = M1_setpoint_ticks - right_ticks;
  E2_error_ticks = M2_setpoint_ticks - left_ticks;

  // Perform PID calculation for the new motor speed.
  M1_ticks_PID = right_ticks + (E2_error_ticks * KP) + (E2_prev_error * KD) + (E2_sum_error * KI);
  M2_ticks_PID = left_ticks + (E1_error_ticks * KP * 0.95) + (E1_prev_error * (KD + 0.2)) + (E1_sum_error * KI);

  //Serial.print(", Right (M1) PID ticks: "); Serial.print(M1_ticks_PID);
  //Serial.print(", Left (M2) PID ticks: "); Serial.println(M2_ticks_PID);

  // Convert the adjusted ticks to the new motor power.
  // Due to gradients of both motors (see functions below), the 'C' constant
  // Compensation is larger for the left motor, nd smaller for the right motor,
  // Hence the left ticks PID is fed to the right motor,
  // And the right ticks PID is fed to the left motor.
  right_speed = right_ticks_to_power(M1_ticks_PID);
  left_speed = left_ticks_to_power(M2_ticks_PID);
  
  // Set the new motor speed.
  motorShield.setM1Speed(right_speed * right_mul);    // Right motor.
  motorShield.setM2Speed(left_speed * left_mul);     // Left motor.
  stopIfFault();

  Serial.print(", R power: "); Serial.print(right_speed);
  Serial.print(", L power: "); Serial.println(left_speed);

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
