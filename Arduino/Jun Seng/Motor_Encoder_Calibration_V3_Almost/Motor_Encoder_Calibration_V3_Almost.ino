// VERSION 2 - IMPLEMENT THE PID CONTROLLER.
// Include the required libraries.
#include "DualVNH5019MotorShield.h"
#include "EnableInterrupt.h"

// Create an object for the motor shield.
DualVNH5019MotorShield motorShield;

// Right Motor.
#define encoder1A 3   // Output 1.
#define encoder1B 5   // Output 2.

// Left Motor.
#define encoder2A 11  // Output 1.
#define encoder2B 13  // Output 2.

// Motor commanded power variables, set the initial motor speed here.
// Note that right motor is slightly faster than left motor.
double right_speed = 150.0;
double left_speed = 150.0;

// Tick count variables.
int right_ticks;
int left_ticks;

int tick_setpoint = 300;

// PID function output variables.
double right_ticks_PID = 0.0;
double left_ticks_PID = 0.0;

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

/*-----PID Variables-----*/
double M1_ticks_PID = 0;
double M2_ticks_PID = 0;
double M1_setpoint_ticks = 100;
double M2_setpoint_ticks = 100;
double E1_error_ticks = 0;
double E2_error_ticks = 0;
double E1_prev_error = 0;
double E2_prev_error = 0;
double E1_sum_error = 0; //16919;
double E2_sum_error = 0; //17198;
double KP = 1;
double KD = 3;
double KI = 2;

// ----------------------------------------------------------------------------------
// Setup code runs once at startup.
void setup()
{
  //Send and receive serial monitor console data at rate of 115200 baud.
  Serial.begin(115200);

  // Initialize the motor shield driver object.
  motorShield.init();

  // Read in pulse wave data from the first pin of each encoder.
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pulseIn(encoder1A, HIGH);

  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);
  pulseIn(encoder2A, HIGH);

  // Enable interrupts for each encoder input pin 1 on the rising edge.
  // Function 'right_tick_increment" is called on rising edge of input 'encoder1A'.
  enableInterrupt(encoder1A, right_tick_increment, RISING);
  enableInterrupt(encoder2A, left_tick_increment, RISING);

  // Set the motor initial speed.
  motorShield.setM1Speed(150);    // Right motor.
  motorShield.setM2Speed(160);     // Left motor.
  stopIfFault();
  delay(100);
}

// Looping code runs continuously.
void loop()
{
  // Assume we want about 60 RPM, which is about 562 or 563 ticks per second.
  Serial.print("R ticks: "); Serial.print(right_ticks);
  Serial.print(", L ticks: "); Serial.print(left_ticks);
  
  E1_error_ticks = M1_setpoint_ticks - right_ticks;
  E2_error_ticks = M2_setpoint_ticks - left_ticks;
  
  M1_ticks_PID = right_ticks + (E1_error_ticks * (KP * 0.95)) + (E1_prev_error * (KD + 0.01)) + (E1_sum_error * KI);
  M2_ticks_PID = left_ticks + (E2_error_ticks * KP) + (E2_prev_error * KD) + (E2_sum_error * KI);

  //convert adjusted ticks to RPM
  right_speed = right_ticks_to_power(M2_ticks_PID);
  left_speed = left_ticks_to_power(M1_ticks_PID);
  
  // Set the new motor speed.
  motorShield.setM1Speed(right_speed);    // Right motor.
  motorShield.setM2Speed(left_speed);     // Left motor.
  stopIfFault();

  Serial.print(", R power: "); Serial.print(right_speed);
  Serial.print(", L power: "); Serial.println(left_speed);

  // Reset the tick count after the PID has been performed.
  right_ticks = 0;
  left_ticks = 0;

  E1_prev_error = E1_error_ticks;
  E2_prev_error = E2_error_ticks;

  E1_sum_error += E1_error_ticks;
  E2_sum_error += E2_error_ticks;

  delay(100);
}

// ----------------------------------------------------------------------------------
void right_tick_increment()
{
  right_ticks++;
}

void left_tick_increment()
{
  left_ticks++;
}

// Based on obtained data of power vs ticks for right motor.
double right_ticks_to_power(double right_ticks)
{
  return (right_ticks * 0.22748) + 20.06839;
}

// Based on obtained data of power vs ticks for left motor.
double left_ticks_to_power(double left_ticks)
{
  return (left_ticks * 0.221907) + 31.70289;
}

// ----------------------------------------------------------------------------------
// Functions to move the robot in each particular direction.
void Forward(int duration)
{
  motorShield.setM1Speed(left_speed);
  motorShield.setM2Speed(right_speed);
  delay(duration);
}

void Backward(int duration)
{
  motorShield.setM1Speed(-left_speed);
  motorShield.setM2Speed(-right_speed);
  delay(duration);
}

void Left(int duration)
{
  motorShield.setM1Speed(-left_speed);
  motorShield.setM2Speed(right_speed);
  delay(duration);
}

void Right(int duration)
{
  motorShield.setM1Speed(left_speed);
  motorShield.setM2Speed(-right_speed);
  delay(duration);
}

void Stop(int duration)
{
  motorShield.setM1Speed(0);
  motorShield.setM2Speed(0);
  delay(duration);
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
