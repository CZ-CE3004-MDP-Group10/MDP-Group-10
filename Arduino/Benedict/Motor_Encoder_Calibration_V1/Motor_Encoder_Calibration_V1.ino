// VERSION 1 - VIEW TICKS DIFFERENCE ONLY.
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

// Encoder tick variables. There are 562.25 ticks (square waves) per wheel revolution.
int right_ticks = 0;   // Right motor.
int left_ticks = 0;    // Left motor.

// Set the number of ticks we want.
int set_right_ticks = 562;
int set_left_ticks = 562;

// Error difference in ticks between the desired and current values.
int error_right_ticks = 0;
int error_left_ticks = 0;

// Motor commanded power variables.
double right_speed = 160.0;
double left_speed = 160.0;

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
  enableInterrupt(encoder1A, right_tick_increment, RISING);
  enableInterrupt(encoder2A, left_tick_increment, RISING);

  // Set the motor initial speed.
  motorShield.setM1Speed(right_speed);    // Right motor.
  motorShield.setM2Speed(left_speed);     // Left motor.
  stopIfFault();
}

// Looping code runs continuously.
void loop()
{
  // Assume we want about 60 RPM, which is about 562 or 563 ticks per second.
  //Serial.print("Right ticks per second: "); Serial.print(right_ticks);
  //Serial.print(", Left ticks per second: "); Serial.println(left_ticks);

  // Display the error difference in set ticks and actual ticks.
  // Positive value indictaes it is too slow, negative value indicates it is too fast.
  Serial.print("Right ticks difference / sec: "); Serial.print(error_right_ticks);
  Serial.print(", Left ticks difference / sec: "); Serial.println(error_left_ticks);

  // Determine the error difference in the ticks of both motors.
  error_right_ticks = set_right_ticks - right_ticks;
  error_left_ticks = set_left_ticks - left_ticks;

  // Perform the correction to reduce the error ticks on both left and right
  // side motors to be very close to or equal to zero, to ensure both motors 
  // are travelling at the same RPM.
  

  // Reset the tick count.
  right_ticks = 0;
  left_ticks = 0;
  delay(1000);
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
