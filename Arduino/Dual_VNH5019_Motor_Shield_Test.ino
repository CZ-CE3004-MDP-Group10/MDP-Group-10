// DRIVE MOTORS TESTING FILE.
// TESTS FORWARDS AND BACKWARDS MOVEMENT, AND LEFT AND RIGHT STEERING.

// Motor M1 -> Left motor viewed from front.
// Motor M2 -> Right motor viewed from front.

// Left motor M1 is slightly slower than the right motor M2, hence its speed 
// value must be slightly higher. Need to fix this issue using encoder. 
// Current consumption in milliamps is obtained midway through each operation.

// Include the library for the motor shield driver.
#include "DualVNH5019MotorShield.h"

// Create an object for the motor shield.
DualVNH5019MotorShield motorShield;

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

// Setup code runs once at startup.
void setup()
{
  Serial.begin(9600);
  Serial.println("Dual VNH5019 Motor Shield");

  // Initialize the motor shield driver object.
  motorShield.init();
}

// Looping code runs continuously.
void loop()
{
  // MOVE FORWARDS. (Positive value.)
  motorShield.setM1Speed(107);
  motorShield.setM2Speed(100);
  stopIfFault();
  delay(1000);

  Serial.print("MOVE FORWARDS: ");
  Serial.print("Left motor current: ");
  Serial.print(motorShield.getM1CurrentMilliamps());
  Serial.print(", Right motor current: ");
  Serial.println(motorShield.getM2CurrentMilliamps());
  delay(1000);

  // STOP MOVING.
  motorShield.setM1Speed(0);
  motorShield.setM2Speed(0);
  stopIfFault();
  delay(1000);

  // MOVE BACKWARDS. (Negative value.)
  motorShield.setM1Speed(-107);
  motorShield.setM2Speed(-100);
  stopIfFault();
  delay(1000);

  Serial.print("MOVE BACKWARDS: ");
  Serial.print("Left motor current: ");
  Serial.print(motorShield.getM1CurrentMilliamps());
  Serial.print(", Right motor current: ");
  Serial.println(motorShield.getM2CurrentMilliamps());
  delay(1000);

  // STOP MOVING.
  motorShield.setM1Speed(0);
  motorShield.setM2Speed(0);
  stopIfFault();
  delay(1000);

  // STEER LEFT. (Left wheel backwards, right wheel forwards.)
  motorShield.setM1Speed(-107);
  motorShield.setM2Speed(100);
  stopIfFault();
  delay(1000);

  Serial.print("TURN LEFT: ");
  Serial.print("Left motor current: ");
  Serial.print(motorShield.getM1CurrentMilliamps());
  Serial.print(", Right motor current: ");
  Serial.println(motorShield.getM2CurrentMilliamps());
  delay(1000);

  // STOP MOVING.
  motorShield.setM1Speed(0);
  motorShield.setM2Speed(0);
  stopIfFault();
  delay(1000);

  // STEER RIGHT. (Right wheel backwards, left wheel forwards.)
  motorShield.setM1Speed(107);
  motorShield.setM2Speed(-100);
  stopIfFault();
  delay(1000);

  Serial.print("TURN RIGHT: ");
  Serial.print("Left motor current: ");
  Serial.print(motorShield.getM1CurrentMilliamps());
  Serial.print(", Right motor current: ");
  Serial.println(motorShield.getM2CurrentMilliamps());
  delay(1000);

  // STOP MOVING.
  motorShield.setM1Speed(0);
  motorShield.setM2Speed(0);
  stopIfFault();
  delay(1000);
}
