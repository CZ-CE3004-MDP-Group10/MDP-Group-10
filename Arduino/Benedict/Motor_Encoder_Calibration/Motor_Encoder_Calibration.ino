// Include the library for the motor shield driver.
#include "DualVNH5019MotorShield.h"

// Create an object for the motor shield and PID controller.
DualVNH5019MotorShield motorShield;

// Declare input pins, each encoder has 2 inputs, only one is used.
byte encoderE1_right = 3;   // Right motor encoder.
byte encoderE2_left = 11;   // Left motor encoder.

// ------------------------------------------------------------------------
// STEP 1: SET REQUIRED WHEEL SPEED.
// Robot is moving forwards in this example.
double set_right_RPM = 80.0;      // Set the RPM for the right wheel.
double set_left_RPM = 80.0;       // Set the RPM for the left wheel.
double set_right_speed = 100.0;   // Set the initial right motor speed.
double set_left_speed = 100.0;    // Set the initial left motor speed.
double E1_right_PWM = 0.0;
double E2_left_PWM = 0.0;
double E1_right_RPM = 0.0;
double E2_left_RPM = 0.0;
double error_right_RPM = 0.0;
double error_left_RPM = 0.0;

// STEP 2: CALCULATE VALUES OF K1, K2 AND K3 for both motors.
double right_K1;
double right_K2;
double right_K3;
double left_K1;
double left_K2;
double left_K3;
// ------------------------------------------------------------------------

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
  //Send and receive serial monitor console data at rate of 115200 baud.
  Serial.begin(115200);

  // Initialize the inputs.
  pinMode(E1_right_PWM,INPUT);
  pinMode(E2_left_PWM,INPUT);

  // Initialize the motor shield driver object.
  motorShield.init();

  // Move Forwards - Positive value, move backwards, negative value.
  // For calibration, adjust the speed from 50 to 350 with incrementing steps of 50.
  motorShield.setM1Speed(set_right_speed);    // Right motor.
  motorShield.setM2Speed(set_left_speed);     // Left motor.
  stopIfFault();
}

// Looping code runs continuously.
void loop()
{
  // ------------------------------------------------------------------------
  // STEP 3: READ ENCODER VALUES AND CONVERT THEM INTO RPM.
  // Obtain the duration in microseconds that the encoder output square wave is high.
  // A faster motor speed will result in a smaller microseconds value.
  E1_right_PWM = pulseIn(encoderE1_right,HIGH);
  E2_left_PWM = pulseIn(encoderE2_left,HIGH);

  // Obtain the duration of an entire square wave for both motors.
  // Assuming the duty cycle of the square wave is 50%.
  E1_right_PWM *= 2;   // Right motor.
  E2_left_PWM *= 2;   // Left motor.

  // Find the total number of square waves in a second.
  // Supposed to be 1sec / <duration of square wave in microseconds>,
  // But values are too small, so multiply each side by 10^6.
  E1_right_RPM = 1000000 / E1_right_PWM;
  E2_left_RPM = 1000000 / E2_left_PWM;

  // If either motor is stationary.
  if(E1_right_PWM == 0) {E1_right_RPM = 0;}
  if(E2_left_PWM == 0) {E2_left_RPM = 0;}
  
  // Find the number of wheel revolutions in a minute.
  E1_right_RPM = (E1_right_RPM / 562.25) * 60;
  E2_left_RPM = (E2_left_RPM / 562.25) * 60;

  // Debugging - View actual motor RPM.
  Serial.print("Right Motor Actual RPM: ");
  Serial.print(E1_right_RPM);
  Serial.print(", Left Motor Actual RPM: ");
  Serial.print(E2_left_RPM);
  
  // ------------------------------------------------------------------------
  // STEP 4: CALCULATE ERROR BETWEEN THE SET RPM AND RECORDED RPM.
  // Using the absolute value of the error margin for each motor.
  error_right_RPM = abs(set_right_RPM - E1_right_RPM);
  error_left_RPM = abs(set_left_RPM - E2_left_RPM);
  
  // ------------------------------------------------------------------------
  // STEP 5: COMPUTE THE NEW MOTOR POWER VALUES BASED ON THE PID CONTROL LAW.
  // TO DO.
  // set_right_speed = 
  // set_left_speed = 
  
  // ------------------------------------------------------------------------
  // STEP 6: SET THE NEW VALUES AS THE MOTOR SPEED CONTROL.
  motorShield.setM1Speed(set_right_speed);    // Right motor.
  motorShield.setM2Speed(set_left_speed);    // Left motor.
  stopIfFault();

  // Debugging - View motor speed control.
  Serial.print(", Right Motor Speed: ");
  Serial.print(set_right_speed);
  Serial.print(", Left Motor Speed: ");
  Serial.println(set_left_speed);
  
  // ------------------------------------------------------------------------
  // STEP 7: WAIT FOR A SAMPLING TIME BEFORE REPEATING THE CALIBRATION PROCESS.
  delay(100);
  // ------------------------------------------------------------------------
}

// Functions to move the robot in each particular direction.
void Forward(float duration)
{
  motorShield.setM1Speed(leftSpeed);
  motorShield.setM2Speed(rightSpeed);
  delay(duration);
}

void Backward(float duration)
{
  motorShield.setM1Speed(-leftSpeed);
  motorShield.setM2Speed(-rightSpeed);
  delay(duration);
}

void Left(float duration)
{
  motorShield.setM1Speed(-leftSpeed);
  motorShield.setM2Speed(rightSpeed);
  delay(duration);
}

void Right(float duration)
{
  motorShield.setM1Speed(leftSpeed);
  motorShield.setM2Speed(-rightSpeed);
  delay(duration);
}

void Stop(float duration)
{
  motorShield.setM1Speed(0);
  motorShield.setM2Speed(0);
  delay(duration);
}
