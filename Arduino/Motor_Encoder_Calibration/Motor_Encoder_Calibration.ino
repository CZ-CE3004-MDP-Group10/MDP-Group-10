// DRIVE MOTOR ENCODER TESTING FILE.
// PRINTS THE DURATION OF EACH MOTOR ENCODER'S SQUARE WAVE IN MILLISECONDS TO THE CONSOLE.

// Declare input pins, each encoder has 2 inputs.
byte encoderE1 = 3;   // Right motor encoder.
byte encoderE2 = 11;  // Left motor encoder.

// Variables to store the duration that each encoder square wave remains high.
double encoderE1_PWM = 0.0;
double encoderE2_PWM = 0.0;
double E1_M1_Right_RPM = 0.0;
double E2_M2_Left_RPM = 0.0;
double right_RPM_avg = 0.0;
double left_RPM_avg = 0.0;
int count = 0;

// Include the library for the motor shield driver and PID controller.
#include "DualVNH5019MotorShield.h"

// Create an object for the motor shield and PID controller.
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
  //Send and receive serial monitor console data at rate of 115200 baud.
  Serial.begin(115200);

  // Initialize the inputs.
  pinMode(encoderE1,INPUT);
  pinMode(encoderE2,INPUT);

  // Initialize the motor shield driver object.
  motorShield.init();

  // MOVE FORWARDS. (Positive value.)
  // Adjust the speed here from 0 to 400 with incrementing steps of 50.
  motorShield.setM1Speed(100);
  motorShield.setM2Speed(100);
  stopIfFault();
}

// Looping code runs continuously.
void loop()
{
  for(count = 0; count < 10; count++)
  {
    // Obtain the duration in microseconds that the encoder output square wave is high.
    // A faster motor speed will result in a smaller microseconds integer value.
    encoderE1_PWM = pulseIn(encoderE1,LOW);
    encoderE2_PWM = pulseIn(encoderE2,LOW);

    // Obtain the duration of an entire square wave for both motors.
    // Assuming the duty cycle of the square wave is 50%.
    encoderE1_PWM *= 2;   // Right motor.
    encoderE2_PWM *= 2;   // Left motor.

    // Find the total number of square waves in a second.
    // Supposed to be 1sec/duration of square wave in microseconds,
    // But values are too small for working, so multiply each side by 10^6.
    E1_M1_Right_RPM = 1000000/encoderE1_PWM;
    E2_M2_Left_RPM = 1000000/encoderE2_PWM;

    // If the motor is stationary.
    if(encoderE1_PWM == 0)
    {
      E1_M1_Right_RPM = 0;
    }
    if(encoderE2_PWM == 0)
    {
      E2_M2_Left_RPM = 0;
    }
    // Find the number of wheel revolutions in a minute.
    E1_M1_Right_RPM = (E1_M1_Right_RPM / 562.25) * 60;
    E2_M2_Left_RPM = (E2_M2_Left_RPM / 562.25) * 60;

    right_RPM_avg += E1_M1_Right_RPM;
    left_RPM_avg += E2_M2_Left_RPM;
    delay(500);
  }
  // Obtain an average for every 10 readings.
  right_RPM_avg /= 10;
  left_RPM_avg /= 10;

  Serial.print("Right Motor RPM: ");
  Serial.print(right_RPM_avg);
  Serial.print(", Left Motor RPM: ");
  Serial.println(left_RPM_avg);

  // Reset the counter and variables.
  count = 0;
  right_RPM_avg = 0.0;
  left_RPM_avg = 0.0;
}
