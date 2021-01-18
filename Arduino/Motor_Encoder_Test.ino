// DRIVE MOTOR ENCODER TESTING FILE.
// PRINTS THE DURATION OF EACH MOTOR ENCODER'S SQUARE WAVE IN MILLISECONDS TO THE CONSOLE.

// Declare input pins, each encoder has 2 inputs.
byte encoderE1 = 3;   // Right motor encoder.
byte encoderE2 = 11;  // Left motor encoder.

// Variables to store the duration that each encoder square wave remains high.
int encoderE1_PWM = 0;
int encoderE2_PWM = 0;

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
  //Send and receive serial monitor console data at rate of 115200 baud.
  Serial.begin(9600);

  // Initialize the inputs.
  pinMode(encoderE1,INPUT);
  pinMode(encoderE2,INPUT);

  // Initialize the motor shield driver object.
  motorShield.init();
}

// Looping code runs continuously.
void loop()
{
  // MOVE FORWARDS. (Positive value.)
  motorShield.setM1Speed(107);  // 107
  motorShield.setM2Speed(100);  // 100
  stopIfFault();
  delay(500);
  
  // Obtain the duration in milliseconds that the encoder output square wave remains high.
  // A faster motor speed will result in a smaller milliseconds value.
  encoderE1_PWM = pulseIn(encoderE1,HIGH);
  encoderE2_PWM = pulseIn(encoderE2,HIGH);

  Serial.print("Encoder E1 Right Motor PWM: ");
  Serial.print(encoderE1_PWM);
  Serial.print(", Encoder E2 Left Motor PWM: ");
  Serial.println(encoderE2_PWM);
}
