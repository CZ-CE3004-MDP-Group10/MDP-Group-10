// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 ARDUINO FILE.
// CAUTION: MOTOR MOVEMENT DISTANCES, ROTATION ANGLES AND SPEEDS ARE AFFECTED BY REMAINING BATTERY CAPACITIY.
// IMPORTANT: ALL SENSORS AND MOTORS MUST BE RECALIBRATED BEFORE ANY EVALUATION OR NAVIGATION.

// LIBRARIES.***********************************************************************************************
#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
//#include <Sensors.h>
//#include <PID.h>
#include <Movement.h>


// DEFINITIONS.*********************************************************************************************

// Motor M1 = Encoder E1 = Right.
// Motor M2 = Encoder E2 = Left.

// Motor encoder inputs
#define encoder_M1_A 3   // Right motor output 1.
#define encoder_M1_B 5   // Right motor output 2. Not used.
#define encoder_M2_A 11  // Left motor output 1.
#define encoder_M2_B 13  // Left motor output 2. Not used.

// VARIABLES.***********************************************************************************************

// For movement control when receiving commands for navigating around obstacles.
char readChar = ' ';            // Command character indicating the direction to move.
boolean waitingInput = true;    // Determines if the robot is waiting for input from serial link.

// CREATE OBJECTS.******************************************************************************************

// Create an object for the motor shield.
//DualVNH5019MotorShield motorShield;
//Sensors sensor;
//PID pid;
Movement robot;


// SETUP - RUNS ONCE.***************************************************************************************
void setup()
{
  //Send and receive serial monitor console data at rate of 115200 baud.
  Serial.begin(115200);

  // Initialize the motor shield driver object.
  //motorShield.init();
  //sensor.init();
  //pid.init();

  robot.init();

  // Set the encoder pins as input.
  pinMode(encoder_M1_A, INPUT);
  pinMode(encoder_M1_B, INPUT);
  pinMode(encoder_M2_A, INPUT);
  pinMode(encoder_M2_B, INPUT);

  // Enable interrupts for each encoder input pin 1 on the rising edge.
  // Function 'right_tick_increment" is called on rising edge of input 'encoder_M1_A'.
  enableInterrupt(encoder_M1_A, right_tick_increment, RISING);
  enableInterrupt(encoder_M2_A, left_tick_increment, RISING);

  // Introduce an initial delay to prevent power up surges from interfering.
  delay(3000);  
  
  // Send initial string of sensor readings when the robot is ready and initialized.
  robot.readSensor();
}

// LOOPING - RUNS CONTINUOUSLY.*****************************************************************************

void loop()
{
  // READ FROM SERIAL.**************************************************************************************
  
  // Check if data has been received at the serial link (USB).
  while(waitingInput and Serial.available() > 0)
  {
    // Read up to the entire string that is passed in up to the newline character.
    String data = Serial.readStringUntil("\n");
    
    // Capture the first character indicating the direction to move.
    readChar = data.charAt(0);

    // FOR COMMUNICATION COMMAND WITH SYNTAX "F1".
    // Capture the second integer in the string for number of steps to move.
    robot.distsub = data.substring(1).toInt();

    // FOR COMMUNICATION COMMAND WITH SYNTAX "F".
    // Comment the above line of code and uncomment this line below, or vice versa.
    //distsub = 1;
    
    // Robot has received a command and does not need to wait for further input.
    waitingInput = false;

    // Acknowledgement string to send back to the Raspberry Pi.
    Serial.print("ALG|MOV|"); Serial.println(readChar);
  }

  // SET PARAMETERS BASED ON INPUT COMMAND GIVEN. **********************************************************
  
  // Read the input command given.
  // NOTE: Decrementing of this value 'distsub' is done in the 
  // 'stopIfReached()' and 'stopIfRotated()' functions.
  switch(readChar)
  {
    // Move forward.
    case 'F': robot.forwards();

              // Need to change the command character to a non-functional character
              // So that the robot stops moving.
              readChar = ' ';
              waitingInput = true;
              break;

    // Rotate to the left by 90 degrees.
    case 'L': robot.rotate90left();
              readChar = ' ';
              waitingInput = true;
              break;

    // Rotate to the right by 90 degrees.
    case 'R': robot.rotate90right();
              readChar = ' ';
              waitingInput = true;
              break;

    // Rotate 180 degrees from the left.
    case 'B': robot.rotate180();
              readChar = ' ';
              waitingInput = true;
              break;
  }
}

void right_tick_increment()
{
  robot.right_tick_increment();
}

void left_tick_increment()
{
  robot.left_tick_increment();
}

// DIRECTION CONTROLLERS. *********************************************************************************

// If an input has already been given, it needs to be executed here.

// Rotating right 90 degrees.


// Rotating left 180 degrees.


// STOP MOVING OR ROTATING IF THE DISTANCE OR ANGLE HAS BEEN REACHED. *************************************

// If the robot is meant to go straight.

// If the robot is meant to rotate.


// PID CONTROLLER.*****************************************************************************************


// INTERRUPT, FAULT STOP AND MOTOR TICKS TO POWER FUNCTIONS.************************************************
