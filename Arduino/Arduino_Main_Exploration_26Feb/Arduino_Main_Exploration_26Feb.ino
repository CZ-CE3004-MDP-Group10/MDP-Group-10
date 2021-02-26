// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 ARDUINO FILE.
// EXPLORATION AND IMAGE RECOGNITION NAVIGATION MAIN FILE.

// CAUTION: MOTOR MOVEMENT DISTANCES, ROTATION ANGLES AND SPEEDS ARE AFFECTED BY REMAINING BATTERY CAPACITIY.
// ENSURE BATTERY IS FULLY CHARGED BEFORE STARTING NAVIGATION.
// IMPORTANT: ALL SENSORS AND MOTORS MUST BE RECALIBRATED BEFORE ANY EVALUATION OR NAVIGATION.

// Include libraries.
//#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include <Movement.h>
// PID, sensor and motor shield libraries are included in the movement library.

// Motor M1 = Encoder E1 = Right.
// Motor M2 = Encoder E2 = Left.

// Motor encoder inputs.
#define encoder_M1_A 3   // Right motor output 1.
#define encoder_M1_B 5   // Right motor output 2. Not used.
#define encoder_M2_A 11  // Left motor output 1.
#define encoder_M2_B 13  // Left motor output 2. Not used.

// Variables for movement control when receiving commands for navigating around obstacles.
char readChar = ' ';            // Command character indicating the direction to move.
boolean waitingInput = true;    // Determines if the robot is waiting for input from serial link.
String data;

// PID, sensor and motor shield objects are created in the movement file.
Movement robot;

// SETUP.
void setup()
{
  //Send and receive serial monitor console data at rate of 115200 baud.
  Serial.begin(115200);

  // PID, sensor and motor shield objects are initialized in the movement file.
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
  delay(2000);  
  
  // Send initial string of sensor readings when the robot is ready and initialized.
  robot.readSensor();
}

// LOOPING.
void loop()
{ 
  // Check if data has been received at the serial link (USB).
  while(waitingInput and Serial.available() > 0)
  {
    // Read up to the entire string that is passed in up to the newline character.
    data = Serial.readStringUntil("\n");
    
    // Check if the first 4 characters are "ARD|", indicating they are for Arduino.
    if(data.substring(0,4) == "ARD|")
    {
		// Take only the substring starting from the 5th character onwards.
		data = data.substring(4);
	}
    
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
  
  // Read and execute the input command given.
  // NOTE: Decrementing of this value 'distsub' is done in the 'stopIfReached()' and 'stopIfRotated()' functions.
  switch(readChar)
  {
    // Move forward.
    case 'F': robot.forwards();
              // The robot needs to stop and wait for the next commmand after finishing its current one.
              waitingInput = true;
              Serial.print("AND|MOV("); Serial.print(readChar); Serial.print(")["); Serial.print(data.substring(1).toInt()); Serial.println("]");
              // Insert while loop here for fixed number of iterations for tilt checking.
              robot.calibrate();
              readChar = " ";
              break;

    // Rotate to the left by 90 degrees.
    case 'L': robot.rotate90left();
              waitingInput = true;
              Serial.print("AND|MOV("); Serial.print(readChar); Serial.println(")[1]");
              robot.calibrate();
              readChar = " ";
              break;

    // Rotate to the right by 90 degrees.
    case 'R': robot.rotate90right();
              waitingInput = true;
              Serial.print("AND|MOV("); Serial.print(readChar); Serial.println(")[1]");
              robot.calibrate();
              readChar = " ";
              break;

    // Rotate 180 degrees from the left.
    case 'B': robot.rotate180();
              waitingInput = true;
              Serial.print("AND|MOV("); Serial.print(readChar); Serial.println(")[1]");
              robot.calibrate();
              readChar = " ";
              break;
  }
}

// INTERRUPT HANDLERS TO INCREMENT THE MOTOR ENCODER TICK COUNTERS.
void right_tick_increment()
{
  robot.right_tick_increment();
}

void left_tick_increment()
{
  robot.left_tick_increment();
}
