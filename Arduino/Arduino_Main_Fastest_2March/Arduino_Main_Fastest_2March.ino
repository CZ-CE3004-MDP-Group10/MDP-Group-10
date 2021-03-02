// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 ARDUINO FILE.
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

// PID, sensor and motor shield objects are created in the movement file.
Movement robot;

String receiveData; // Holds the entire string of commands received.
String subData;     // Holds the substring of an individual command extracted from the main string.
int count = 4;      // Counter to track the position of the command being read from the string.

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
  Serial.println("ALG|Robot Ready.");
}

// LOOPING.
void loop()
{
  // Check if data has been received at the serial link (USB).
  while(waitingInput)
  {
    // If the string is empty and there is serial data received.
    if(receiveData == NULL and Serial.available() > 0)
    {
      receiveData = Serial.readStringUntil("\n");
      //Serial.print("Data read in: "); Serial.println(receiveData);
      Serial.println("AND|status(Running)");
      continue;
    }
    // As long as the string is not empty, split it into individual commands.
    else if(receiveData != NULL)
    {
      // Delay here allowsone movement to stop before commencing another.
      delay(500);

      // Extract the next command from the main string.
      subData = receiveData.substring(count, count+2);
      //Serial.print("ALG|Movement Performed: "); Serial.println(subData);

      // Read the movement character command.
      readChar = subData.charAt(0);

      // If the character read is null, the end of the main string has been reached.
      if(readChar == '\n')
      {
        Serial.println("Reached end of command string.");

        // Clear all the contents of the main string and reset the counter.
        count = 4;
        receiveData = "";
        readChar = ' ';
        Serial.println("AND|status(Stopped)");
        continue;
      }

      // Extract the number of steps to move for the given direction command.
      robot.distsub = subData.substring(1).toInt();
      count += 2;
      
      // Robot has received a command and does not need to wait for further input.
      waitingInput = false;
  
      // Acknowledgement string to send back to the algorithm.
      Serial.print("ALG|MOV|"); Serial.print(readChar); Serial.println(robot.distsub);
    }
  
    // Read and execute the input command given.
    // NOTE: Decrementing of this value 'distsub' is done in the 'stopIfReached()' and 'stopIfRotated()' functions.
    switch(readChar)
    {
      // Move forward.
      case 'F': robot.forwards();
                // The robot needs to stop and wait for the next commmand after finishing its current one.
                waitingInput = true;

                // Acknowlegdement string to send to the Android to update the movement.
                Serial.print("AND|MOV("); Serial.print(readChar); Serial.print(")["); Serial.print(subData.substring(1).toInt()); Serial.println("]");
                
                // Perform movement corrections for tilt and forward position.
                robot.calibrate();

                // Reset the command character to break out of the switch case.
                readChar = " ";

                // Read and print the sensor values to detect obstacles in the surroundings, sending it to algorithm.
                robot.readSensor();
                break;
  
      // Rotate to the left by 90 degrees.
      case 'L': robot.rotate90left();
                waitingInput = true;
                Serial.print("AND|MOV("); Serial.print(readChar); Serial.println(")[1]");
                robot.calibrate();
                readChar = " ";
                //robot.readSensor();
                break;
  
      // Rotate to the right by 90 degrees.
      case 'R': robot.rotate90right();
                waitingInput = true;
                Serial.print("AND|MOV("); Serial.print(readChar); Serial.println(")[1]");
                robot.calibrate();
                readChar = " ";
                //robot.readSensor();
                break;
  
      // Rotate 180 degrees from the left.
      case 'B': robot.rotate180();
                waitingInput = true;
                Serial.print("AND|MOV("); Serial.print(readChar); Serial.println(")[1]");
                robot.calibrate();
                readChar = " ";
                //robot.readSensor();
                break;
    }
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
