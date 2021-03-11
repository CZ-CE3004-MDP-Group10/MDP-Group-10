// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 ARDUINO MAIN FILE.

// Include libraries. Movement library contains all other robot component libraries.
#include <EnableInterrupt.h>
#include <Movement.h>

// Motor M1 = Encoder E1 = Right.
// Motor M2 = Encoder E2 = Left.

// Motor encoder inputs.
#define encoder_M1_A 3   // Right motor output 1.
#define encoder_M1_B 5   // Right motor output 2. Not used.
#define encoder_M2_A 11  // Left motor output 1.
#define encoder_M2_B 13  // Left motor output 2. Not used.

// Variables for movement control when receiving commands for navigating around obstacles.
char readChar = ' ';            // Command character indicating the direction to move.
//boolean waitingInput = true;    // Determines if the robot is waiting for input from serial link.

// PID, sensor, motor shield, movement and correction objects are created in the movement file.
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
  Serial.println("ALG|Robot Ready.");
}

// LOOPING.
void loop()
{
  while(1)
  {
    // If the string is empty and there is serial data received via USB.
    if(receiveData == NULL and Serial.available() > 0)
    {
      receiveData = Serial.readStringUntil("\n");
      //Serial.print("Data read in: "); Serial.println(receiveData);

      // UNCOMMENT THIS LINE FOR FASTEST PATH.
      //Serial.println("AND|status(Running)");
      continue;
    }
    // As long as the string is not empty, split it into individual commands.
    else if(receiveData != NULL)
    {
      // Extract the next command from the main string.
      subData = receiveData.substring(count, count+2);

      // Check if the current command is the last command. 
	  // This is only applicable to forward commands.
      if(receiveData.charAt(count + 2) == '\0' or receiveData.charAt(count + 2) == '\n')
      {
        // Set the last command boolean.
        //Serial.println("Last command set.");
        robot.lastCommand = true;
      }

      // Read the movement character command.
      readChar = subData.charAt(0);

      // If the character read is null or newline, the end of the main string has been reached.
      if(readChar == '\n' or readChar == '\0')
      {
        // Clear all the contents of the main string and reset the last command boolean and counters.
        count = 4;
        receiveData = "";
        readChar = " ";
        robot.lastCommand = false;
		
		    // Need to insert a delay here for the Android application to separate the last string.
        delay(500);

        // UNCOMMENT THIS LINE FOR FASTEST PATH.
        //Serial.println("AND|status(Stopped)");
		
		    // Go back to wait for another input string command or set of commands.
        continue;
      }
      //Serial.print("Movement to perform: "); Serial.println(subData);

	    // If the command is not for calibration or sensor readings.
      if(readChar != 'C' and readChar != 'S')
      {
		    // Get the number of steps the robot should move in the particular direction.
        robot.distsub = subData.substring(1).toInt();

        // Acknowledgement string to send back to the algorithm.
        //Serial.print("ALG|MOV|"); Serial.print(readChar); Serial.println(robot.distsub);
      }
	  
	    // If the command is meant for calibration, check the second character of the substring.
      else if(readChar == 'C')
      {
		    // Perform front calibration.
        if(subData.charAt(1) == 'F')
        {
          readChar = 'W';
        }
		    // Perform right side calibration.
        else if(subData.charAt(1) == 'R')
        {
          readChar = 'D';
        }
      }

      // Extract the number of steps to move for the given direction command.
      robot.distsub = subData.substring(1).toInt();
      count += 2;
      
      // Robot has received a command and does not need to wait for further input.
      //waitingInput = false;
    }

	  // Set a delay between robot movements.
    delay(200);
  
    // Read and execute the input command given.
    // NOTE: Decrementing of this value 'distsub' is done in the 'stopIfReached()' and 'stopIfRotated()' functions.
    switch(readChar)
    {
      // Move forward.
      case 'F': robot.forwards();
                // Acknowlegdement string to send to the Android to update the movement.
                Serial.print("AND|MOV("); Serial.print(readChar); Serial.print(")["); Serial.print(subData.substring(1).toInt()); Serial.println("]");

                // Reset the command character to break out of the switch case.
                readChar = " ";
                break;
  
      // Rotate to the left by 90 degrees.
      case 'L': robot.rotate90left();
                Serial.print("AND|MOV("); Serial.print(readChar); Serial.println(")[1]");
                readChar = " ";
                break;
  
      // Rotate to the right by 90 degrees.
      case 'R': robot.rotate90right();
                Serial.print("AND|MOV("); Serial.print(readChar); Serial.println(")[1]");
                readChar = " ";
                break;
  
      // Rotate 180 degrees from the left.
      case 'B': robot.rotate180();
                Serial.print("AND|MOV("); Serial.print(readChar); Serial.println(")[1]");
                readChar = " ";
                break;

      // Calibrate by the front.
      case 'W': //robot.frontCalibrate();
                robot.frontWallCheckTilt();
                //delay(500);
                Serial.println("ALG|CF");
                readChar = " ";
                break;
                
      // Calibrate by the right.
      // NEED TO PERFORM THE RIGHT CALIBRATION 2 TIMES ON AVERAGE TO CORRECT THE TILT.
      case 'D': //robot.rightCalibrate();
                //delay(500);
                robot.rightWallDistCheck();
                Serial.println("ALG|CR");
                readChar = " ";
                break;
  
      // Command to return sensor data to algorithm.
      case 'S': robot.readSensor();
                robot.printSensor();

                // FOR DEBUGGING:
                //Serial.println("ALG|0,0,0,0,0,0");
                readChar = " ";
                break;

	  // If an invalid character is read.
      default:  readChar = " ";
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
