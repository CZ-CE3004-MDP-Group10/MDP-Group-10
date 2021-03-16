// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 ARDUINO FASTEST PATH MAIN FILE.

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
String receiveData;             // Holds the string for the command received.

// PID, sensor, motor shield, movement and correction objects are created in the movement file.
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
  //delay(1000);

  // Infinite while loop waits for the starting command from the algorithm.
  while(Serial.available() <= 0);

  // Clear out the whole buffer.
  readChar = Serial.read();
  delay(5);
  readChar = Serial.read();
  delay(5);
  readChar = Serial.read();
  delay(5);
  readChar = Serial.read();
  delay(5);
  readChar = Serial.read();
  delay(5);

  // Indicate to the algorithm that the robot is ready.
  //delay(1000);
  Serial.println("ALG|Robot Ready.");
  robot.readSensor();
  robot.printSensor();
}

// LOOPING.
void loop()
{
  // Reset the character and string commands.
  readChar = ' ';
  receiveData = " ";

  // Wait to receive a command.
  while(1)
  { 
    // If there is serial data received via USB.
    if(Serial.available() > 0)
    {
      Serial.read();
      delay(5);
      Serial.read();
      delay(5);
      Serial.read();
      delay(5);
      Serial.read();
      delay(5);
      readChar = Serial.read();
      break;
    }
  }
  robot.distsub = 1;
      
// ---------------------------------------------------------
  
    // Read and execute the input command given.
    // NOTE: Decrementing of this value 'distsub' is done in the 'stopIfReached()' and 'stopIfRotated()' functions.
    switch(readChar)
    {
      // Move forward.
      case 'F': robot.forwards();
                // Acknowledgement string to send to the Android to update the movement.
                Serial.print("AND|MOV("); Serial.print(readChar); Serial.print(")["); Serial.print(receiveData.substring(1).toInt()); Serial.println("]");
                //Serial.println("ALG|DMV");
                delay(10);

                // Return sensor data to the algorithm after each movement.
                robot.readSensor();
                robot.printSensor();
                break;

      // Rotate to the left by 90 degrees.
      case 'L': robot.rotate90left();
                Serial.print("AND|MOV("); Serial.print(readChar); Serial.println(")[1]");
                //Serial.println("ALG|DMV");
                delay(10);
                robot.readSensor();
                robot.printSensor();
                break;
  
      // Rotate to the right by 90 degrees.
      case 'R': robot.rotate90right();
                Serial.print("AND|MOV("); Serial.print(readChar); Serial.println(")[1]");
                //Serial.println("ALG|DMV");
                delay(10);
                robot.readSensor();
                robot.printSensor();
                break;
  
      // Rotate 180 degrees from the left.
      case 'B': robot.rotate180();
                Serial.print("AND|MOV("); Serial.print(readChar); Serial.println(")[1]");
                //Serial.println("ALG|DMV");
                delay(10);
                robot.readSensor();
                robot.printSensor();
                break;

      // Calibrate by the front for both distance and tilt.
      case 'W': robot.frontDistanceCheck();
                //delay(100);
                robot.frontTiltCheck();
                Serial.println("ALG|W");
                delay(50);
                robot.readSensor();
                robot.printSensor();
                break;
                
      // Calibrate by the right for both distance and tilt.
      case 'D': robot.rightDistanceCheck();
                //delay(100);
                robot.rightTiltCheck();
                Serial.println("ALG|D");
                delay(50);
                robot.readSensor();
                robot.printSensor();
                break;

	  // If an invalid character is received.
      default:  break;
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
