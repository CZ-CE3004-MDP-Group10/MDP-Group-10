// MULTIDISCIPLINARY DESIGN PROJECT SEMESTER 2 YEAR 20-21 GROUP 10 ARDUINO FILE.

// LIBRARIES.***********************************************************************************************
#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>

// DEFINITIONS.*********************************************************************************************

// Motor M1 = Encoder E1 = Right.
// Motor M2 = Encoder E2 = Left.

// Motor encoder inputs
#define encoder_M1_A 3   // Right motor output 1.
#define encoder_M1_B 5   // Right motor output 2. Not used.
#define encoder_M2_A 11  // Left motor output 1.
#define encoder_M2_B 13  // Left motor output 2. Not used.

// VARIABLES.***********************************************************************************************

// Integer variables to hold sensor analog values.
// Voltage values range: 0V to 5V, Analog integer values range: 0 to 1023.
int sensorA0 = 0;   // Proximity Sensor 1 on board - FRONT LEFT SENSOR.
int sensorA1 = 0;   // Proximity Sensor 2 on board - FRONT MIDDLE SENSOR.
int sensorA2 = 0;   // Proximity Sensor 3 on board - FRONT RIGHT SENSOR.
int sensorA3 = 0;   // Proximity Sensor 4 on board - RIGHT SIDE FRONT SENSOR.
int sensorA4 = 0;   // Proximity Sensor 5 on board - RIGHT SIDE BACK SENSOR.
int sensorA5 = 0;   // Proximity Sensor 6 on board - FACING LEFT LONG RANGE SENSOR.
int count = 0;

// Float variables to hold the voltage value of the sensor outputs.
double sensorA0_avg = 0.0;
double sensorA1_avg = 0.0;
double sensorA2_avg = 0.0;
double sensorA3_avg = 0.0;
double sensorA4_avg = 0.0;
double sensorA5_avg = 0.0;

// Float variables to hold the converted distance based on sensor analog value.
double distanceA0 = 0.0;    // FRONT LEFT SENSOR.
double distanceA1 = 0.0;    // FRONT MIDDLE SENSOR.
double distanceA2 = 0.0;    // FRONT RIGHT SENSOR.
double distanceA3 = 0.0;    // RIGHT SIDE FRONT SENSOR.
double distanceA4 = 0.0;    // RIGHT SIDE BACK SENSOR.
double distanceA5 = 0.0;    // FACING LEFT LONG RANGE SENSOR.

// Motor commanded power, set the initial motor speed here.
// Note that right motor is slightly faster than left motor by default.
double right_speed = 0.0;
double left_speed = 0.0;

// Tick counters.
int right_ticks = 0;
int left_ticks = 0;
double M1_ticks_moved = 0;
double M2_ticks_moved = 0;
double M1_ticks_diff = 0;
double M2_ticks_diff = 0;
double Total_M1_moved = 0;
double Total_M2_moved = 0;
double M1_ticks_to_move = 0;    // Individual number of ticks to move is specified in each scenario.
double M2_ticks_to_move = 0;
double M1_setpoint_ticks = 5;   // Number of ticks before each iteration of PID controller, also controls motor speed.
double M2_setpoint_ticks = 5;

// Proportional Integral Derivative (PID) controller.
double right_ticks_PID = 0.0;
double left_ticks_PID = 0.0;
double M1_ticks_PID = 0;
double M2_ticks_PID = 0;
double E1_error_ticks = 0;
double E2_error_ticks = 0;
double E1_prev_error = 0;
double E2_prev_error = 0;
double E1_sum_error = 0;
double E2_sum_error = 0;
double KP = 0.2;        // Adjust for proportional component.
double KD = 0.3;        // Adjust for derivative component.
double KI = 7;          // Adjust for integral component.

// For movement control when receiving commands for navigating around obstacles.
char readChar = 'F';            // Command character indicating the direction to move.
boolean waitingInput = true;    // Determines if the robot is waiting for input from serial link.

// Set the distance travelled with each step and the angle of rotation here.
int Front_Back_ticks = 260;     // 260 ticks moves the robot forward by approximately 10cm.
int Right_ticks = 385;          // Theoretically, 398 ticks rotates the robot by approximately 90 degrees.
int Left_ticks = 385;           // In reality, 385 ticks are needed to rotate the robot by approximately 90 degrees.

// CREATE OBJECTS.******************************************************************************************

// Create an object for the motor shield.
DualVNH5019MotorShield motorShield;

// SETUP - RUNS ONCE.***************************************************************************************
void setup()
{
  //Send and receive serial monitor console data at rate of 115200 baud.
  Serial.begin(115200);

  // Initialize the motor shield driver object.
  motorShield.init();

  // Set the encoder pins as input.
  pinMode(encoder_M1_A, INPUT);
  pinMode(encoder_M1_B, INPUT);
  pinMode(encoder_M2_A, INPUT);
  pinMode(encoder_M2_B, INPUT);

  // Enable interrupts for each encoder input pin 1 on the rising edge.
  // Function 'right_tick_increment" is called on rising edge of input 'encoder_M1_A'.
  enableInterrupt(encoder_M1_A, right_tick_increment, RISING);
  enableInterrupt(encoder_M2_A, left_tick_increment, RISING);

  // Multiply the number of ticks by 4 to get a full rotation.
  // But still need to under compensate to achieve exact 360 degrees.
  M1_ticks_to_move = 1580;   // Right motor.
  M2_ticks_to_move = 1580;   // Left motor.

  // Introduce an initial delay to prevent power up surges from interfering.
  delay(3000);

  // Send initial string of sensor readings when the robot is ready and initialized.
  readSensor();
    
  Serial.println("ALG|RDY|" + String(distanceA0) + "," + String(distanceA1) + "," + String(distanceA2)
  + "," + String(distanceA3) + "," + String(distanceA4) + "," + String(distanceA5));
}

// LOOPING - RUNS CONTINUOUSLY.*****************************************************************************

boolean start45 = false;
int steps_moved_45 = 0;
boolean turnLeft45 = false;
int numOfTurns45 = 0;

void loop()
{
  // READ FROM SERIAL.**************************************************************************************
  
  // Check if data has been received at the serial link (USB).
  while(waitingInput ) // and Serial.available() > 0)
  {
    // Read up to the entire string that is passed in up to the newline character.
    //String data = Serial.readStringUntil("\n");

    // ************************************* START OF 45 DEGREE CODE **************************************
    if(start45 == true)
    {
      if( distanceA0 > 30 && distanceA1 > 30 && distanceA2 > 30 && steps_moved_45 < 5)
      {
        readChar = 'F';
        steps_moved_45 += 1;
        Serial.println("In 45 mode, nth infront move forward");
      }
      else if ( steps_moved_45 == 0 && ( distanceA0 < 30 || distanceA1 < 30 || distanceA2 < 30 ) )
      {
        readChar = 'R';
        turnLeft45 = false;
        Serial.println("Cant move here, Turn Right instead");
      }

      if(steps_moved_45 >= 5 && numOfTurns45 == 0)
      {
        Left_ticks *= 2;
        Right_ticks *= 2;
        
        if(turnLeft45)
        {
          readChar = 'R';
          turnLeft45 = false;
        }
        else
        {
          readChar = 'L';
          turnLeft45 = true;
        }
        steps_moved_45 = 0;
        numOfTurns45 += 1;

        Serial.println("Turn other way");
      }

      if(steps_moved_45 >= 5 && numOfTurns45 == 1)
      {
        if(turnLeft45)
        {
          readChar = 'R';
          turnLeft45 = false;
        }
        else
        {
          readChar = 'L';
          turnLeft45 = true;
        }

        numOfTurns45 += 1;
        //start45 = false;
        Right_ticks /= 2;
        Left_ticks /= 2;
        
        Serial.println("Turn other way and end");
      }
      else if ( numOfTurns45 == 2)
      {
        Right_ticks *= 2;
        Left_ticks *= 2;

        start45 = false;
        numOfTurns45 = 0;
        steps_moved_45 = 0;
      }
      
      
    }

    if(start45 == false)
    {
      if( distanceA0 > 20 && distanceA1 > 20 && distanceA2 > 20)
      {
        readChar = 'F';
        Serial.println("Nothing, move forward");
      }
      else
      {
        start45 = true;
        Right_ticks /= 2;
        Left_ticks /= 2;
        readChar = 'L';
        turnLeft45 = true;
        Serial.println("Found obstacle , Turn Left");
      }
    }

    //********************************************** END OF 45 DEGREE CODE *****************************

    
    
    // Capture the first character in a variable, remaining characters are ignored.
    //readChar = data.charAt(0);

    // Robot has received a command and does not need to wait for further input.
    waitingInput = false;

    // Acknowledgement string to send back to the Raspberry Pi.
    Serial.print("ALG|MOV|"); Serial.println(readChar);
  }

  // SET PARAMETERS BASED ON INPUT COMMAND GIVEN. **********************************************************
  
  // Read the input command given.
  switch(readChar)
  {
    // Move forward.
    case 'F': forwards();
              break;

    // Rotate to the left by 90 degrees.
    case 'L': rotate90left();
              break;

    // Rotate to the right by 90 degrees.
    case 'R': rotate90right();
              break;

    // Rotate 180 degrees from the left.
    case 'B': rotate180();
              break;
  }
}

// DIRECTION CONTROLLERS. *********************************************************************************

// If an input has already been given, it needs to be executed here.
// Moving forwards.
void forwards()
{
  while(!waitingInput)
  {
    M1_ticks_to_move = Front_Back_ticks - M1_ticks_diff;
    M2_ticks_to_move = Front_Back_ticks - M2_ticks_diff;

    // Both motors have positive speed values.
    PID(1,1);

    // Stop the motor movement if the required distance or step is reached.
    // Step distance and rotation angle are set at the top of this file.
    stopIfReached();
  }
  // Insert delay here between steps if necessary.
  //delay(2000);
}

// Rotating left 90 degrees.
void rotate90left()
{
  while(!waitingInput)
  {
    M1_ticks_to_move = (Right_ticks) - M1_ticks_diff;
    M2_ticks_to_move = (Left_ticks) - M2_ticks_diff;

    // Left motor is negative and right motor is positive.
    PID(-1,1);

    // When rotating, the number of ticks for both motors is observed to fluctuate
    // Anywhere from 5 to 15 ticks more than the set value at the top of this file.
    stopIfRotated();
  }
  delay(2000);
}

// Rotating right 90 degrees.
void rotate90right()
{
  while(!waitingInput)
  {
    M1_ticks_to_move = (Right_ticks) - M1_ticks_diff;
    M2_ticks_to_move = (Left_ticks) - M2_ticks_diff;

    // Left motor is positive and right motor is negative.
    PID(1,-1);
    stopIfRotated();
  }
  delay(2000);
}

// Rotating left 180 degrees.
void rotate180()
{
  while(!waitingInput)
  {
    // Need to insert offsets to improve accuracy.
    M1_ticks_to_move = (Right_ticks * 2) - M1_ticks_diff;
    M2_ticks_to_move = (Left_ticks * 2) - M2_ticks_diff;

    // Left motor is negative and right motor is positive.
    PID(-1,1);
    stopIfRotated();
  }
  delay(2000);
}

// STOP MOVING OR ROTATING IF THE DISTANCE OR ANGLE HAS BEEN REACHED. *************************************

// If the robot is meant to go straight.
void stopIfReached()
{
  // Once the desired number of ticks to move the required distance has been reached.
  if(M1_ticks_moved > M1_ticks_to_move and M2_ticks_moved > M2_ticks_to_move)
  {
    // Error difference for how many ticks the current step exceeded by.
    M1_ticks_diff = M1_ticks_moved - M1_ticks_to_move;
    M2_ticks_diff = M2_ticks_moved - M2_ticks_to_move;
      
    // For debugging, two master counters count the total number of ticks moved through
    // the entire motor operation to check for tick discrepancies over time.
    Total_M1_moved += M1_ticks_moved;
    Total_M2_moved += M2_ticks_moved;
      
    //Serial.print("R ticks moved : "); Serial.print(M1_ticks_moved);
    //Serial.print(", L ticks moved : "); Serial.println(M2_ticks_moved);
    //Serial.print(", Total right ticks moved : "); Serial.print(Total_M1_moved);
    //Serial.print(", Total left ticks moved : "); Serial.println(Total_M2_moved);

    // Set the brakes on both motors simultaneously to bring the robot to a stop.
    // Syntax: motorShield.setBrakes(M1 right motor, M2 left motor);
    motorShield.setBrakes(400, 400);

    // Reset the tick counters.
    M1_ticks_moved = 0;
    M2_ticks_moved = 0;

    // When the robot stops moving, read in sensor data.
    readSensor();

    // Return acknowledgement string that the robot has stopped moving, and the readings in cm from all sensors.
    Serial.println("ALG|DMV|" + String(distanceA0) + "," + String(distanceA1) + "," + String(distanceA2)
    + "," + String(distanceA3) + "," + String(distanceA4) + "," + String(distanceA5));

    // The robot needs to wait for another input command before continuing.
    waitingInput = true;

    // Set the 'readChar' movement command variable to a dummy value to prevent further movement,
    // Until another command is received from the serial.
    //readChar = ' ';
  }
}

// If the robot is meant to rotate.
void stopIfRotated()
{
  if(M1_ticks_moved > M1_ticks_to_move and M2_ticks_moved > M2_ticks_to_move)
  {
    // NOTE: ERROR IN TICKS DIFFERENCE IS NOT NEEDED HERE AS EACH ROTATION IS INDEPENDENT.
      
    // For debugging, two master counters count the total number of ticks moved through
    // the entire motor operation to check for tick discrepancies over time.
    Total_M1_moved += M1_ticks_moved;
    Total_M2_moved += M2_ticks_moved;
      
    //Serial.print("R ticks moved : "); Serial.print(M1_ticks_moved);
    //Serial.print(", L ticks moved : "); Serial.print(M2_ticks_moved);
    //Serial.print(", Total right ticks moved : "); Serial.print(Total_M1_moved);
    //Serial.print(", Total left ticks moved : "); Serial.println(Total_M2_moved);
  
    // Stop the robot movement, braking is more effective then setting the speed to 0.
    motorShield.setBrakes(400,400);

    // Reset the tick counters.
    M1_ticks_moved = 0;
    M2_ticks_moved = 0;

    // Get sensor readings for any obstacles detected and their distances around the starting point.
    readSensor();

    // Return acknowledgement string that the robot has stopped moving, and the readings in cm from all sensors.
    Serial.println("ALG|DMV|" + String(distanceA0) + "," + String(distanceA1) + "," + String(distanceA2)
    + "," + String(distanceA3) + "," + String(distanceA4) + "," + String(distanceA5));

    // The robot needs to wait for another input command before continuing.
    waitingInput = true;

    // Set the 'readChar' movement command variable to a dummy value to prevent further movement,
    // Until another command is received from the serial.
    //readChar = ' ';
  }
  // NOTE: THIS DELAY IS CRUCIAL TO ENSURING THE ACCURACY OF ROTATION ANGLE.
  // A SMALLER DELAY VALUE MAKES ROTATION FASTER BUT MORE LIKELY TO LOSE ANGLE ACCURACY.
  delay(5);
}

// PID CONTROLLER.*****************************************************************************************

// Input multipliers determine the rotation direction of each wheel.
void PID(int right_mul , int left_mul)
{
  // Sum up the total number of ticks per iteration of PID computation.
  M1_ticks_moved += right_ticks;
  M2_ticks_moved += left_ticks;

  // Determine the error difference in the number of ticks.
  E1_error_ticks = M1_setpoint_ticks - right_ticks;
  E2_error_ticks = M2_setpoint_ticks - left_ticks;

  // NOTE: PID CALCULATION 'KP', 'KI', 'KD' VALUES NEED TO BE MANUALLY TUNED FOR EACH MOTOR.
  
  // Perform PID calculation for the new motor speed for the right motor.
  M1_ticks_PID = right_ticks + (E2_error_ticks * KP) + (E2_prev_error * KD) + (E2_sum_error * KI * 1.19);

  // Perform PID calculation for the new motor speed for the left motor.
  M2_ticks_PID = left_ticks + (E1_error_ticks * KP * 0.95) + (E1_prev_error * (KD + 0.2)) + (E1_sum_error * KI * 0.98);

  // For debugging.
  //Serial.print(", Right (M1) PID ticks: "); Serial.print(M1_ticks_PID);
  //Serial.print(", Left (M2) PID ticks: "); Serial.println(M2_ticks_PID);

  // Convert the adjusted ticks to the new motor power using the functions below.
  right_speed = right_ticks_to_power(M1_ticks_PID);
  left_speed = left_ticks_to_power(M2_ticks_PID);
  
  // Set both motors speed and direction with the multiplier simultaneously.
  Syntax: // motorShield.setSpeeds(M1Speed (right), M2Speed (left));
  motorShield.setSpeeds(right_speed * right_mul, left_speed * left_mul);
  stopIfFault();

  // For debugging.
  //Serial.print(", R power: "); Serial.print(right_speed);
  //Serial.print(", L power: "); Serial.println(left_speed);

  // Reset the tick count for the next iteration after the PID computation is performed.
  right_ticks = 0;
  left_ticks = 0;

  // Store the last error value for the next computation.
  E1_prev_error = E1_error_ticks;
  E2_prev_error = E2_error_ticks;

  // Store the sum of all errors. Errors with negative values are subtracted from the sum.
  E1_sum_error += E1_error_ticks;
  E2_sum_error += E2_error_ticks;

  // Perform the PID computation again after a sampling time.
  delay(10);
}

// READING OF SENSOR VALUES. ******************************************************************************

void readSensor()
{
  for(count = 0; count < 20; count++)
  {
    // Read the analog value of each sensor and accumulate them for averaging.
    sensorA0 = analogRead(A0);
    sensorA0_avg += sensorA0;
    sensorA1 = analogRead(A1);
    sensorA1_avg += sensorA1;
    sensorA2 = analogRead(A2);
    sensorA2_avg += sensorA2;
    sensorA3 = analogRead(A3);
    sensorA3_avg += sensorA3;
    sensorA4 = analogRead(A4);
    sensorA4_avg += sensorA4;
    sensorA5 = analogRead(A5);
    sensorA5_avg += sensorA5;
  }
  
  // Obtain the average of the values of each sensor.
  sensorA0_avg /= 20;
  sensorA1_avg /= 20;
  sensorA2_avg /= 20;
  sensorA3_avg /= 20;
  sensorA4_avg /= 20;
  sensorA5_avg /= 20;

  // EQUATIONS TO CONVERT THE ANALOG VALUES INTO CENTIMETERS:

  // SENSOR 1 CALCULATION AND OFFSETS:   READINGS DIFFERENT FROM EXCEL
  distanceA0 = -9.1042 + (8155.745 / (sensorA0_avg + 22.11502));
  if(distanceA0 < 15) {distanceA0 += 2;}
  else if(distanceA0 > 15 and distanceA0 < 30) {distanceA0 -= 1;}
  else if(distanceA0 > 30 and distanceA0 < 45) {distanceA0 -= 2;}
  else if(distanceA0 > 50 and distanceA0 < 55) {distanceA0 += 1;}
  else if(distanceA0 > 65 and distanceA0 < 70) {distanceA0 += 2;}

  // SENSOR 2 CALCULATION AND OFFSETS:
  distanceA1 = -3.53939 + (5891.966 / (sensorA1_avg - 11.84241));
  if(distanceA1 < 10) {distanceA1 += 1;}
  else if(distanceA1 > 15 and distanceA1 < 20) {distanceA1 += 1;}

  // SENSOR 3 CALCULATION AND OFFSETS:
  distanceA2 = 1.41294 + (4269.218 / (sensorA2_avg - 28.92149));
  if(distanceA2 > 10 and distanceA2 < 40) {distanceA2 += 1;}
  else if(distanceA2 > 55 and distanceA2 < 60) {distanceA2 += 1;}
  else if(distanceA2 > 65) {distanceA2 += 2;}
  
  // SENSOR 4 CALCULATION AND OFFSETS:
  distanceA3 = 0.252644 + (4894.633 / (sensorA3_avg - 26.90775));
  if(distanceA3 > 70) {distanceA3 -= 3;}
  else if(distanceA3 > 80) {distanceA3 -= 2;}

  // SENSOR 5 CALCULATION AND OFFSETS:
  distanceA4 = 0.404528 + (5267.347 / (sensorA4_avg - 7.79982));
  if(distanceA4 < 13) {distanceA4 -= 1;}
  else if(distanceA4 > 50 and distanceA4 < 70) {distanceA4 -= 2;}

  // SENSOR 6 CALCULATION AND OFFSETS:
  distanceA5 = -3.3012 + (12806.428 / (sensorA5_avg - 9.81909));
  if(distanceA5 < 25) {distanceA5 -= 1;}
  else if(distanceA5 > 25 and distanceA5 < 45) {distanceA5 += 1;}
  else if(distanceA5 > 75 and distanceA5 < 80) {distanceA5 -= 1;}

  if(distanceA0 < 0) { distanceA0 = 100; }
  if(distanceA1 < 0) { distanceA1 = 100; }
  if(distanceA2 < 0) { distanceA2 = 100; }
  if(distanceA3 < 0) { distanceA3 = 100; }
  if(distanceA4 < 0) { distanceA4 = 100; }
  if(distanceA5 < 0) { distanceA5 = 100; }
  
  // Reset the counter and variables.
  count = 0;
  sensorA0_avg = 0;
  sensorA1_avg = 0;
  sensorA2_avg = 0;
  sensorA3_avg = 0;
  sensorA4_avg = 0;
  sensorA5_avg = 0;
}

// INTERRUPT, FAULT STOP AND MOTOR TICKS TO POWER FUNCTIONS.************************************************

// Increment the number of ticks at the rising edge of the right motor encoder square wave.
void right_tick_increment()
{
  right_ticks++;
}

// Increment the number of ticks at the rising edge of the left motor encoder square wave.
void left_tick_increment()
{
  left_ticks++;
}

// Based on obtained data of ticks moved vs set power for right motor,
// Convert right motor ticks to commanded power.
double right_ticks_to_power(double right_ticks)
{
  return (right_ticks * 0.22748) + 20.06839;
}

// Based on obtained data of ticks moved vs set power for left motor,
// Convert left motor ticks to commanded power.
double left_ticks_to_power(double left_ticks)
{
  return (left_ticks * 0.221907) + 31.70289;
}

// Stops the motor if there is a fault, like a short circuit.
void stopIfFault()
{
  if (motorShield.getM1Fault())
  {
    Serial.println("Left motor fault.");

    // Infinite loop stops the program from continuing.
    while(1);
  }
  if (motorShield.getM2Fault())
  {
    Serial.println("Right motor fault.");

    // Infinite loop stops the program from continuing.
    while(1);
  }
}
