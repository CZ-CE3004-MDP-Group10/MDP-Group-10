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

// Declare input pins, each encoder has 2 inputs.
byte R_encoder = 3;   // Right motor encoder.
byte L_encoder = 11;  // Left motor encoder.

// Variables to store the duration that each encoder square wave remains high.
int R_encoder_PWM = 0;
int L_encoder_PWM = 0;

float rightSpeed = 100;
float leftSpeed = rightSpeed * 1.13;

float timer = 0;
int state = 1;
int interval = 100;

// To calculate delta Time
float currentTime = 0;
float prevTime = 0;
float elapsedTime = 0;

// Number of revolution per wheels.
float LeftRPM = 0;
float RightRPM = 0;

// Total revolution
float TLrevo = 0;
float TRrevo = 0;

// current revolution in this dt period
float Lrevo = 0;
float Rrevo = 0;

// Set point ticks
int setPoint_ticks = 650;

double R_ticks_PID = 0;
double L_ticks_PID = 0;

double R_error_ticks = 0;
double L_error_ticks = 0;
double R_prev_error = 0;
double L_prev_error = 0;
double R_sum_error = 0; 
double L_sum_error = 0; 

double KP = 0.2;
double KD = 0.01;
double KI = 0.005;

// Integer variables to hold sensor analog values.
// Voltage values range: 0V to 5V, Analog values range: 0 to 1023.
int sensorB_Left = 0;   // Proximity Sensor 1 on board -> Back Left Sensor.
int sensorB_Right = 0;   // Proximity Sensor 2 on board -> Back Right Sensor.
int sensorLeft = 0;   // Proximity Sensor 3 on board -> Left Side Sensor.
int sensorRight = 0;   // Proximity Sensor 4 on board -> Right Side Sensor.
int sensorF_Left = 0;   // Proximity Sensor 5 on board -> Front Left Sensor.
int sensorF_Right = 0;   // Proximity Sensor 6 on board -> Front Right Sensor.

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

  // Initialize the inputs.
  pinMode(R_encoder,INPUT);
  pinMode(L_encoder,INPUT);

  // Initialize the motor shield driver object.
  motorShield.init();
}

// TRYING OUT PID CONSTANTS
/*
double kp = 2;
double ki = 5;
double kd = 1;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

double computePID(double setPoint , double inp)
{     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
*/

void updateSpeed()
{
  currentTime = millis();
  elapsedTime = currentTime - prevTime;
  
  R_encoder_PWM = pulseIn(R_encoder,HIGH);
  L_encoder_PWM = pulseIn(L_encoder,HIGH);

  prevTime = currentTime;

  Lrevo = ( elapsedTime * 1000 / ( (float)L_encoder_PWM * 2 ) ) / 562.25 ;
  Rrevo = ( elapsedTime * 1000 / ( (float)L_encoder_PWM * 2 ) ) / 562.25 ;

  TLrevo += Lrevo;
  TRrevo += Rrevo;

  LeftRPM = 1000 / elapsedTime * Lrevo * 60;
  RightRPM = 1000 / elapsedTime * Rrevo * 60;  

  Serial.print("Total Left Motor Revolution: ");
  Serial.print(TLrevo);
  Serial.print(", Total Right Motor Revolution: ");
  Serial.print(TRrevo);
  
  Serial.print("  Left Motor RPM: ");
  Serial.print(LeftRPM);
  Serial.print(", Right Motor RPM: ");
  Serial.println(RightRPM);

  R_error_ticks = setPoint_ticks - R_encoder_PWM;
  L_error_ticks = setPoint_ticks - L_encoder_PWM;
  
  R_ticks_PID = R_encoder_PWM + (R_error_ticks * (KP * 0.95)) + (R_prev_error * (KD + 0.01)) + (R_sum_error * KI);
  L_ticks_PID = L_encoder_PWM + (L_error_ticks * KP) + (L_prev_error * KD) + (L_sum_error * KI);

  //convert adjusted ticks to RPM
  leftSpeed = L_ticks_PID / 6.5;
  rightSpeed = R_ticks_PID / 6.5;

  
  R_prev_error = R_error_ticks;
  L_prev_error = L_error_ticks;

  R_sum_error += R_error_ticks;
  L_sum_error += L_error_ticks;
    

  
  /*
  if(encoderE2_PWM - encoderE1_PWM > 5)
  {
    if (encoderE1_PWM < encoderE2_PWM)
    {
      leftSpeed += 1;
      rightSpeed -= 1;
    }
  }
  else if(encoderE1_PWM - encoderE2_PWM > 15)
  {
    if (encoderE2_PWM < encoderE1_PWM)
    {
      leftSpeed -= 1;
      rightSpeed += 1;
    }
  }
  */

  /*
  if (encoderE1_PWM < encoderE2_PWM)
  {
    //(encoderE2_PWM - encoderE1_PWM)/encoderE1_PWM
    
    leftSpeed += (float)(encoderE2_PWM - encoderE1_PWM)/encoderE1_PWM * 2;
    rightSpeed -= (float)(encoderE2_PWM - encoderE1_PWM)/encoderE1_PWM * 2;
  }
    
  if (encoderE2_PWM < encoderE1_PWM)
  {
    leftSpeed -= (float)(encoderE1_PWM - encoderE2_PWM)/encoderE2_PWM * 2;
    rightSpeed += (float)(encoderE1_PWM - encoderE2_PWM)/encoderE2_PWM * 2;
  }
  */

  /*
  if(encoderE1_PWM != 0 and encoderE2_PWM != 0)
  {
    // Print the raw analog values and voltage values to the console.
    Serial.print("Raw values of PS1, PS2, PS3, PS4, PS5, PS6 -->> ");
    Serial.print(sensorB_Left); Serial.print(", ");
    Serial.print(sensorB_Right); Serial.print(", ");
    Serial.print(sensorLeft); Serial.print(", ");
    Serial.print(sensorRight); Serial.print(", ");
    Serial.print(sensorF_Left); Serial.print(", ");
    Serial.println(sensorF_Right);
  }
  */
  
}

void Forward(float dur)
{
  motorShield.setM1Speed(leftSpeed);
  motorShield.setM2Speed(rightSpeed);

  delay(dur);
}

void Backward(float dur)
{
  motorShield.setM1Speed(-leftSpeed);
  motorShield.setM2Speed(-rightSpeed);

  delay(dur);
}

void Left(float dur)
{
  motorShield.setM1Speed(-leftSpeed);
  motorShield.setM2Speed(rightSpeed);
  
  delay(65);
}

void Right(float dur)
{
  motorShield.setM1Speed(leftSpeed);
  motorShield.setM2Speed(-rightSpeed);
  
  delay(65);
}

void Stop(float dur = 0)
{
  motorShield.setM1Speed(0);
  motorShield.setM2Speed(0);

  delay(dur);
}

int SensorReact()
{
  // Read the analog value of each sensor.
  // Convert that analog value into a voltage value.
  sensorB_Left = analogRead(A0);
  sensorB_Right = analogRead(A1);
  sensorLeft = analogRead(A2);
  sensorRight = analogRead(A3);
  sensorF_Left = analogRead(A4);
  sensorF_Right = analogRead(A5);

  if(sensorF_Left > 550 || sensorF_Right > 580)
  {
    Serial.println("Front blocked");

    if(sensorLeft < 200)
    {
      Serial.println("Go left");

      state = 3;
    }
    else if (sensorRight < 215)
    {
      Serial.println("Go right");

      state = 4;
    }
    else if (sensorB_Left < 200 && sensorB_Right < 215)
    {
      Serial.println("Gostun");

      state = 2;
    }
    else
    {
      Serial.println("Cant move, stop");

      state = 0;
    }
    
  }

  if( state == 2 and sensorLeft < 160 )
  {
    Serial.println("Back to left");

    state = 3;
  }

  if( state == 2 and sensorRight < 170 )
  {
    Serial.println("Back to right");

    state = 4;
  }
  
}

// Looping code runs continuously.
void loop()
{
  /*
  Serial.print("Encoder E1 Right Motor PWM: ");
  Serial.print(encoderE1_PWM);
  Serial.print(", Encoder E2 Left Motor PWM: ");
  Serial.println(encoderE2_PWM);
  */
  //Serial.println(timer);

  timer += elapsedTime;
  
  updateSpeed(); 

  if(state != 3 and state != 4)
    SensorReact();
  
  
  if(timer > 1000)
  {
    if(state == 3 or state == 4)
    {
      state = 1;
    }
    timer = 0;
  }

  /*
  if(state > 3)
    state = 0;
  */
  
  switch(state)
  {
    case 0: // STOP MOVING.
      Stop(10);
      break;

    case 1: // FORWARD
      Forward(interval);
      break;

    case 2: // MOVE BACKWARDS. (Negative value.)
      Backward(interval);
      break;

    case 3: // STEER LEFT. (Left wheel backwards, right wheel forwards.)
      Left(interval);
      break;

    case 4: // STEER RIGHT. (Right wheel backwards, left wheel forwards.)
      Right(interval);
      break;
  }

  stopIfFault();
}
