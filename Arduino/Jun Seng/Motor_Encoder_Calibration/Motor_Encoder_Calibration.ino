// Include the required libraries.
#include "DualVNH5019MotorShield.h"
#include "EnableInterrupt.h"

// Create an object for the motor shield.
DualVNH5019MotorShield md;

// Motor 1 parameters.
#define encoder1A 3
#define encoder1B 5

// Motor 2 parameters.
#define encoder2A 11
#define encoder2B 13

// How often the data is read.
#define SAMPLETIME 1

// Ticks Variables.
volatile long E1_ticks = 0;
volatile long E2_ticks = 0;

// Motor Speed Variables.
double M1_speed = 0;
double M2_speed = 0;
double M1_ticks_PID = 0;
double M2_ticks_PID = 0;

// Proportional Integral Derivative Variables.
double M1_setpoint_ticks = 0;
double M2_setpoint_ticks = 0;
double E1_error_ticks = 0;
double E2_error_ticks = 0;
double E1_prev_error = 0;
double E2_prev_error = 0;
double E1_sum_error = 0; //16919;
double E2_sum_error = 0; //17198;
double KP = 0.2;
double KD = 0.01;
double KI = 0.005;

// Set the desired RPM.
double targetRPM = 70;

void setup()
{
  // Initialize serial communication and motor shield.
  Serial.begin(115200);
  md.init();
  
  // Set motor encoder pins as input.
  // Read in pulse wave data from the first pin of each encoder.
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pulseIn(encoder1A, HIGH);

  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);
  pulseIn(encoder2A, HIGH);

  // Set the initial motor speeds based on calculated function below.
  md.setSpeeds(RPMtoSpeedM1(targetRPM),RPMtoSpeedM2(targetRPM));
  
  // Enable interrupts for each encoder input pin 1 on the rising edge.
  enableInterrupt(encoder1A, E1_ticks_increment, RISING);
  enableInterrupt(encoder2A, E2_ticks_increment, RISING);
}

void loop ()
{
  moveforward();
}

void moveforward()
{
  M1_setpoint_ticks = 200;
  M2_setpoint_ticks = 200;  
  PIDController();

  delay(1000);
}

// Perform Proportional Integral Derivative calculation.
//https://projects.raspberrypi.org/en/projects/robotPID/5
void PIDController()
{
  Serial.println("PID is running");
  Serial.print("Actual E1_ticks: ");
  Serial.print(E1_ticks);
  Serial.print("     E2_ticks: ");
  Serial.print(E2_ticks);
  
  E1_error_ticks = M1_setpoint_ticks - E1_ticks;
  E2_error_ticks = M2_setpoint_ticks - E2_ticks;
  
  M1_ticks_PID = E1_ticks + (E1_error_ticks * (KP * 0.95)) + (E1_prev_error * (KD + 0.01)) + (E1_sum_error * KI);
  
  M2_ticks_PID = E2_ticks + (E2_error_ticks * KP) + (E2_prev_error * KD) + (E2_sum_error * KI);

  //Convert adjusted ticks into RPM.
  M1_speed = RPMtoSpeedM1(ticks_to_rpm(M1_ticks_PID));
  M2_speed = RPMtoSpeedM2(ticks_to_rpm(M2_ticks_PID));

  // Set the new motor speeds.
  md.setSpeeds(M1_speed, M2_speed);
      
  Serial.print(", M1_speed: ");
  Serial.print(M1_speed);
  Serial.print("     M2_speed: ");
  Serial.println(M2_speed);
      
  //Reset the ticks.
  E1_ticks = 0;
  E2_ticks = 0;

  E1_prev_error = E1_error_ticks;
  E2_prev_error = E2_error_ticks;

  // Sum up all errors.
  E1_sum_error += E1_error_ticks;
  E2_sum_error += E2_error_ticks;
}

void E1_ticks_increment()
{
  E1_ticks ++;
}

void E2_ticks_increment()
{
  E2_ticks ++;
}

// Convert desired motor RPM to its commanded power.
double RPMtoSpeedM1(double rpm1 ){
  return 2.7037*rpm1 + 41.27;
}

// Convert desired motor RPM to its commanded power.
double RPMtoSpeedM2(double rpm2 ){
  return 2.6872*rpm2 + 34.236;
}

// Convert ticks into motor RPM.
double ticks_to_rpm(unsigned long tick){
  return (((double)tick*2/2249)/SAMPLETIME*1000000*60);
}
