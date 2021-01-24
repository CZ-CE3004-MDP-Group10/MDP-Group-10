// Include library for Sharp Infrared sensors.
// Can use either the library that takes the mean or median value.
#include <SharpIR.h>

// FOR MANUAL CALIBRATION.

/*// Integer variables to hold sensor analog values.
// Voltage values range: 0V to 5V, Analog integer values range: 0 to 1023.
int sensorA0 = 0;   // Proximity Sensor 1 on board -> Back Left Sensor.
int sensorA1 = 0;   // Proximity Sensor 2 on board -> Back Right Sensor.
int sensorA2 = 0;   // Proximity Sensor 3 on board -> Left Side Sensor.
int sensorA3 = 0;   // Proximity Sensor 4 on board -> Right Side Sensor.
int sensorA4 = 0;   // Proximity Sensor 5 on board -> Front Left Sensor.
int sensorA5 = 0;   // Proximity Sensor 6 on board -> Front Right Sensor.
int count = 0;

// Float variables to hold the voltage value of the sensor outputs.
double sensorA0_avg = 0.0;
double sensorA1_avg = 0.0;
double sensorA2_avg = 0.0;
double sensorA3_avg = 0.0;
double sensorA4_avg = 0.0;
double sensorA5_avg = 0.0;*/

// -----------------------------------------------------------------------------------
// SHARP IR SENSOR LIBRARY CALIBRATION.
// Define IR sensor input pins.
#define PS1_pin A0
#define PS2_pin A1
#define PS3_pin A2
#define PS4_pin A3
#define PS5_pin A4
#define PS6_pin A5

// Define IR sensor models, both large and small are used.
#define large_model 20150
#define small_model 1080

/* Model and code:
  GP2Y0A02YK0F --> 20150 (our large model)
  GP2Y0A21YK0F --> 1080 (our small model)
  GP2Y0A710K0F --> 100500
  GP2YA41SK0F --> 430
*/

// Create variable to store the distance, all in centimetres.
int PS1_distance;
int PS2_distance;
int PS3_distance;
int PS4_distance;
int PS5_distance;
int PS6_distance;

// Create a new instance of the SharpIR class for each small and large sensor.
SharpIR PS1 = SharpIR(PS1_pin, small_model);  // Back left sensor.
SharpIR PS2 = SharpIR(PS2_pin, small_model);  // Back right sensor.
SharpIR PS3 = SharpIR(PS3_pin, small_model);  // Left sensor.
SharpIR PS4 = SharpIR(PS4_pin, small_model);  // Right sensor.
SharpIR PS5 = SharpIR(PS5_pin, large_model);  // Front left sensor.
SharpIR PS6 = SharpIR(PS6_pin, large_model);  // Front right sensor.

// -----------------------------------------------------------------------------------

// Setup code runs once at startup.
void setup()
{
  //Send and receive serial monitor console data at rate of 9600 baud.
  Serial.begin(115200);
}

// Looping code runs continuously.
void loop()
{
  // CODE FOR CALIBRATION USING SHARP IR SENSOR LIBRARY.
  
  // Sharp IR sensor library code helps to obtain the average of each sensor's readings.
  // May wish to add in buffer distance to each measured value to reduce collision likelihood.
  PS1_distance = PS1.distance() + 2;    // Need to +2cm for improved accuracy.
  PS2_distance = PS2.distance() + 1;    // Need to +1cm for improved accuracy.
  PS3_distance = PS3.distance() + 2;    // Need to +2cm for improved accuracy.
  PS4_distance = PS4.distance() + 2;    // Need to +2cm for improved accuracy.
  PS5_distance = PS5.distance() + 1;    // Need to +1cm +more as distance increases for improved accuracy.
  PS6_distance = PS6.distance() + 2;    // Need to +2cm +more as distance increases for improved accuracy.

  // Print out the calculated distance values in centiemtres.
  Serial.print("Back left PS1: "); Serial.print(PS1_distance); Serial.print("cm, ");
  Serial.print("Back right PS2: "); Serial.print(PS2_distance); Serial.print("cm, ");
  Serial.print("Left PS3: "); Serial.print(PS3_distance); Serial.print("cm, ");
  Serial.print("Right PS4: "); Serial.print(PS4_distance); Serial.print("cm, ");
  Serial.print("Front left PS5: "); Serial.print(PS5_distance); Serial.print("cm, ");
  Serial.print("Front right PS6: "); Serial.print(PS6_distance); Serial.println("cm");

  delay(500);

  // -----------------------------------------------------------------------------------
  // CODE FOR MANUAL CALIBRATION.
  /*for(count = 0; count < 20; count++)
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
    
    delay(50);
  }
  // Obtain the average of the values of each sensor.
  sensorA0_avg /= 20;
  sensorA1_avg /= 20;
  sensorA2_avg /= 20;
  sensorA3_avg /= 20;
  sensorA4_avg /= 20;
  sensorA5_avg /= 20;

  // Print the raw analog values and voltage values to the console.
  Serial.print("Back left PS1: "); Serial.print(sensorA0); Serial.print(", ");
  Serial.print("Back right PS2: "); Serial.print(sensorA1); Serial.print(", ");
  Serial.print("Left PS3: "); Serial.print(sensorA2); Serial.print(", ");
  Serial.print("Right PS4: "); Serial.print(sensorA3); Serial.print(", ");
  Serial.print("Front left PS5: "); Serial.print(sensorA4); Serial.print(", ");
  Serial.print("Front right PS6: "); Serial.print(sensorA5); Serial.print("");

  // Reset the counter and variables.
  count = 0;
  sensorA0_avg = 0;
  sensorA1_avg = 0;
  sensorA2_avg = 0;
  sensorA3_avg = 0;
  sensorA4_avg = 0;
  sensorA5_avg = 0;*/
}
