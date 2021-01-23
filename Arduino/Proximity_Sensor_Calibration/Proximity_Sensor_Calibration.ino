// PROXIMITY SENSOR TESTING FILE.
// PRINTS THE AVERAGE ANALOG VALUE 10 READINGS PER SECOND FOR EACH SENSOR.

// Integer variables to hold sensor analog values.
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
double sensorA5_avg = 0.0;

// Setup code runs once at startup.
void setup()
{
  //Send and receive serial monitor console data at rate of 9600 baud.
  Serial.begin(115200);
}

// Looping code runs continuously.
void loop()
{
  for(count = 0; count < 10; count++)
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
    
    delay(100);
  }
  // Obtain the average of the values of each sensor.
  sensorA0_avg /= 10;
  sensorA1_avg /= 10;
  sensorA2_avg /= 10;
  sensorA3_avg /= 10;
  sensorA4_avg /= 10;
  sensorA5_avg /= 10;

  // Print the raw analog values and voltage values to the console.
  Serial.print("PS1: "); Serial.print(sensorA0); Serial.print(", ");
  Serial.print("PS2: "); Serial.print(sensorA1); Serial.print(", ");
  Serial.print("PS3: "); Serial.print(sensorA2); Serial.print(", ");
  Serial.print("PS4: "); Serial.print(sensorA3); Serial.print(", ");
  Serial.print("PS5: "); Serial.print(sensorA4); Serial.print(", ");
  Serial.print("PS6: "); Serial.print(sensorA5); Serial.println("");

  // Reset the counter and variables.
  count = 0;
  sensorA0_avg = 0;
  sensorA1_avg = 0;
  sensorA2_avg = 0;
  sensorA3_avg = 0;
  sensorA4_avg = 0;
  sensorA5_avg = 0;
}
