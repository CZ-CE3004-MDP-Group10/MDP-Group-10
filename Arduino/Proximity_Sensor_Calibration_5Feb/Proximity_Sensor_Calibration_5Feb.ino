// MANUAL PROXIMITY SENSOR CALIBRATION.

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
double distanceA0 = 0.0;
double distanceA1 = 0.0;
double distanceA2 = 0.0;
double distanceA3 = 0.0;
double distanceA4 = 0.0;
double distanceA5 = 0.0;

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
  // -----------------------------------------------------------------------------------
  // READ AND AVERAGE SENSOR VALUES.
  
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
    
    delay(50);
  }
  // Obtain the average of the values of each sensor.
  sensorA0_avg /= 20;
  sensorA1_avg /= 20;
  sensorA2_avg /= 20;
  sensorA3_avg /= 20;
  sensorA4_avg /= 20;
  sensorA5_avg /= 20;

  // -----------------------------------------------------------------------------------
  // EQUATIONS TO CONVERT THE ANALOG VALUES INTO CENTIMETERS:

  // SENSOR 1 CALCULATION AND OFFSETS:
  distanceA0 = -9.24678 + (8592.735 / (sensorA0_avg + 40.4344));

  // SENSOR 2 CALCULATION AND OFFSETS:
  distanceA1 = -3.53939 + (5891.966 / (sensorA1_avg - 11.84241));

  // SENSOR 3 CALCULATION AND OFFSETS:
  distanceA2 = -5.6782 + (6790.144 / (sensorA2_avg - 1.58589));

  // SENSOR 4 CALCULATION AND OFFSETS:
  distanceA3 = 0.252644 + (4894.633 / (sensorA3_avg - 26.90775));

  // SENSOR 5 CALCULATION AND OFFSETS:
  distanceA4 = 0.404528 + (5267.347 / (sensorA4_avg - 7.79982));

  // SENSOR 6 CALCULATION AND OFFSETS:
  distanceA5 = 0.33899 + (11687.6939 / (sensorA5_avg - 28.01154));

  // -----------------------------------------------------------------------------------
  // PRINT OUT ANALOG VALUES AND CALCULATED DISTANCES.
  Serial.print("Sensor A0: "); Serial.print(sensorA0_avg); Serial.print(", Distance A0: "); Serial.print(distanceA0);
  Serial.print("Sensor A1: "); Serial.print(sensorA1_avg); Serial.print(", Distance A1: "); Serial.print(distanceA1);
  Serial.print("Sensor A2: "); Serial.print(sensorA2_avg); Serial.print(", Distance A2: "); Serial.print(distanceA2);
  Serial.print("Sensor A3: "); Serial.print(sensorA3_avg); Serial.print(", Distance A3: "); Serial.print(distanceA3);
  Serial.print("Sensor A4: "); Serial.print(sensorA4_avg); Serial.print(", Distance A4: "); Serial.print(distanceA4);
  Serial.print("Sensor A5: "); Serial.print(sensorA5_avg); Serial.print(", Distance A5: "); Serial.print(distanceA5);

  // Reset the counter and variables.
  count = 0;
  sensorA0_avg = 0;
  sensorA1_avg = 0;
  sensorA2_avg = 0;
  sensorA3_avg = 0;
  sensorA4_avg = 0;
  sensorA5_avg = 0;
}
