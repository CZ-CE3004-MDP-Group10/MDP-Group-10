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

//int wholeCount = 0;

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

  // -----------------------------------------------------------------------------------
  // PRINT OUT ANALOG VALUES AND CALCULATED DISTANCES.

  //if(wholeCount < 20)
  //{
    //Serial.print(""); Serial.print(sensorA5_avg); Serial.print(", "); Serial.print("");
  //}

  //wholeCount += 1;
  
  Serial.print("Sensor A0: "); Serial.print(sensorA0_avg); Serial.print(", Distance A0: "); Serial.println(distanceA0);
  Serial.print("Sensor A1: "); Serial.print(sensorA1_avg); Serial.print(", Distance A1: "); Serial.println(distanceA1);
  Serial.print("Sensor A2: "); Serial.print(sensorA2_avg); Serial.print(", Distance A2: "); Serial.println(distanceA2);
  Serial.print("Sensor A3: "); Serial.print(sensorA3_avg); Serial.print(", Distance A3: "); Serial.println(distanceA3);
  Serial.print("Sensor A4: "); Serial.print(sensorA4_avg); Serial.print(", Distance A4: "); Serial.println(distanceA4);
  Serial.print("Sensor A5: "); Serial.print(sensorA5_avg); Serial.print(", Distance A5: "); Serial.println(distanceA5);

  // Reset the counter and variables.
  count = 0;
  sensorA0_avg = 0;
  sensorA1_avg = 0;
  sensorA2_avg = 0;
  sensorA3_avg = 0;
  sensorA4_avg = 0;
  sensorA5_avg = 0;
}
