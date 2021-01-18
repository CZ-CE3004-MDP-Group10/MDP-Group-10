// PROXIMITY SENSOR TESTING FILE.
// PRINTS ANALOG VALUE OUTPUTS TO THE CONSOLE.

// Integer variables to hold sensor analog values.
// Voltage values range: 0V to 5V, Analog values range: 0 to 1023.
int sensorA0 = 0;   // Proximity Sensor 1 on board -> Back Left Sensor.
int sensorA1 = 0;   // Proximity Sensor 2 on board -> Back Right Sensor.
int sensorA2 = 0;   // Proximity Sensor 3 on board -> Left Side Sensor.
int sensorA3 = 0;   // Proximity Sensor 4 on board -> Right Side Sensor.
int sensorA4 = 0;   // Proximity Sensor 5 on board -> Front Left Sensor.
int sensorA5 = 0;   // Proximity Sensor 6 on board -> Front Right Sensor.

// Float variables to hold the voltage value of the sensor outputs.
float voltageA0 = 0.0;
float voltageA1 = 0.0;
float voltageA2 = 0.0;
float voltageA3 = 0.0;
float voltageA4 = 0.0;
float voltageA5 = 0.0;

// Setup code runs once at startup.
void setup()
{
  //Send and receive serial monitor console data at rate of 9600 baud.
  Serial.begin(9600);
}

// Looping code runs continuously.
void loop()
{
  // Read the analog value of each sensor.
  // Convert that analog value into a voltage value.
  sensorA0 = analogRead(A0);
  voltageA0 = sensorA0 * (5.0 / 1023.0);
  sensorA1 = analogRead(A1);
  voltageA1 = sensorA1 * (5.0 / 1023.0);
  sensorA2 = analogRead(A2);
  voltageA2 = sensorA2 * (5.0 / 1023.0);
  sensorA3 = analogRead(A3);
  voltageA3 = sensorA3 * (5.0 / 1023.0);
  sensorA4 = analogRead(A4);
  voltageA4 = sensorA4 * (5.0 / 1023.0);
  sensorA5 = analogRead(A5);
  voltageA5 = sensorA5 * (5.0 / 1023.0);

  // Print the raw analog values and voltage values to the console.
  Serial.print("Raw values of PS1, PS2, PS3, PS4, PS5, PS6 -->> ");
  Serial.print(sensorA0); Serial.print(", ");
  Serial.print(sensorA1); Serial.print(", ");
  Serial.print(sensorA2); Serial.print(", ");
  Serial.print(sensorA3); Serial.print(", ");
  Serial.print(sensorA4); Serial.print(", ");
  Serial.print(sensorA5); Serial.print(", ");

  Serial.print("    Voltage values of PS1, PS2, PS3, PS4, PS5, PS6 -->> ");
  Serial.print(voltageA0); Serial.print(", ");
  Serial.print(voltageA1); Serial.print(", ");
  Serial.print(voltageA2); Serial.print(", ");
  Serial.print(voltageA3); Serial.print(", ");
  Serial.print(voltageA4); Serial.print(", ");
  Serial.print(voltageA5); Serial.print(", ");

  Serial.println("");
  delay(500);
}
