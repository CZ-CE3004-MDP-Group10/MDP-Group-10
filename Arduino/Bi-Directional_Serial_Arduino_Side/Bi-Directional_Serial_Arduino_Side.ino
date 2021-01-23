void setup()
{
	// Baud rate may need to increase to 115200.
	Serial.begin(9600);
}

void loop()
{
	if (Serial.available() > 0)
	{
		String data = Serial.readStringUntil('\n');
		Serial.print("You sent me: ");
		Serial.println(data);
	}
}