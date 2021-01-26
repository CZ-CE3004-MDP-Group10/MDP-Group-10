char incomingByte = ' ';

void setup()
{
	// Baud rate may need to increase to 115200.
	Serial.begin(115200);
}

void loop()
{
	if (Serial.available() > 0)
	{
    // Read the incoming byte.
    incomingByte = Serial.read();

    if(incomingByte != '\n')
    {
      Serial.print("I received: ");
      Serial.println(incomingByte);
    }
    // Read the incoming string.
		//String data = Serial.readStringUntil('\n');
	}
}
