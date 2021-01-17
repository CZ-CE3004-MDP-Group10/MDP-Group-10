#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

// Stops the motor if there is a fault, like a short circuit.
void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();
}

void loop()
{
  /* 
   *  Motor M1 -> Left motor viewed from front.
   *  Motor M2 -> Right motor viewed from front.
   *  
   *  Left motor M1 is slightly slower than the right motor M2, hence its speed 
   *  value must be slightly higher. May be able to fix this issue using encoder.
  */

  // MOVE FORWARDS. (Positive value.)
  md.setM1Speed(107);
  md.setM2Speed(100);
  stopIfFault();
  delay(2000);

  // STOP MOVING.
  md.setM1Speed(0);
  md.setM2Speed(0);
  stopIfFault();
  delay(1000);

  // MOVE BACKWARDS. (Negative value.)
  md.setM1Speed(-107);
  md.setM2Speed(-100);
  stopIfFault();
  delay(2000);

  // STOP MOVING.
  md.setM1Speed(0);
  md.setM2Speed(0);
  stopIfFault();
  delay(1000);

  // STEER LEFT. (Left wheel backwards, right wheel forwards.)
  md.setM1Speed(-107);
  md.setM2Speed(100);
  stopIfFault();
  delay(2000);

  // STOP MOVING.
  md.setM1Speed(0);
  md.setM2Speed(0);
  stopIfFault();
  delay(1000);

  // STEER RIGHT. (Right wheel backwards, left wheel forwards.)
  md.setM1Speed(107);
  md.setM2Speed(-100);
  stopIfFault();
  delay(2000);

  // STOP MOVING.
  md.setM1Speed(0);
  md.setM2Speed(0);
  stopIfFault();
  delay(1000);
  
  // Debugging: Display the current consumption of each motor.
  /*Serial.print("M1 current: ");
  Serial.println(md.getM1CurrentMilliamps());

  Serial.print("M2 current: ");
  Serial.println(md.getM2CurrentMilliamps());*/

  // ----------------------------------------------------------------------------------
  // ORIGINAL PROGRAM CODE:
  /*for (int i = 0; i <= 400; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = 400; i >= -400; i--)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = -400; i <= 0; i++)
  {
    md.setM1Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M1 current: ");
      Serial.println(md.getM1CurrentMilliamps());
    }
    delay(2);
  }

  for (int i = 0; i <= 400; i++)
  {
    md.setM2Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = 400; i >= -400; i--)
  {
    md.setM2Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }
  
  for (int i = -400; i <= 0; i++)
  {
    md.setM2Speed(i);
    stopIfFault();
    if (i%200 == 100)
    {
      Serial.print("M2 current: ");
      Serial.println(md.getM2CurrentMilliamps());
    }
    delay(2);
  }*/
}
