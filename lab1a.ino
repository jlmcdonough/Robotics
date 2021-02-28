/*
 * Lab requirements:

Purchase robot
Setup Development Environment 
Flash "Beep" program
Make your robot beep using the Buzzer class
 */

#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;

//checkEncoders time
unsigned long currMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 20;

//checkEncoders wheels
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_DIAMETER = 3.2;
const float WHEEL_CIRCUMFERENCE = 10.0531;
long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;
float Sl = 0.0F;   //distance traveled by left wheel
float Sr = 0.0F;   //distance traveled by right wheel

//to run once
void setup()
{
  Serial.begin(57600);
  delay(1000);
  buzzer.play("c32");
}
  
//to run repeatedly
void loop() 
{
 /*
  if(buttonA.isPressed())
  {
    motors.setSpeeds(100, 100);
    Serial.println("Motors are running!");
  }
  else
  {
    motors.setSpeeds(0, 0);
  }
 */
 
  checkEncoders();
}

void checkEncoders()
{
  currMillis = millis();

  if(currMillis > (prevMillis + PERIOD) )
  {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    Sl += ( (countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE );
    Sr += ( (countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE );

    int wheelSpeed = 75;

    if(Sl < 300.5)
    {
      if(Sl > 200)
      {
        wheelSpeed = 75 * ( (300 - Sl) / 10 );
        if(wheelSpeed < 20)
        {
          wheelSpeed = 20;
        }
          buzzer.play("fs5");
      }
      motors.setSpeeds(wheelSpeed, wheelSpeed);
    }
    else
    {
      motors.setSpeeds(0, 0);
    }
    
    Serial.print("Left: ");
    Serial.print(Sl);
    Serial.print(" Right: ");
    Serial.println(Sr);

    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currMillis;
  }
}
