/*
 * Your completed robot for lab 1b will be able to demonstrate the following:

    Ability to control motor speed
    Ability to move and stop at precise distances 
    Ability to move forward and backwards
    I want to see graceful movement, smooth de-acceleration (bonus for adding smooth acceleration as well!)
    Your Grade will be based on the following:
    
    Code submission (as text in iLearn or attached .ino file)
    Presentation: Robot must complete the following maneuvers 
    Drive forward 12 inches
    Drive backwards 12 inches
    Drive forwards 18 inches.
    Beep when complete
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

//distances - in cm (1 in = 2.54cm)
int destination = 0;    //indicates starting distance
const int listDistSize = 3;
float listDist[listDistSize] = {30.48, -30.48, 45.72};  //negative indicates motors to flip and go backwards

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
  checkEncoders();
}

void checkEncoders()
{
  currMillis = millis();

  if(currMillis > (prevMillis + PERIOD) )
  {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    //taking the absolute value of the difference in counts allows for one formula regardless if distance is forwards or backwards
    Sl += ( abs(countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE );
    Sr += ( abs(countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE );

    //more destinations to go, therefore keep controlling motors
    if(destination < listDistSize ) 
    {
      controlMotors();
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

//function does the controlling of the motors (i.e. setting speed, wheel direction)
void controlMotors()
{
  int wheelSpeed = 60;  //max speed
  float thisDist = listDist[destination];  //get the distance needed to travel 

  //if distance is negative, must go backwards - else go forwards
  if(thisDist < 0)     
  {
    motors.flipLeftMotor(true);
    motors.flipRightMotor(true);
    thisDist = abs(thisDist);
  }
  else                
  {
    motors.flipLeftMotor(false);
    motors.flipRightMotor(false);
  }  

  //so long as there is more distance needing to be travelled, check current distance to determine wheel speed
  if(Sl < thisDist)
  {
    if(Sl > thisDist - 10) //gives it 10cm to slow down
    {
      wheelSpeed = 60 * ( (thisDist - Sl) / 10 );
    }
    else if(Sl < 10)   //gradually accelerates to its top speed within first 10 cm
    {
      wheelSpeed = 60 * ( Sl / 10 );
    }

    //if wheelSpeed dips below 30, chance of stalling so must re-up speed
    if(wheelSpeed < 30)
    {
      wheelSpeed = 30;
    }

    Serial.print("WHEEL SPEED: ");
    Serial.println(wheelSpeed);
         
    motors.setSpeeds(wheelSpeed, wheelSpeed);
  }
  else
  {
    resetMotors();
    delay(500);
    destination += 1;
    //upon completing all traversing all distances, beep to let user know it is done
    if(destination == listDistSize )
    {
      buzzer.play("fs2");
    }
      
    Serial.print("DESTINATION #: ");
    Serial.print( destination );
    Serial.print(" DISTANCE: ");
    Serial.println(listDist[destination]); 
   }
}

//function resets the changing values to default so the next distance can be accurately traversed
void resetMotors()
{
  motors.setSpeeds(0, 0);
  countsLeft = 0;
  countsRight = 0;
  Sl = 0;
  Sr = 0;
}
