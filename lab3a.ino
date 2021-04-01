/*
Follow along with video to get your servo working.
  We are going to combine a servo with an ultrasonic sensor to enable our robot to “see” obstacles at various angles relative to the robot.  
  First we assemble the components using a breadboard.  We see how to use the battery power combined with a capacitor to ensure that the servo 
  has enough juice to move.  After completing the hardware build, we will dive into code to control the servos movements so that our Ultrasonic 
  sensor will be steady to get accurate readings.  In order to accomplish this, we must both stop the servo for the correct amount of time, but 
  also manage set angles that we are going to be taking the readings from to ensure we know where our obstacles are.  
 */

#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;
Servo headServo; 

//switches
const boolean HEAD_DEBUG = true;

//HEAD SERVO
//timing
unsigned long headCurrMillis;
unsigned long headPrevMillis;

//constants
const unsigned long HEAD_MOVEMENT_PERIOD = 300;
const int HEAD_SERVO_PIN = 0;
const int HEAD_POSITIONS_SIZE = 7;
const int HEAD_POSITIONS[HEAD_POSITIONS_SIZE] = {135, 120, 105, 90, 75, 60, 45};

//data
boolean headDirectionClockwise = true;
int currHeadPosition = 0;

void setup() 
{
  Serial.begin(57600);

  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(90);

  delay(3000);
  buzzer.play("c5");
}

void loop() 
{
  moveHead();
}

void moveHead()
{
  headCurrMillis = millis();

  if(headCurrMillis > (headPrevMillis + HEAD_MOVEMENT_PERIOD) )
  {
    if(HEAD_DEBUG)
    {
      Serial.print(currHeadPosition);
      Serial.print(": ");
      Serial.println(HEAD_POSITIONS[currHeadPosition]);
    }

    headServo.write(HEAD_POSITIONS[currHeadPosition]);

    if(headDirectionClockwise)
    {
      if(currHeadPosition >= (HEAD_POSITIONS_SIZE - 1) )
      {
        headDirectionClockwise = !headDirectionClockwise;
        currHeadPosition--;
      }
      else
      {
        currHeadPosition++;
      }
    }
    else
    {
      if(currHeadPosition <= 0)
      {
        headDirectionClockwise = !headDirectionClockwise;
        currHeadPosition++;
      }
      else
      {
        currHeadPosition--;
      }
    }
    
    headPrevMillis = headCurrMillis;
  }

}