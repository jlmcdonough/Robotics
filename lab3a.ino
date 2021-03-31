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

//DEBUGGING
const boolean HEAD_DEBUG = true;
const boolean TIMING_DEBUG = false;
const boolean US_DEBUG = true;

//HEAD SERVO
//timing
unsigned long headCurrMillis;
unsigned long headPrevMillis;

//constants
const unsigned long HEAD_MOVEMENT_PERIOD = 150;
const int HEAD_SERVO_PIN = 0;
const int HEAD_POSITIONS_SIZE = 7;
const int HEAD_POSITIONS[HEAD_POSITIONS_SIZE] = {135, 120, 105, 90, 75, 60, 45};

//data
boolean headDirectionClockwise = true;
int currHeadPosition = 0;


//ULTRASONIC
//pins
const int ECHO_PIN = 30;
const int TRIG_PIN = 17;

//constants
const float MAX_DISTANCE = 300.0;
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 30;

//timing
unsigned long usCurrMillis;
unsigned long usPrevMillis;
const unsigned long US_PERIOD =  80;
boolean usReadFlag = false;

//data
int currReadPosition = currHeadPosition;
float distanceReadings[HEAD_POSITIONS_SIZE] = {MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE};

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
 // moveHead();

  usReadCm();
}

void moveHead()
{
  headCurrMillis = millis();

  if(headCurrMillis > (headPrevMillis + HEAD_MOVEMENT_PERIOD) )
  {

    Serial.println("MOVING HEAD");
    headServo.write(HEAD_POSITIONS[currHeadPosition]);
    Serial.print("CURR HEAD: ");
    Serial.println(currHeadPosition);
    currReadPosition = currHeadPosition;
    
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

void usReadCm()
{
  usCurrMillis = millis();

  if(usCurrMillis > (usPrevMillis + US_PERIOD) )
  {
    Serial.println("TAKING READING");
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 38000);
    float distance = duration * 0.034 / 2;
    Serial.print("CURR READ: ");
    Serial.println(currReadPosition);
    distanceReadings[currReadPosition] = distance;    
    
    usPrevMillis = usCurrMillis;

    if(US_DEBUG)
    {
      Serial.print("Duration: ");
      Serial.print(duration);
      Serial.print(" Distance: ");
      Serial.print(distance);
      Serial.println(" cm"); 
      Serial.print("Distance List: [");
      for(int i = 0; i < HEAD_POSITIONS_SIZE - 1; i++)
      {
        Serial.print(i);
        Serial.print(" ");
        Serial.print(distanceReadings[i]);
        Serial.print(", ");
      }
      Serial.print(HEAD_POSITIONS_SIZE - 1);
      Serial.print(" ");
      Serial.print(distanceReadings[HEAD_POSITIONS_SIZE]);
      Serial.println("]");
    }
    
    if(TIMING_DEBUG)
    {
      Serial.print("MOVE HEAD START TIME: ");
      Serial.println(headCurrMillis);
      Serial.print("US START TIME: ");
      Serial.println(usCurrMillis);
      Serial.print("US RECEIVED TIME: ");
      Serial.println(duration);
    }
  }
}
