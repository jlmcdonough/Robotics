/*
 * develop a program that uses the ultrasonic sensor at 3 or more angles determined by servo position.
 * record sensor distances along the angles
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
const boolean HEAD_DEBUG = false;
const boolean TIMING_DEBUG = false;
const boolean US_DEBUG = true;

//HEAD SERVO
//pins 
const int HEAD_SERVO_PIN = 0;

//constants
const unsigned long HEAD_MOVEMENT_PERIOD = 120;
const int HEAD_POSITIONS_SIZE = 7;
const int HEAD_POSITIONS[HEAD_POSITIONS_SIZE] = {135, 120, 105, 90, 75, 60, 45};

//timing
unsigned long headCurrMillis;
unsigned long headPrevMillis;

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
  headServo.write(40);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  delay(1000);
  buzzer.play("c5");

  
}

void loop() 
{  
  moveHead();

  usReadCm();
}

void moveHead()
{
  headCurrMillis = millis();

  if(headCurrMillis > (headPrevMillis + HEAD_MOVEMENT_PERIOD) )
  {
    headServo.write(HEAD_POSITIONS[currHeadPosition]);
    currReadPosition = currHeadPosition;

    if(HEAD_DEBUG)
    {
      Serial.print(currHeadPosition);
      Serial.print(": ");
      Serial.println(HEAD_POSITIONS[currHeadPosition]);
    }
  
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
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 38000);
    float distance = duration * 0.034 / 2;
    distanceReadings[currReadPosition] = distance;    
    
    usPrevMillis = usCurrMillis;

    if(US_DEBUG)
    {
      //Serial.print("Duration: ");
      //Serial.print(duration);
      //Serial.print(" Distance: ");
      //Serial.print(distance);
      //Serial.println(" cm"); 
      Serial.print("Distance Readings List: [");
      for(int i = 0; i < HEAD_POSITIONS_SIZE - 1; i++)
      {
        Serial.print(distanceReadings[i]);
        Serial.print(", ");
      }
      Serial.print(distanceReadings[HEAD_POSITIONS_SIZE - 1]);
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