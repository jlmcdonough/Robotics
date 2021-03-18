/*
The second part of the lab creates the object following behavior displayed in the second lab video.  
This lab is largely following along with the video, getting accustomed to working with sensor data and 
robot dynamics.  When you demo this lab in class on week 5 I will be looking for accuracy, smoothness and 
dynamics control.  I want you to show that you are comfortable using the sensor data and actuators to create 
a Braitenberg like behavior of wall following.  

 */

#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;

//distances
float distance = 0;
const float MAX_DISTANCE = 200; //max distance robot can see/cares about
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 10; //want to not get closer than  10cm from object, therefore max distance is 30.48 cm
const int rollingDistanceSize = 5;
static float rollingDistance[rollingDistanceSize];

//ULTRASONIC CONTROLS
//timing
unsigned long usCurrMillis;
unsigned long usPrevMillis;
const unsigned long US_PERIOD =  50;

//pins
const int ECHO_PIN = 30;
const int TRIG_PIN = 17;

//MOTOR CONTROLS
//constants
const float MOTOR_BASE_SPEED = 110.0;
const int MOTOR_MIN_SPEED = 30;
const int MOTOR_MAX_SPEED = 170; //strength of right wheel compared to left gets very choppy after 150 
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 125;  //allows for normalize a base motor speed


//timing
unsigned long motorCurrMillis;
unsigned long motorPrevMillis;
const unsigned long MOTOR_PERIOD = 50;

//tuning
const float L_MOTOR_FACTOR = 1.0;
const float R_MOTOR_FACTOR = .03;      //over the course of [30,170] the difference percentage is rightWheel>leftWheel goes from [3.6,6.8] --> (6.8-3.6)/(170-30)=.022857 per 1 speed.  Right wheel still pulls harder so rounded up to .03
const float R_MOTOR_BASE_DIFFERENCE = 3.6; //when speed is at the lowest (30), the right wheel is 3.6% stronger than the left wheel
const float L_MOTOR_FACTOR_THRESHOLD = 0;
const float R_MOTOR_FACTOR_THRESHOLD = 0;

void setup()
{
  Serial.begin(57600);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT); 
  
  delay(1000);
  buzzer.play("c32");
}
  

void loop() 
{
  usReadCm();

  setMotors();
}

void usReadCm()
{
  float sumDistance = 0;
  float avgDistance = 0;
  usCurrMillis = millis();

  if(usCurrMillis > (usPrevMillis + US_PERIOD) )
  {

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 38000);

    distance = duration * 0.034 / 2;

    float * currDistance = shiftDistance(rollingDistance, distance);

    for(int i = 0; i < 5; i++)
    {
      sumDistance += currDistance[i];
    }

    avgDistance = sumDistance / 5;
    distance = avgDistance;
    
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm"); 

    for (int j = 0; j < 5; j++)   //updates rollingDistance array with the latest distance reading
    {
      rollingDistance[j] = currDistance[j];
    }
  
    usPrevMillis = usCurrMillis;
  }
}

void setMotors()
{
  motorCurrMillis = millis();

  if(motorCurrMillis > (motorPrevMillis + MOTOR_PERIOD) )
  {

    float leftSpeed = MOTOR_BASE_SPEED;
    float rightSpeed = MOTOR_BASE_SPEED;

    if(distance <= MAX_DISTANCE)
    {
      float magnitude = (float)(MAX_DISTANCE - distance) / DISTANCE_FACTOR;
      Serial.print("MAGNITUDE: ");
      Serial.println(magnitude);
      leftSpeed = MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR);
      rightSpeed = rightWheelAdjustment(leftSpeed);
    }


    if(leftSpeed < MOTOR_MIN_SPEED)
    {
      leftSpeed = MOTOR_MIN_SPEED;
    }

    if(rightSpeed < MOTOR_MIN_SPEED)
    {
      rightSpeed = MOTOR_MIN_SPEED;
      rightSpeed = rightWheelAdjustment(rightSpeed);
    }

    if(leftSpeed > MOTOR_MAX_SPEED)
    {
      leftSpeed = MOTOR_MAX_SPEED;
    }

    if(rightSpeed > MOTOR_MAX_SPEED)
    {
      rightSpeed = MOTOR_MAX_SPEED;
      rightSpeed = rightWheelAdjustment(rightSpeed);
    }

    if(distance <= STOP_DISTANCE)
    {
      leftSpeed = 0;
      rightSpeed = 0;
    }

    Serial.print("LEFT: ");
    Serial.print(leftSpeed);
    Serial.print(" RIGHT: ");
    Serial.println (rightSpeed); 

    motors.setSpeeds(leftSpeed, rightSpeed);

  /*  if(rightSpeed > (leftSpeed + 15) )
    {
      buzzer.play("A1"); 
    }

    else if(rightSpeed > (leftSpeed + 10) )
    {
      buzzer.play("D1"); 
    }

    else if(rightSpeed > (leftSpeed + 5) )
    {
      buzzer.play("G1"); 
    }
    else if(rightSpeed > leftSpeed)
    {
      buzzer.play("CS5");
    } */

    motorPrevMillis = motorCurrMillis;
  }
}

float rightWheelAdjustment(float speed)
{
  float temp = speed - 30;                                 //the beginning is at speed 30, so by subtracting 30 from the actual speed and, can treat 30 speed as 0 (0 is bottom, but 30 is lowest speed, so 30 is bottom)
  float thisDiff = temp * R_MOTOR_FACTOR;                  //adjust the multipler for the current speed
  float totalDiff = thisDiff + R_MOTOR_BASE_DIFFERENCE;    //need to add the base difference since at the lowest speed, there already exists a difference in wheel strength
  float percentage = 100 - totalDiff;                      //concerned about the percentage stronger the right wheel is to the left wheel
  float adjustment = percentage / 100;                     //turn the adjustment into a number less than 1 (i.e. its percentage) such that the code can tell the right wheel to operate at a discounted value in comparison to the left wheel
  float adjustedSpeed = speed * adjustment;               //set the right wheel to its discounted speed compared to left wheel
  return adjustedSpeed;

 /* Serial.print("TEMP: ");
  Serial.println(temp);
    Serial.print("thisDiff: ");
  Serial.println(thisDiff);
     Serial.print("totalDiff: ");
  Serial.println(totalDiff);
     Serial.print("percentage: ");
  Serial.println(percentage);
     Serial.print("adjustment: ");
  Serial.println(adjustment);
        Serial.print("adjustedSpeed: ");
  Serial.println(adjustedSpeed); */

  /*
   * For speed is 110, this is a walkthrough of how to calculate right wheel's new speed
   * temp = 110 - 30 --> temp = 80
   * thisDiff = 80 * .022857 --> thisDiff = 1.82856   (means that right wheel is 1.82856% strogner than what it was at base speed, not over left wheel)
   * totalDiff = 1.82856 + 3.6 --> totalDiff = 5.42856 (here shows how much stronger the right wheel is compared to left wheel i.e. right wheel is 5.43% stronger than the left)
   * percentage = 100 - 5.42856 --> percentage = 94.57144 (right wheel needs to operate at 94.57144% the speed of the left wheel so it can go close to straight)
   * adjustment = 94.57144 / 100 --> adjustment = .9457144  (.9457144 is the multipler for the right wheel to discount its speed)
   * adjustmentSpeed = 110 * .9457144 --> adjustmentSpeed = 104.028584 (right wheel's new speed is 104.028584)
   * 
   * While the left wheel is spinning at 110, the right wheel is spinning at 104.028584, which is 94.57% the speed.  
   * By doing such, this should help the left wheel keep the pace and allow the robot to get much closer to a straight line.
   */
}


float * shiftDistance(float arr[], float dist)  //shifts old distance down one and replaces oldest one with current distance
    {
      static float shiftedArray[5];

      for (int j = 0; j < 4; j++)
      {
        shiftedArray[j] = arr[j+1];
      }
      
      shiftedArray[4] = dist;
      return shiftedArray;
    }
