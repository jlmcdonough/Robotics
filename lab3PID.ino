/*
 * develop a program that uses the ultrasonic sensor at 3 or more angles determined by servo position.
 * record sensor distances along the angles
 * 
 * We are going to apply our knowledge with an implementation of the PID controller that will solve an 
 * interesting problem, keeping our robot smoothly following walls in a room.  The robot should be able 
 * to deal with changes in room layout, wall angles, and random obstacles it might encounter, without 
 * prior knowledge of the environment.  It should also act in a natural and efficient manner.  Moving at 
 * a good pace and not appearing to need to stop or slow necessarily to deal with obstacles.  
 */

#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;
ButtonC buttonC;
Servo headServo; 

//DEBUGGING
const boolean HEAD_DEBUG = false;
const boolean US_DEBUG = false;
const boolean TIMING_DEBUG = false;
const boolean MOTOR_DEBUG = false;
const boolean PID_DEBUG = false;

//HEAD SERVO
//pins 
const int HEAD_SERVO_PIN = 0;

//constants
const unsigned long HEAD_MOVEMENT_PERIOD = 220;
const int HEAD_POSITIONS_SIZE = 4;
const int HEAD_POSITIONS_LEFT[HEAD_POSITIONS_SIZE] = {170, 170, 170, 90}; 
const int HEAD_POSITIONS_RIGHT[HEAD_POSITIONS_SIZE] = {10, 10, 10, 90}; 
int HEAD_POSITIONS[HEAD_POSITIONS_SIZE] = {10, 10, 10, 90};  //default head position to left, but with buttons can change it

//timing
unsigned long headCurrMillis;
unsigned long headPrevMillis;

//data
int currHeadPosition = 0;
bool sensorDirectionRight = true;  //sensor faces right by default

//ULTRASONIC
//pins
const int ECHO_PIN = 30;
const int TRIG_PIN = 17;

//constants
const float MAX_DISTANCE = 100.0;
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 30;

//timing
unsigned long usCurrMillis;
unsigned long usPrevMillis;
const unsigned long US_PERIOD =  80;

//data
int currReadPosition = currHeadPosition;
int prevReadPosition = currReadPosition - 1;
int prevPosition = currReadPosition - 1;
float distanceReadings[HEAD_POSITIONS_SIZE] = {STOP_DISTANCE, STOP_DISTANCE, STOP_DISTANCE, STOP_DISTANCE};
float distance = 0;
float duration = 0;
float thisRoundDistance = 0;


//MOTORS
//constants
const float LEFT_MOTOR_BASE_SPEED = 35.0;  
const float RIGHT_MOTOR_BASE_SPEED = 35.0;
const float MOTOR_MIN_SPEED = 30.0;
const float MOTOR_MAX_SPEED = 130.0;

//timing
unsigned long motorCurrMillis;
unsigned long motorPrevMillis;
const unsigned long MOTOR_PERIOD = 50;

//data
float leftSpeed = LEFT_MOTOR_BASE_SPEED;
float rightSpeed = RIGHT_MOTOR_BASE_SPEED;


//PID Controller
//constants
const float Kp = .2;
const float Ki = .0005;
const float Kd = 3;
const float PID_UPPER_LIMIT = 10000;    //so high that it is irrelevant
const float PID_LOWER_LIMIT = -10000;   //so low that it is irrelevant
const float INTEGRAL_MAX = 1;
const float INTEGRAL_MIN = 1;

//timing
unsigned long pidCurrMillis;
unsigned long pidPrevMillis;

//data
float totalError = 0;  //used for integral
float prevError = 0;   //used for derivative
float pid = 0;

void setup() 
{
  Serial.begin(57600);

  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(90);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  
  Serial.println("3");
  buzzer.play("a5");  
  delay(1000);
  Serial.println("2");
  buzzer.play("b5");
  delay(2000);

  //choose direction of which way for sensor to face
  if(buttonA.isPressed())    //user wants sensors to the left
  {
    HEAD_POSITIONS[0] = 170;
    HEAD_POSITIONS[1] = 170;
    HEAD_POSITIONS[2] = 170;    
    sensorDirectionRight = false;
  }

  if(buttonC.isPressed())   //user wants sensors to the left
  {
    HEAD_POSITIONS[0] = 10;
    HEAD_POSITIONS[1] = 10;
    HEAD_POSITIONS[2] = 10;  
    sensorDirectionRight = true;
  }
  
  Serial.println(HEAD_POSITIONS[0]);

  Serial.println("1");
  buzzer.play("c5");
  headServo.write(HEAD_POSITIONS[0]);
  delay(3000);
  Serial.println("GOGOGOGO");
}

void loop() 
{  
  moveHead();

  usReadCm();

  setMotors();
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
  
    if(currHeadPosition == (HEAD_POSITIONS_SIZE - 1) )
    {
      currHeadPosition = 0;
    }
    else
    {
      currHeadPosition++;
    }

    
    headPrevMillis = headCurrMillis; 
  }
}

void usReadCm()
{
  usCurrMillis = millis();

  if(usCurrMillis > (usPrevMillis + HEAD_MOVEMENT_PERIOD)) 
  {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    distance = duration * 0.034 / 2;

    if(distance > MAX_DISTANCE)
    {
      distance = MAX_DISTANCE;
    }

    distanceReadings[currReadPosition] = distance;

    //HEAD_POSITIONS_SIZE - 1 is the last element and that is the forward facing direction, which has its own controller, therefore everything less than last is accounted in this pid controller
    //when the second to last element is reached, the readings for this cycle of pid is exhausted and is calculated at this time
        //take average of all the sensor readings to try and get an accurate reading of how far from wall
    if(currReadPosition == HEAD_POSITIONS_SIZE - 2)
    {
      thisRoundDistance += distance;
      float avgDistance = thisRoundDistance / (HEAD_POSITIONS_SIZE - 1); 

      thisRoundDistance = 0;
      
      computePID(avgDistance);
    }
    //no pid controller for front facing yet, just want it to stop and turn
    else if(currReadPosition == HEAD_POSITIONS_SIZE - 1)
    {
       if(distance < 30 && distance > 0)  //sometimes sesnor will read a 0.0 which is due to sensor inaccuracy and will cause an necessary turn.  An actual object will render a distance of at least .1
       {
          if(sensorDirectionRight)
          {
            turnLeft();
          }
          if(!sensorDirectionRight)
          {
            turnRight();
          }
       }
    }
    else
    {
      thisRoundDistance += distance;
    }

  
    usPrevMillis = usCurrMillis;

    prevReadPosition = currReadPosition;
    

    if(US_DEBUG)
    {
      Serial.print("Duration: ");
      Serial.print(duration);
      Serial.print(" Distance: ");
      Serial.print(distance);
      Serial.println(" cm"); 
    
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
      Serial.println(duration * .001); //want duration in milliseconds, recorded in microseconds
      Serial.print("US END TIME: ");
      Serial.println(usCurrMillis + (duration * .001) );
   }
  }
}


void setMotors()
{
  motorCurrMillis = millis();

  if(motorCurrMillis > (motorPrevMillis + HEAD_MOVEMENT_PERIOD) )
  {

    //SENSOR FACING RIGHT
    if(pid < 0  && sensorDirectionRight)  //too close to wall on right -> strengthen right wheel
    {
      leftSpeed = LEFT_MOTOR_BASE_SPEED - abs(pid);
      rightSpeed = RIGHT_MOTOR_BASE_SPEED + abs(pid);
    }

    if(pid > 0  && sensorDirectionRight) //too far to wall on right -> strengthen left wheel
    {
      leftSpeed = LEFT_MOTOR_BASE_SPEED + abs(pid);
      rightSpeed = RIGHT_MOTOR_BASE_SPEED - abs(pid);
    }

    //SENSOR FACING LEFT
    if(pid < 0  && !sensorDirectionRight)  //too close to wall on left -> strengthen left wheel
    {
      leftSpeed = LEFT_MOTOR_BASE_SPEED + abs(pid);
      rightSpeed = RIGHT_MOTOR_BASE_SPEED - abs(pid);
    }

    if(pid > 0  && !sensorDirectionRight) //too far to wall on left -> strengthen right wheel
    {
      leftSpeed = LEFT_MOTOR_BASE_SPEED - abs(pid);
      rightSpeed = RIGHT_MOTOR_BASE_SPEED + abs(pid);
    }

    if(leftSpeed > MOTOR_MAX_SPEED)
    {
      leftSpeed = MOTOR_MAX_SPEED;
    }
    if(rightSpeed > MOTOR_MAX_SPEED)
    {
      rightSpeed = MOTOR_MAX_SPEED;
    }

    if(leftSpeed < MOTOR_MIN_SPEED)
    {
      leftSpeed = MOTOR_MIN_SPEED;
    }
    if(rightSpeed < MOTOR_MIN_SPEED)
    {
      rightSpeed = MOTOR_MIN_SPEED;
    }
    
    motors.setSpeeds(leftSpeed, rightSpeed);

    motorPrevMillis = motorCurrMillis;

    if(MOTOR_DEBUG)
    {
      if(pid > 0 && sensorDirectionRight)
        Serial.println("DECREASING RIGHT");
      if(pid < 0 && sensorDirectionRight)
        Serial.println("INCREASING RIGHT");

      if(pid > 0 && !sensorDirectionRight)
        Serial.println("DECREASING LEFT");
      if(pid < 0 && !sensorDirectionRight)
        Serial.println("INCREASING LEFT");
              
      Serial.print(" LEFT SPEED: ");
      Serial.print(leftSpeed);
      Serial.print("RIGHT SPEED: ");
      Serial.println(rightSpeed);

    }
  }
}

float computePID(float distance)
{
    pidCurrMillis = millis();
  
    float error = distance - STOP_DISTANCE;
  
    float p = computeProportional(error);
    float i = computeIntegral(error);
    float d = computeDerivative(error);
  
    pid = p + i + d;

    if(pid > PID_UPPER_LIMIT)
    {
      pid = PID_UPPER_LIMIT;
    }
    if(pid < PID_LOWER_LIMIT)
    {
      pid = PID_LOWER_LIMIT;
    }
    
    pidPrevMillis = pidCurrMillis;
  
    if(PID_DEBUG)
    {
      Serial.print("DISTANCE: ");
      Serial.print(distance);
      Serial.print(" ERROR: ");
      Serial.println(error);
      Serial.print("P: ");
      Serial.print(p);
      Serial.print(" I: ");
      Serial.print(i);
      Serial.print(" D: ");
      Serial.print(d);
      Serial.print(" PID: ");
      Serial.println(pid);
    }
}

float computeProportional(float error)
{
  float proportional = Kp * error;
  
  return proportional;
}

float computeIntegral(float error)
{
  totalError += error * (pidCurrMillis - pidPrevMillis);
  float integral = Ki * totalError;

  if(integral > INTEGRAL_MAX)
  {
    integral = INTEGRAL_MAX;
  }
  if(integral < INTEGRAL_MIN)
  {
    integral = INTEGRAL_MIN;
  }
  
  return integral;   
}

float computeDerivative(float error)
{
  float rate = (error - prevError) / (pidCurrMillis - pidPrevMillis);
  float derivative = Kd * rate;

  prevError = error;
  
  return derivative;
}

void turnLeft()
{
  motors.setSpeeds(0,0);
  buzzer.play("G5");
  delay(1000);
  motors.setSpeeds(0, 50);
  delay(1500);  //takes ~1.5 seconds to turn 90 degrees left on testing surface
  motors.setSpeeds(LEFT_MOTOR_BASE_SPEED, RIGHT_MOTOR_BASE_SPEED); 
}

void turnRight()
{
  motors.setSpeeds(0,0);
  buzzer.play("G5");
  delay(1000);
  motors.setSpeeds(50, 0);
  delay(1500);  //takes ~1.5 seconds to turn 90 degrees right on testing surface
  motors.setSpeeds(LEFT_MOTOR_BASE_SPEED, RIGHT_MOTOR_BASE_SPEED); 
}
  
