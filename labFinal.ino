/*
 * We have created robots that can move around their environment and are doing well controlling dynamics.  Now we are going to complete them by adding 
 * an obstacle avoidance behavior and tuning it to work with our localization and go-to-goal behavior to create a full potential fields algorithm.
 * The potential fields algorithm works by creating attractive and repulsive “fields”.  The robot will move towards an attractive field just like a ball 
 * rolling down a hill.  Repulsive fields are equivalent to smooth “hills” which increase in height towards the center.  The close the robot gets to them, 
 * the higher the “hill” and the more it will be pushed away.  You can (and should) read more about potential fields and its implementation by reading the authors paper here.
 * 
 * Requirements: 
 *    Your robot must be able to successfully navigate to multiple goal locations (one at a time) in x,y cartesian space as it did in lab 4, emitting 
 *    an audible note when it reaches each goal.  It should stop at the final goal.  The goals should be easy to change in your code as you will be given 
 *    them on the day of the competition.
 *    Your robot must be able to avoid randomly placed obstacles while navigating to the goal.  To accomplish this well, you robot should be able to take 
 *    in multiple readings from the range finding ultrasonic sensor at different angles.
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
const boolean MAGNITUDE_DEBUG = false;
const boolean SENSOR_PID_DEBUG = false;
const boolean LOCAL_PID_DEBUG = false;
const boolean MATH_DEBUG = false;
const boolean THETA_DEBUG = false;
const boolean DESTINATION_DEBUG = false;
const boolean LOCATION_DEBUG = false;
const boolean MOTOR_DEBUG = false;

//HEAD SERVO
//pins 
const int HEAD_SERVO_PIN = 0;

//constants
const unsigned long HEAD_MOVEMENT_PERIOD = 200;
const int HEAD_POSITIONS_SIZE = 5;
const int HEAD_POSITIONS[HEAD_POSITIONS_SIZE] = {130, 110, 90, 70, 50}; 

//timing
unsigned long headCurrMillis;
unsigned long headPrevMillis;

//data
int currHeadPosition = 0;

//ULTRASONIC
//pins
const int ECHO_PIN = 30;
const int TRIG_PIN = 17;

//constants
const float MAX_DISTANCE = 60.0;
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 10;

//timing
unsigned long usCurrMillis;
unsigned long usPrevMillis;
const unsigned long US_PERIOD =  80;

//data
int currReadPosition = currHeadPosition;
int prevReadPosition = currReadPosition - 1;
int prevPosition = currReadPosition - 1;
float distanceReadings[HEAD_POSITIONS_SIZE] = {STOP_DISTANCE, STOP_DISTANCE, STOP_DISTANCE, STOP_DISTANCE, STOP_DISTANCE};
float distance = 0;
float duration = 0;
float thisRoundDistance = 0;
float distMagnitude[HEAD_POSITIONS_SIZE] = {0, 0, 0, 0, 0};


//MOTORS
//constants
const int CLICKS_PER_ROTATION = 12;
const double GEAR_RATIO = 29.86F;
const double WHEEL_DIAMETER = 3.2;
const double WHEEL_CIRCUMFERENCE = 10.0531;
const double WHEEL_SEPARATION_DISTANCE = 8.5 ; //this is the constant "b" in Localization algorithm - measured in cm

const float LEFT_MOTOR_BASE_SPEED = 50.0;  
const float RIGHT_MOTOR_BASE_SPEED = 50.0;
const float MOTOR_MIN_SPEED = 40.0;
const float MOTOR_MAX_SPEED = 110.0;
const double MARGIN_ERROR = .5; //range of cm the robot is allowed to be within the spot (e.g. 1 cm above or below the x and y)


//timing
unsigned long motorCurrMillis;
unsigned long motorPrevMillis;
const unsigned long MOTOR_PERIOD = 50;

//data
long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;
double Sl = 0;   //distance traveled by left wheel
double Sr = 0;   //distance traveled by right wheel

double prevSl = 0;   //previous distance traveled by left wheel
double prevSr = 0;   //previous distance traveled by right wheel

double currTheta = 0;
double prevTheta = 0;

//difference between these values and Sl and Sr is that these are the coordinates, Sl and Sr are strictly distance and has no sense of direction
double currX = 0;    //distance along x robot has traveled for the current goal
double currY = 0;    //distance along y robot has traveled for the current goal
double prevX = 0;    //distance along x robot has traveled in the previous iteration
double prevY = 0;    //distance along y robot has traveled in the previous iteration

double robotDist = 0;  //distance robot has traveled, based off its center point

//change variables
double deltaS = 0;
double deltaTheta = 0;
double deltaX = 0;
double deltaY = 0;

double leftSpeed = LEFT_MOTOR_BASE_SPEED;
double rightSpeed = RIGHT_MOTOR_BASE_SPEED;

//goals
//distances - in cm (1 in = 2.54cm) - angle is measured in radians
int goalNumber = 0;    //indicates current goal target
const int NUMBER_OF_GOALS = 1;
double xGoals[NUMBER_OF_GOALS] = {500};
double yGoals[NUMBER_OF_GOALS] = {100};
double thetaGoal = 0;     //this is the angle of adjustment needed to get from current position to the next goal


//PID Controller
//SENSOR
//constants
const float localKp = 40;
const float LOCAL_PID_UPPER_LIMIT = 10000;    //so high that it is irrelevant
const float LOCAL_PID_LOWER_LIMIT = -10000;   //so low that it is irrelevant

//data
float localPID = 0;
float sensorPID = 0;
float magnitudeMulti[HEAD_POSITIONS_SIZE] = {5, 7, 12, -7, -5}; 
//float magnitudeMulti[HEAD_POSITIONS_SIZE] = {0, 0, 0, 0, 0};

void setup() 
{
  Serial.begin(57600);
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(90);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  
  Serial.println("3");
  buzzer.play("c5");  
  thetaGoal = atan2(yGoals[0], xGoals[0]);
    
  delay(2000);
  Serial.println("2");
  
  buzzer.play("b5");
  delay(2000);
  
  Serial.println(HEAD_POSITIONS[0]);

  Serial.println("1");
  buzzer.play("a5");
  headServo.write(HEAD_POSITIONS[0]);
  delay(1000);

  for(int i = 0; i < HEAD_POSITIONS_SIZE; i++)
  {
    headServo.write(HEAD_POSITIONS[i]);
    currReadPosition = currHeadPosition;
    if(currHeadPosition == (HEAD_POSITIONS_SIZE - 1) )
    {
      currHeadPosition = 0;
    }
    else
    {
      currHeadPosition++;
    } 

    delay(HEAD_MOVEMENT_PERIOD);

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    distance = duration * 0.034 / 2;

    if(distance > MAX_DISTANCE || distance == 0)  //a result of exactly 0 is impossible as the sensor is behind the front of the robot.  The "common" 0 return is a bug and just going to assume it did not detect any objects within its max distance range
    {
      distance = MAX_DISTANCE;
    }

    distanceReadings[currReadPosition] = distance;

    distMagnitude[currReadPosition] = (MAX_DISTANCE - distance) * magnitudeMulti[currReadPosition];


    //Because of frequent sensor inaccuracies, this "ideal" method would not work and results in the robot being super jittery
    if(currReadPosition == HEAD_POSITIONS_SIZE - 1) //end sweep
    {
      //robot is currently to the right of the goal trajectory (wants to go more left to readjust), so if an obstacle is head on, would want it to try and favor going left around the object because negative pid means strengthen right     
      //take the side with the least obstacles
      float leftSideSum = magLeft();
      float rightSideSum = magRight();
      
      if(leftSideSum < abs(rightSideSum))  //more obstacles to the right, therefore go left
      {
        distMagnitude[HEAD_POSITIONS_SIZE / 2] *= -1;
      }   
    }
    delay(US_PERIOD);    
  }

      Serial.print("Magnitude List: [");
      for(int i = 0; i < HEAD_POSITIONS_SIZE - 1; i++)
      {
        Serial.print(distMagnitude[i]);
        Serial.print(", ");
      }
      Serial.print(distMagnitude[HEAD_POSITIONS_SIZE - 1]);
      Serial.println("]");

      Serial.print("MAGNITUDE AVG: ");
      Serial.println(magnitudeAvg());
    
  
  Serial.println("GOGOGOGO");
}

void loop() 
{
  sensorStuff();
  localizing();
}


void sensorStuff()
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

    if(distance > MAX_DISTANCE || distance == 0)  //a result of exactly 0 is impossible as the sensor is behind the front of the robot.  The "common" 0 return is a bug and just going to assume it did not detect any objects within its max distance range
    {
      distance = MAX_DISTANCE;
    }

    distanceReadings[currReadPosition] = distance;

    distMagnitude[currReadPosition] = (MAX_DISTANCE - distance) * magnitudeMulti[currReadPosition];

    //will decide to make middle multiplier positive of negative depending on whether there appears to be more obstacles to the right or left
    //due to the time this value is calculated, the beginning positions (the left of robot) is more recent and the latter (right of robot) is older
    if(currReadPosition == (HEAD_POSITIONS_SIZE / 2)) //STRAIGHT ON
    {
      //robot is currently to the right of the goal trajectory (wants to go more left to readjust), so if an obstacle is head on, would want it to try and favor going left around the object because negative pid means strengthen right      
      //take the side with the least obstacles
      float leftSideSum = magLeft();
      float rightSideSum = magRight();
      
      if(leftSideSum < abs(rightSideSum))  //more obstacles to the right, therefore go left
      {
        distMagnitude[HEAD_POSITIONS_SIZE / 2] *= -1;
      }   
    }
       
    sensorPID = magnitudeAvg();
    
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

    if(MAGNITUDE_DEBUG)
    {
      Serial.print("Magnitude List: [");
      for(int i = 0; i < HEAD_POSITIONS_SIZE - 1; i++)
      {
        Serial.print(distMagnitude[i]);
        Serial.print(", ");
      }
      Serial.print(distMagnitude[HEAD_POSITIONS_SIZE - 1]);
      Serial.println("]");

      Serial.print("MAGNITUDE AVG: ");
      Serial.println(magnitudeAvg());
    }
  }
}

float magnitudeAvg()
{
  float magSum = 0;
  for(int i = 0; i < HEAD_POSITIONS_SIZE; i++)
  {
    magSum += distMagnitude[i];
  }
  return magSum/HEAD_POSITIONS_SIZE;
}



void localizing()
{
  motorCurrMillis = millis();

  if (motorCurrMillis > (motorPrevMillis + MOTOR_PERIOD) )
  {
    Serial.println(thetaGoal);
    if (! ( ( (xGoals[goalNumber] - MARGIN_ERROR <= currX) && (currX <= xGoals[goalNumber] + MARGIN_ERROR) ) && ( (yGoals[goalNumber] - MARGIN_ERROR <= currY) && (currY <= yGoals[goalNumber] + MARGIN_ERROR) ) ) && goalNumber < NUMBER_OF_GOALS) //has not reached goal yet
    {
      doMath();

      computeLocalPID();

      slowdown();
      adjustSpeeds();
    }
    else //goal is reached
    {
      goalNumber += 1;  //goal is reached, checking for next goal

      if (NUMBER_OF_GOALS == (goalNumber) ) //all goals have been reached
      {
        motors.setSpeeds(0, 0);
        delay(1000);
        buzzer.play("F5");
      }
      else if (NUMBER_OF_GOALS < (goalNumber) )  //same as above but don't want the buzzer to be continuous
      {
        motors.setSpeeds(0, 0);
      }
      else   //more goals to be reached, increment goal tracker counter
      {
        double yGoalAdjust = yGoals[goalNumber] - currY;
        double xGoalAdjust = xGoals[goalNumber] - currX;
        thetaGoal = atan2(yGoalAdjust, xGoalAdjust);
        motors.setSpeeds(0, 0);
        buzzer.play("A5");
        delay(1000);

        if (THETA_DEBUG)
        {
          Serial.println("**********************************************");
          Serial.print("Y GOAL ADJUST: ");
          Serial.print(yGoalAdjust);
          Serial.print(" X GOAL ADJUST: ");
          Serial.print(xGoalAdjust);
          Serial.print(" THETA GOAL: ");
          Serial.println(thetaGoal);
          Serial.println("**********************************************");
        }


      }
    }

    if (DESTINATION_DEBUG)
    {
      Serial.println(" ");
      Serial.print("DESTINATION #: ");
      Serial.print( goalNumber );
      Serial.print(" X GOAL: ");
      Serial.print(xGoals[goalNumber]);
      Serial.print(" Y GOAL: ");
      Serial.print(yGoals[goalNumber]);
      Serial.print(" THETA GOAL: ");
      Serial.print(thetaGoal);
      Serial.print(" TOTAL DISTANCE: ");
      Serial.println(robotDist);
      Serial.println(" ");
    }


    if (LOCATION_DEBUG)
    {
      Serial.print("CURR X: ");
      Serial.print( currX );
      Serial.print(" CURR Y: ");
      Serial.print(currY);
      Serial.print(" CURR THETA: ");
      Serial.println(currTheta);
      Serial.println(" ");
    }

    motorPrevMillis = motorCurrMillis;
    adjustAngle();
  }
}

void doMath()
{      
  countsLeft += encoders.getCountsAndResetLeft();
  countsRight += encoders.getCountsAndResetRight();

  Sl += ( (countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE );
  Sr += ( (countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE );

  deltaS = ( (Sr - prevSr) + (Sl - prevSl) ) / 2;
  robotDist += deltaS;

  deltaTheta = ( (Sr - prevSr) - (Sl - prevSl) ) / WHEEL_SEPARATION_DISTANCE; //this is in radians, theta is used as radians
  currTheta += deltaTheta;

  deltaX = deltaS * cos(thetaGoal + (deltaTheta / 2));
  currX += deltaX;

  deltaY = deltaS * sin(thetaGoal + (deltaTheta / 2));
  currY += deltaY;

  if (MATH_DEBUG)
  {
    Serial.println("DISTANCE:");
    Serial.print(" Sr: ");
    Serial.print(Sr);
    Serial.print(" PREVSR: ");
    Serial.print(prevSr);
    Serial.print(" Sl: ");
    Serial.print(Sl);
    Serial.print(" PREVSL: ");
    Serial.println(prevSl);

    Serial.println("ANGLE:");
    Serial.print(" THETA GOAL: ");
    Serial.print(thetaGoal);
    Serial.print(" DTHETA: ");
    Serial.print(deltaTheta);
    Serial.print(" CURR ANGLE: ");
    Serial.println(currTheta);

    Serial.println("X");
    Serial.print(" deltaX: ");
    Serial.print(deltaX);
    Serial.print(" cos: ");
    Serial.print(cos(thetaGoal + (deltaTheta / 2)));
    Serial.print(" prevX: ");
    Serial.println(prevX);

    Serial.println("Y:");
    Serial.print(" deltaY: ");
    Serial.print(deltaY);
    Serial.print(" sin: ");
    Serial.print(sin(thetaGoal + (deltaTheta / 2)));
    Serial.print(" prevY: ");
    Serial.println(prevY);
  }
  
  prevLeft = countsLeft;
  prevRight = countsRight;
  prevX = currX;
  prevY = currY;
  prevSl = Sl;
  prevSr = Sr;
}

void computeLocalPID()
{
  double error = currTheta - thetaGoal;

  localPID = localKp * error;

  if (localPID > LOCAL_PID_UPPER_LIMIT)
  {
    localPID = LOCAL_PID_UPPER_LIMIT;
  }
  if (localPID < LOCAL_PID_LOWER_LIMIT)
  {
    localPID = LOCAL_PID_LOWER_LIMIT;
  }

  //if pid > 0, then right wheel is going too strong, if pid < 0, then left wheel is going too strong
    leftSpeed = LEFT_MOTOR_BASE_SPEED + sensorPID + localPID;
    rightSpeed = RIGHT_MOTOR_BASE_SPEED - sensorPID - localPID;

  if (LOCAL_PID_DEBUG)
  {
    Serial.println("PID:");
    Serial.print(" ERROR: ");
    Serial.print(error);
    Serial.print(" PID: ");
    Serial.println(localPID);
    if (localPID > 0)
      Serial.println("DECREASING RIGHT");
    if (localPID < 0)
      Serial.println("INCREASING RIGHT");
  }

}

void slowdown()
{
  double xDist = xGoals[goalNumber] - currX;
  double yDist = currY - yGoals[goalNumber];
  double distToGo = sqrt((xDist * xDist) + (yDist * yDist));

  if (distToGo < 15)
  {
    leftSpeed = leftSpeed - (15 - distToGo);   //if distToGo is 4.9, then will only subtract 5.1 off speed
    rightSpeed = rightSpeed - (15 - distToGo); //when distToGo reached .1, then will subtract 9.9 off speed and robot will almost be at base speed
  }
}

void adjustSpeeds()
{
  if (leftSpeed > MOTOR_MAX_SPEED)
  {
    leftSpeed = MOTOR_MAX_SPEED;
  }
  if (rightSpeed > MOTOR_MAX_SPEED)
  {
    rightSpeed = MOTOR_MAX_SPEED;
  }

  if (leftSpeed < MOTOR_MIN_SPEED)
  {
    leftSpeed = MOTOR_MIN_SPEED;
  }
  if (rightSpeed < MOTOR_MIN_SPEED)
  {
    rightSpeed = MOTOR_MIN_SPEED;
  }

  if (MOTOR_DEBUG)
  {
    Serial.print(" LEFT SPEED: ");
    Serial.print(leftSpeed);
    Serial.print("RIGHT SPEED: ");
    Serial.println(rightSpeed);
    Serial.println(" ");
    Serial.println("------------------------------------------------------");
  }

  motors.setSpeeds(leftSpeed, rightSpeed);
}

void adjustAngle()
{
  double yGoalAdjust = yGoals[goalNumber] - currY;
  double xGoalAdjust = xGoals[goalNumber] - currX;
  thetaGoal = atan2(yGoalAdjust, xGoalAdjust);
}

float magLeft()
{
  int leftElems = (HEAD_POSITIONS_SIZE / 2);   //integer division, should round down -- HEAD_POSITION SIZE = 5 --> 5 // 2 = 2
  float sumLeft = 0;
  for(int i = 0; i < leftElems; i++)
  {
    sumLeft += distMagnitude[i];
  }

  return sumLeft;
}

float magRight()
{
  int rightElems = (HEAD_POSITIONS_SIZE / 2);   //integer division, should round down     //for HEAD_POSITION SIZE = 5 --> 5 // 2 = 2
  float sumRight = 0;
  for(int i = HEAD_POSITIONS_SIZE - rightElems ; i < HEAD_POSITIONS_SIZE; i++)           //i = 5 - 2 = 3; i < 5, therefore indices 3 and 4 is all that is taken
  {
    sumRight += distMagnitude[i];
  }

  return sumRight;
}

  
