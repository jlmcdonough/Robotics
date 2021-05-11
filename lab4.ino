/*
   The goal of this lab is to implement a full localization controller in 2D cartesian space.  Your robot is required to be able
   to multiple locations stored x,y coordinates in 1 program run.  The amount of "stops" along the run the locations of the stops
   should be editable constants such as:
      // goals
        const int NUMBER_OF_GOALS = 3;
        float xGoals[NUMBER_OF_GOALS] = {30, 30, 0};
        float yGoals[NUMBER_OF_GOALS] = {30, 60, 0};

   This example (assuming starting pose [0, 0, ø0]) would move:
      approximately 1 ft on x and y (45 degree angle)
      turn another 45 degrees to head straight up the y axis another ~1ft while staying on x 30
      come back to the starting location (0,0)

   You will use the trigonometry that we discussed in the localization lectures to implement this behavior.  Remember that the algorithm
   we discussed uses as input the distance traveled by each wheel as measured by the wheel encoders.  You will also likely want to use a PID
   (or part of one) controller to manage your robots response to the error in heading.
*/

#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;
#include <math.h>

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;

//DEBUGGING
const boolean MOTOR_DEBUG = false;
const boolean PID_DEBUG = false;
const boolean LOCATION_DEBUG = false;
const boolean DESTINATION_DEBUG = false;
const boolean THETA_DEBUG = false;
const boolean MATH_DEBUG = false;


//MOTORS
//constants
const int CLICKS_PER_ROTATION = 12;
const double GEAR_RATIO = 29.86F;
const double WHEEL_DIAMETER = 3.2;
const double WHEEL_CIRCUMFERENCE = 10.0531;
const double WHEEL_SEPARATION_DISTANCE = 8.5 ; //this is the constant "b" in Localization algorithm - measured in cm

const double LEFT_MOTOR_BASE_SPEED = 55.0;
const double RIGHT_MOTOR_BASE_SPEED = 55.0;
const double MOTOR_MIN_SPEED = 45.0;
const double MOTOR_MAX_SPEED = 150.0;

const double MARGIN_ERROR = .5; //range of cm the robot is allowed to be within the spot (e.g. 1 cm above or below the x and y)

//timing
unsigned long currMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 20;

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
const int NUMBER_OF_GOALS = 4;
double xGoals[NUMBER_OF_GOALS] = {60, 0, 80, 0};
double yGoals[NUMBER_OF_GOALS] = {0, -60, 50, 0};

//double xGoals[NUMBER_OF_GOALS] = {30, 30, 0, 0};
//double yGoals[NUMBER_OF_GOALS] = {0, 30, 30, 0};

double thetaGoal = 0;     //this is the angle of adjustment needed to get from current position to the next goal


//PID Controller
//constants
const double Kp = 40;
const double Ki = 0;
const double Kd = 0;
const double PID_UPPER_LIMIT = 10000;    //so high that it is irrelevant
const double PID_LOWER_LIMIT = -10000;   //so low that it is irrelevant
const double INTEGRAL_MAX = 1;
const double INTEGRAL_MIN = -1;

//timing
unsigned long pidCurrMillis;
unsigned long pidPrevMillis;

//data
double totalError = 0;  //used for integral
double prevError = 0;   //used for derivative
double pid = 0;


void setup()
{
  Serial.begin(57600);
  delay(5000);
  thetaGoal = atan2(yGoals[0], xGoals[0]);

  //buzzer.play("c32");
}

void loop()
{
  checkEncoders();
}

void checkEncoders()
{
  currMillis = millis();

  if (currMillis > (prevMillis + PERIOD) )
  {
    if (! ( ( (xGoals[goalNumber] - MARGIN_ERROR <= currX) && (currX <= xGoals[goalNumber] + MARGIN_ERROR) ) && ( (yGoals[goalNumber] - MARGIN_ERROR <= currY) && (currY <= yGoals[goalNumber] + MARGIN_ERROR) ) ) && goalNumber < NUMBER_OF_GOALS) //has not reached goal yet
    {
      doMath();

      computePID();

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

        /*
         * this helped solved the issue with not being able to complete a 30 x 30 box 
         *    starting at (0,0) with destinations of (30, 0) (30, 30) (0, 30) and (0, 0)
         *    the robot would previously be unable to return to (0,0) because it would take a wide arcing turn
         *    and the x and y would be off, but the angle correct, due to the turn and never to correct
         *    this is the problem I have not been able to resolve and this "fix" solves at least one case
         *    and in the example outlined in this lab, the robot is closer in returning to 0,0 than it previously was
         */
        if(thetaGoal < 0)
        {
          double thetaComp = (PI * 2) - abs(thetaGoal);
          thetaGoal = thetaComp;
        } 

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

    prevMillis = currMillis;
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


void computePID()
{
  double error = currTheta - thetaGoal;

  pid = Kp * error;

  if (pid > PID_UPPER_LIMIT)
  {
    pid = PID_UPPER_LIMIT;
  }
  if (pid < PID_LOWER_LIMIT)
  {
    pid = PID_LOWER_LIMIT;
  }

  //if pid > 0, then right wheel is going too strong, if pid < 0, then left wheel is going too strong
    leftSpeed = LEFT_MOTOR_BASE_SPEED + pid;
    rightSpeed = RIGHT_MOTOR_BASE_SPEED - pid;

  if (PID_DEBUG)
  {
    Serial.println("PID:");
    Serial.print(" ERROR: ");
    Serial.print(error);
    Serial.print(" PID: ");
    Serial.println(pid);
    if (pid > 0)
      Serial.println("DECREASING RIGHT");
    if (pid < 0)
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
  }

  motors.setSpeeds(leftSpeed, rightSpeed);
}
