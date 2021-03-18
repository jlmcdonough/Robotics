/*
 *The first part of the lab includes adding the HC-SR04 ultrasonic sensor to your robot and successfully producing 
 *in CM in the serial monitor.  Follow along with part 1 of the lab video. I will be providing 1 sensor in class 
 *on Monday.  I will also bring a hot glue gun into school to help mount the sensor if you wish to do it there.  
 *You are welcome to do it on your own as well.  I will provide you 4 wires, but you will be responsible for correct 
 *wiring (the video has a guide). 

Submission for this lab includes:

Your code either as pasted text or attached .ino file. 
A picture of your robot placed 12" away from a wall with some sort 
of measuring tool (a ruler is sufficient) and the accompanying screenshot of your serial monitor 
showing the distance reported.  If there are any deviations for your robot from the ~30.5 cm report 
why you think you are seeing the results you are. 
 */

#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;

//Ultrasonic timing
unsigned long currMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 20;

//Ultrasonic Pins
const int ECHO_PIN = 30;
const int TRIG_PIN = 17;

//Ultrasonic Distances
float distance = 0;

//this lab is not concerned with stopping a foot from the wall, but rather seeing the sensors pick up the wall a foot away by the user manually moving the robot, therefore am not concerned with robot stopping a foot before the wall
//const float MAX_DISTANCE = 30.48; //want to stop 12in from wall, therefore max distance is 30.48 cm


//to run once
void setup()
{
  Serial.begin(57600);

  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT); 
  
  delay(1000);
  buzzer.play("c32");
}
  
//to run repeatedly
void loop() 
{
  usReadCm();
}

void usReadCm()
{
  currMillis = millis();

  if(currMillis > (prevMillis + PERIOD) )
  {

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 38000);

    distance = duration * 0.034 / 2;

/*  Robot does not need to move yet, so am not concerned
    if(distance > MAX_DISTANCE)
    {
      distance = MAX_DISTANCE;
    }

    if(distance == 0)
    {
      distance = MAX_DISTANCE;
    }
*/
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    prevMillis = currMillis;
  }
}
