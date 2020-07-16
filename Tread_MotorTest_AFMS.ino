//TreadBot Chassis Testing Script
//Joshua Curfman
//TESTING PURPOSES, USES ADAFRUIT MOTOR SHIELD


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield afms = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = afms.getMotor(1);
Adafruit_DCMotor *rightMotor = afms.getMotor(2);

const int xAxis = A1;
const int yAxis = A0;
int range = 24;
int center = range/2;
int threshold = range/6;

int currentSpeed;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  afms.begin();
  Serial.println("Motor Shield initialized");
}

void loop() {
  int xReading, yReading; //Readings of joystick
  xReading=readAxis(xAxis);
  yReading=readAxis(yAxis);
  Serial.print("X: ");
  Serial.println(xReading);
  Serial.print("Y: ");
  Serial.println(yReading);
  delay(100);
  
  if(xReading==0 && yReading==0) {
    Serial.println("Stop");
    currentSpeed=0;
    leftMotor->setSpeed(currentSpeed);
    rightMotor->setSpeed(currentSpeed);
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
  }
  if(xReading==0 && yReading!=0) {
    if(yReading>0) {
      Serial.println("Run forward");
      int newSpeed=map(yReading, 0, 12, 0, 255);
      leftMotor->setSpeed(newSpeed);
      rightMotor->setSpeed(newSpeed);
      currentSpeed=newSpeed;
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    }
    else if(yReading<0) {
      Serial.println("Run backwards");
      int newSpeed=map(abs(yReading), 0, 12, 0, 255);
      leftMotor->setSpeed(newSpeed);
      rightMotor->setSpeed(newSpeed);
      currentSpeed=newSpeed;
      leftMotor->run(BACKWARD);
      rightMotor->run(BACKWARD);
    }
  }
  else if(xReading!=0) {
    //Needs substantial work
    if(xReading>0) {
      //turn right
      Serial.println("Turn right");
      int speedOffset=map(xReading, 0, 12, 0, 255);
      leftMotor->setSpeed(128);
      rightMotor->setSpeed(128);
      leftMotor->run(FORWARD);
      rightMotor->run(BACKWARD);
    }
    else if(xReading<0) {
      Serial.println("Turn left");
      leftMotor->setSpeed(128);
      rightMotor->setSpeed(128);
      leftMotor->run(BACKWARD);
      rightMotor->run(FORWARD);
    }
  }
}

void oldLoop() {
  // put your main code here, to run repeatedly:
  leftMotor->setSpeed(200);
  rightMotor->setSpeed(200);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  Serial.println("Motor running forwards");
  delay(1000);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
  Serial.println("Motor running backwards");
  delay(1000);
}

int readAxis(int thisAxis) {
  //read the analog input
  int reading = analogRead(thisAxis);

  //map the reading from the analog input range to the output range
  reading = map(reading, 0, 1023, 0, range);

  //if output reading is outside from the rest position threshold, use it
  int distance = reading - center;
  if(abs(distance)<threshold) {
    distance=0;
  }

  //return the sitance for this axis
  return distance;
}
