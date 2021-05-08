/*
 * MASA
 *  Modular Autonomous Suitcase Attachment
 *  24671 - 05/21
 *  Anthony.Tien, Dhruv.Sharma, Nicole.Chu, Sean.Birke, Vincent.Liu
 *    w/ Nabarun.Banerjee, Mayank.Jain
 *    
 *  Arduino Controls
 *  Roboclaw Library enclosed - "RoboClaw.h", "RoboClaw.cpp"
 *  
 *  
 *  
 *  ---- look at end for helper functions ----
 *  
 */


#include <SoftwareSerial.h>
#include <SharpIR.h>
#include "RoboClaw.h"
#include <Servo.h>


//SoftwareSerial roboSerial(2,3); // for arduino uno
//RoboClaw roboclaw(&roboSerial,10000); // for arduino uno

RoboClaw roboclaw(&Serial2,10000); // roboclaw -> Serial2, Pi -> Serial

#define address 0x80

//Velocity PID coefficients.
#define Kp 1.0 // Can change values ( Don't need to )
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

#define FrontIRPin1 A3
#define FrontIRPin2 A4
#define SideLeftIRPin A5
#define SideRightIRPin A6
#define model 1080

SharpIR frontSensorLeft = SharpIR(FrontIRPin1, model); // calibrate for left or right
SharpIR frontSensorRight = SharpIR(FrontIRPin2, model);
SharpIR sideSensorLeft = SharpIR(SideLeftIRPin, model);
SharpIR sideSensorRight = SharpIR(SideRightIRPin, model);

int frontLeftDistIR;
int frontRightDistIR;
int sideLeftDistIR;
int sideRightDistIR;

int leftWall = false;
int rightWall = false;

int uwb0 = 0;
int uwb1 = 0;
int headingAngle = 0;
int leftOrRight;
int ratioAnglesToCounts = 33; // encoder ticker = 1 degree

//Servo for retraction 
Servo myservo1; //left
Servo myservo2; //right

const int buttonPin = 5;
int buttonState = 0;
int pressed = false;

void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(9600); // Make sure in serial monitor to change the baud rate to see the data
  
  roboclaw.begin(38400);
  
  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps);  

  myservo1.attach(3);
  myservo2.attach(4);
  pinMode(buttonPin, INPUT);
}

void loop() {
  // Read Button, for mode Manual or Autonomous
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    pressed = !pressed;
    delay(500);
    
    if (pressed == true) {
      moveDown();
    }
    else {
      moveUp();
    }
  }

  
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    uwb0 = data.substring(0, 3).toInt();
    uwb1 = data.substring(3, 6).toInt();
    leftOrRight = data.substring(6).toInt();
//    Serial.print("uwb0: ");
//    Serial.print(uwb0);
//    Serial.print(" uwb1: ");
//    Serial.print(uwb1);
//    Serial.print(" lr_bool_s: ");
//    Serial.println(leftOrRight);

    if (leftOrRight == 0){
      Serial.print("Moving Left...");
    }
    else if (leftOrRight == 1){
      Serial.print("Moving Right...");
    }
    else{
      Serial.print("Moving Forward...");
    }
    Serial.print("Distance to User: ");
    Serial.print(uwb0);
    Serial.println(" cm");
  }

  if (pressed == true){
    
    frontLeftDistIR = frontSensorLeft.distance();
    frontRightDistIR = frontSensorRight.distance();
    sideLeftDistIR = sideSensorLeft.distance();
    sideRightDistIR = sideSensorRight.distance();

    if (frontLeftDistIR < 50 || frontRightDistIR < 50){
      // Implement case for local minima - getting stuck in a U-Shaped obstacle
      if ((frontLeftDistIR < 30 && frontRightDistIR < 30) || (frontLeftDistIR < 15) || (frontRightDistIR < 15)){
        moveBackward(32);      
      }
      else if (frontLeftDistIR < frontRightDistIR){
        roboclaw.BackwardM1(address, 32);
        roboclaw.ForwardM2(address, 32);
      }
      else{
        roboclaw.ForwardM1(address, 32);
        roboclaw.BackwardM2(address, 32);
      }
    }

    if (sideLeftDistIR < 50) {
      leftWall = true;
    }
    else {
      leftWall = false;
    }
    if (sideRightDistIR < 50) {
      rightWall = true;
    }
    else {
      rightWall = false;
    }
    
    if (uwb0 > 150) {
      // Move full speed if far away
      if (leftOrRight == 0 and !leftWall) {
        changeSpeed(64,32);
      }
      else if(leftOrRight == 1 and !rightWall){
        changeSpeed(32,64); 
      }
      else {
        moveForward(64);
      }
    }
//    else if(uwb0 > 150 && uwb0 < 400) {
//      // Move full speed if far away
//      if (leftOrRight == 0) {
//        changeSpeed(16, 32);
//      }
//      else if(leftOrRight == 1){
//        changeSpeed(32, 16); 
//      }
//      else {
//        moveForward(32);
//      }
//    }
  
    else{
      moveForward(0); 
    }
  }
  else{
    changeSpeed(0,0);
  }
}

void moveBackward(int motorSpeed){
  // calibrate based on front of suitcase
  roboclaw.ForwardM1(address,motorSpeed);
  roboclaw.ForwardM2(address,motorSpeed);
}

void moveForward(int motorSpeed){
  // calibrate based on front of suitcase
  roboclaw.BackwardM1(address,motorSpeed);
  roboclaw.BackwardM2(address,motorSpeed);
}

void changeSpeed(int motorSpeed1, int motorSpeed2){
// Motor 1 = Left motor
// Motor 2 = Right motor
  roboclaw.BackwardM1(address,motorSpeed1);
  roboclaw.BackwardM2(address,motorSpeed2);
}

void turnLeft(int motorSpeed) {
  changeSpeed(motorSpeed, motorSpeed/2);
}

void turnRight(int motorSpeed) {
  changeSpeed(motorSpeed/2, motorSpeed);
}

void moveUp(){
  myservo1.write(0);
  myservo2.write(90);
}

void moveDown(){
  myservo1.write(90);
  myservo2.write(0);
}
