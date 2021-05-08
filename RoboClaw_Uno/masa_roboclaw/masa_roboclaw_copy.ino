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
#include <SharpIR.h>
#include "RoboClaw.h"
#include <Servo.h>

RoboClaw roboclaw(&Serial2,10000); // roboclaw -> Serial2, Pi -> Serial

#define address 0x80

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
int leftOrRight;
int user = 0;

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
  }
  if (pressed == true) {
      moveDown();
    }
  else {
    moveUp();
  }

  
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    uwb0 = data.substring(0, 3).toInt();
    uwb1 = data.substring(3, 6).toInt();
    leftOrRight = data.substring(6).toInt();
    user = (uwb0 + uwb1)/2;
//    if (leftOrRight == 0){
//      Serial.print("Moving Left...");
//    }
//    else if (leftOrRight == 1){
//      Serial.print("Moving Right...");
//    }
//    else{
//      Serial.print("Moving Forward...");
//    }
//    Serial.print("Distance to User: ");
//    Serial.print(uwb0);
//    Serial.println(" cm");
  }

  if (pressed == true){
    
    frontLeftDistIR = frontSensorLeft.distance();
    frontRightDistIR = frontSensorRight.distance();
    sideLeftDistIR = sideSensorLeft.distance();
    sideRightDistIR = sideSensorRight.distance();

    if (user < 100){
      moveForward(0);
    }
    
    else if (frontLeftDistIR < 50 || frontRightDistIR < 50){
      // Implement case for local minima - getting stuck in a U-Shaped obstacle
//      if ((frontLeftDistIR < 30 && frontRightDistIR < 30) || (frontLeftDistIR < 15) || (frontRightDistIR < 15)){
//        moveBackward(32);      
//      }
//      if (frontLeftDistIR < 30 && frontRightDistIR < 30){
//        moveBackward(32);      
//      }
      if (frontLeftDistIR < frontRightDistIR){
        if (user > 100 && user < 300){
          roboclaw.BackwardM1(address, 32);
          roboclaw.ForwardM2(address, 32);
        }
        else if(user > 300){
          roboclaw.BackwardM1(address, 64);
          roboclaw.ForwardM2(address, 64);
        }
      }
      else{
        if (user > 100 && user < 300){
          roboclaw.ForwardM1(address, 32);
          roboclaw.BackwardM2(address, 32);
        }
        else if(user > 300){
          roboclaw.ForwardM1(address, 64);
          roboclaw.BackwardM2(address, 64);
        }
      }
    }


//    // Checks if suitcase is moving parallel to a wall
//    if (sideLeftDistIR < 50) {
//      leftWall = true;
//    }
//    else {
//      leftWall = false;
//    }
//    if (sideRightDistIR < 50) {
//      rightWall = true;
//    }
//    else {
//      rightWall = false;
//    }
    
    else if (user > 300) {
      // Move full speed if far away
      if (leftOrRight == 0) {
        changeSpeed(127,100);
      }
      else if(leftOrRight == 1){
        changeSpeed(100, 127); 
      }
      else {
        moveForward(127);
      }
    }
    else if(user > 100 && user < 300) {
      // Move full speed if far away
      if (leftOrRight == 0) {
        changeSpeed(100, 70);
      }
      else if(leftOrRight == 1){
        changeSpeed(70, 100); 
      }
      else {
        moveForward(100);
      }
    }
  
    else{
      moveForward(0); 
    }
  }
  else{
    moveForward(0); 
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
  myservo2.write(70);
}

void moveDown(){
  myservo1.write(70);
  myservo2.write(0);
}
