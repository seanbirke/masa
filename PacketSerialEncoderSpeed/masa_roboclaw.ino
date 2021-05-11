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

int threshold1Speed = 64; // between 100-300
int threshold2Speed = 100; // > 300

int turnSpeed1 = 40; // between 100-300
int turnSpeed2 = 32; // >300


//Servo for retraction 
Servo myservo1; //left
Servo myservo2; //right

const int buttonPin = 5;
int buttonState = 0;
int pressed = false;
int isUWBreset = false;


int camAngle = 90;

void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(9600); // Make sure in serial monitor to change the baud rate to see the data
  
  roboclaw.begin(38400);

  myservo1.attach(3);
  myservo2.attach(4);
  pinMode(buttonPin, INPUT);
  moveDown();
}

void loop() {

  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    uwb0 = data.substring(0, 3).toInt();
    uwb1 = data.substring(3, 6).toInt();
    leftOrRight = data.substring(6).toInt();
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
  
    // Read Button, for mode Manual or Autonomous
//  buttonState = digitalRead(buttonPin);
//  if (buttonState == HIGH) {
//    pressed = !pressed;
//    delay(500);
//  }
//  if (pressed == true) {
//      moveDown();
//    }
//  else {
//    moveUp();
//  }
  
  if (!isUWBreset && leftOrRight == 3) {
    pressed = !pressed;
    isUWBreset = true;
    if (pressed == true) {
      moveDown();
    }
    else {
      moveUp();
    }
  }
  else if (isUWBreset && leftOrRight != 3) {
    isUWBreset = false;
  }
  
  if (pressed == true){
    
    frontLeftDistIR = frontSensorLeft.distance();
    frontRightDistIR = frontSensorRight.distance();
    sideLeftDistIR = sideSensorLeft.distance();
    sideRightDistIR = sideSensorRight.distance();
    
    if (uwb0 < 100){
      moveForward(0);
    }
    else if (frontLeftDistIR < 50 || frontRightDistIR < 50){
      // Implement case for local minima - getting stuck in a U-Shaped obstacle
      if ((frontLeftDistIR < 30 && frontRightDistIR < 30) || (frontLeftDistIR < 15) || (frontRightDistIR < 15)){
        moveBackward(32);      
      }
//      if (frontLeftDistIR < 30 && frontRightDistIR < 30){
//        moveBackward(32);      
//      }
      else if (frontLeftDistIR < frontRightDistIR){
        if (uwb0 >= 100 && uwb0 < 1000){
          roboclaw.BackwardM1(address, turnSpeed1);
          roboclaw.ForwardM2(address, turnSpeed1);
        }
//        else if(uwb0 > 300){
//          roboclaw.BackwardM1(address, turnSpeed2);
//          roboclaw.ForwardM2(address, turnSpeed2);
//        }
      }
      else{
        if (uwb0 >= 100 && uwb0 < 1000){
          roboclaw.ForwardM1(address, turnSpeed1);
          roboclaw.BackwardM2(address, turnSpeed1);
        }
//        else if(uwb0 > 300){
//          roboclaw.ForwardM1(address, turnSpeed2);
//          roboclaw.BackwardM2(address, turnSpeed2);
//        }
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
    
    else if (uwb0 >= 300) {
      // Move full speed if far away
      if (leftOrRight == 0) {
        changeSpeed(threshold2Speed,threshold2Speed-27);
      }
      else if(leftOrRight == 1){
        changeSpeed(threshold2Speed-27, threshold2Speed); 
      }
      else {
        moveForward(threshold2Speed);
      }
    }
    else if(uwb0 >= 100 && uwb0 < 300) {
      // Move full speed if far away
      if (leftOrRight == 0) {
        changeSpeed(threshold1Speed, threshold1Speed-30);
      }
      else if(leftOrRight == 1){
        changeSpeed(threshold1Speed-30, threshold1Speed); 
      }
      else if(leftOrRight == 2){ 
        moveForward(threshold1Speed);
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
  myservo2.write(0);
}

void moveDown(){
  myservo1.write(60);
  myservo2.write(camAngle);
}

//void greenLight(){
//  for (int i = 7; i >= 0; i--) {
//    leds[i] = CRGB (0, 255, 0);
//    FastLED.show();
//  }
//}
//
//void yellowLight(){
//  for (int i = 7; i >= 0; i--) {
//    leds[i] = CRGB (255, 255, 0);
//    FastLED.show();
//  }
//}
//
//void rightTurn(){
//  for (int i = 7; i >= 0; i--) {
//    leds[i] = CRGB ( 255, 0, 0);
//    FastLED.show();
//    delay(100);
//  }
//  for (int i = 0; i <= 7; i++) {
//    leds[i] = CRGB ( 0, 0, 0);
//    FastLED.show();
//  }
//}
