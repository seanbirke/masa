//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include <SharpIR.h>
#include "RoboClaw.h"
#include <Servo.h>

Servo myservo1;
Servo myservo2;

RoboClaw roboclaw(&Serial2,10000);

#define address 0x80

//Velocity PID coefficients.
#define Kp 1.0 // Can change values
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

#define FrontIRPin1 A3
#define FrontIRPin2 A4
#define model 1080

SharpIR frontSensor1 = SharpIR(FrontIRPin1, model);
SharpIR frontSensor2 = SharpIR(FrontIRPin2, model);

int frontIRdistance1;
int frontIRdistance2;

int uwb0 = 0;
int uwb1 = 0;
int headingAngle = 0;
int previousHeadingAngle = 0;
int leftOrRight;
int ratioAnglesToCounts = 33;
int timingWindow = 500;

const int buttonPin = 5;
int buttonState = 0;
int pressed = false;

unsigned long time0;
unsigned long time1;
unsigned long time2;
unsigned long time3;
unsigned long time4;
unsigned long time5;
float angleParam = 0.5;

bool angleStatus = false;
bool leftTurn = true;

void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(9600); // Make sure in serial monitor to change the baud rate to see the data
  
//  serialPi.begin(9600);
  roboclaw.begin(38400);
  
  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps);  

  myservo1.attach(3);
  myservo2.attach(4);
  pinMode(buttonPin, INPUT);
}

void loop() {
  buttonState = digitalRead(buttonPin);
//  Serial.println(pressed);
  if (buttonState == HIGH){
    pressed = !pressed;
    
    delay(500);
  }
  if (pressed == true){
    moveDown();
  }
  else{
    moveUp();
  }

  if (pressed == true){
    if (Serial.available() > 0) {
      // uwb0 distance
      String data = Serial.readStringUntil('\n');
      uwb0 = data.toInt();
  //    Serial.print(uwb0);
      data = Serial.readStringUntil('\n');
      uwb1 = data.toInt();
  //    Serial.print(uwb1);
      // heading angle of tag
      data = Serial.readStringUntil('\n');
      leftOrRight = data.toInt();
      if (leftOrRight == 0){
        Serial.println("Moving Left...");
      }
      else if (leftOrRight == 1){
        Serial.println("Moving Right...");
      }
      else{
        Serial.println("Moving Forward...");
      }
    }
  
    frontIRdistance1 = frontSensor1.distance();
    frontIRdistance2 = frontSensor2.distance();
  //  Serial.print("1:");
  //  Serial.print(frontIRdistance1);
  //  Serial.print(" 2:");
  //  Serial.println(frontIRdistance2);
  
    if (frontIRdistance1 < 50 || frontIRdistance2 < 50){
      // Implement case for local minima - getting stuck in a U-Shaped obstacle
      if ((frontIRdistance1 < 30 && frontIRdistance2 < 30) || (frontIRdistance1 < 15) || (frontIRdistance2 < 15)){
        moveBackward(32);      
      }
      else if (frontIRdistance1 < frontIRdistance2){
        roboclaw.BackwardM1(address, 32);
        roboclaw.ForwardM2(address, 32);
      }
      else{
        roboclaw.ForwardM1(address, 32);
        roboclaw.BackwardM2(address, 32);
      }
    }
    else if(uwb0 > 150) {
      // Move full speed if far away
      if (leftOrRight == 0) {
        changeSpeed(32, 64);
      }
      else if(leftOrRight == 1){
        changeSpeed(64,32); 
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

void moveForward(int motorSpeed){
  roboclaw.ForwardM1(address,motorSpeed);
  roboclaw.ForwardM2(address,motorSpeed);
}

void moveBackward(int motorSpeed){
  roboclaw.BackwardM1(address,motorSpeed);
  roboclaw.BackwardM2(address,motorSpeed);
}

void changeSpeed(int motorSpeed1, int motorSpeed2){
// Motor 1 = Left motor
// Motor 2 = Right motor
  roboclaw.ForwardM1(address,motorSpeed1);
  roboclaw.ForwardM2(address,motorSpeed2);
}

void moveUp(){
  myservo1.write(60);
  myservo2.write(0);
}

void moveDown(){
  myservo1.write(0);
  myservo2.write(60);
}

void displayspeed(void)
{
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  
  int32_t enc1= roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  int32_t speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
  Serial.print("Encoder1:");
  if(valid1){
    Serial.print(enc1);
    Serial.print(" ");
    Serial.print(status1);
    Serial.print(" ");
  }
  else{
    Serial.print("invalid ");
  }
  Serial.print("Encoder2:");
  if(valid2){
    Serial.print(enc2);
    Serial.print(" ");
    Serial.print(status2);
    Serial.print(" ");
  }
  else{
    Serial.print("invalid ");
  }
  Serial.print("Speed1:");
  if(valid3){
    Serial.print(speed1);
    Serial.print(" ");
  }
  else{
    Serial.print("invalid ");
  }
  Serial.print("Speed2:");
  if(valid4){
    Serial.print(speed2);
    Serial.print(" ");
  }
  else{
    Serial.print("invalid ");
  }
  Serial.println();
}
