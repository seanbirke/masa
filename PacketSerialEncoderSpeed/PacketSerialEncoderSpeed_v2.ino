//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial

// Connect pins 10 and 11 to signal 1 and 2 labeled S1 and S2 first columns on the Roboclaw Motor shield
// Make sure to ground by connected ground pin to S5 "-" column
// SoftwareSerial serialPi(14,15);
// SoftwareSerial serial(10,11);	

RoboClaw roboclaw(&Serial2,10000);

#define address 0x80

//Velocity PID coefficients.
#define Kp 1.0 // Can change values
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

int uwb0 = 0;
int uwb1 = 0;
int headingAngle;
int moveLeft;
int ratioAnglesToCounts = 33;
int timingWindow = 500;


unsigned long time0;
unsigned long time1;
unsigned long time2;
unsigned long time3;
unsigned long time4;
unsigned long time5;
float angleParam = 0.5;

bool angleStatus = false;

void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(9600); // Make sure in serial monitor to change the baud rate to see the data
  
//  serialPi.begin(9600);
  roboclaw.begin(38400);
  
  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps);  

  headingAngle = 50;
}

void loop() {
//  headingAngle = random(-15, 15);
//  uwb0 = 400;
  
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
    headingAngle = data.toInt();
//    Serial.print(headingAngle);
//    data = Serial.readStringUntil('\n');
//    moveLeft = data.toInt();
//    Serial.println(moveLeft);
  }

//  if (headingAngle > -50){
//    headingAngle = headingAngle - 1;
//    delay(100);
//  }
//  else{
//    headingAngle = 50;
//  }
//  Serial.println(headingAngle);

//  roboclaw.ResetEncoders(address);
  if (headingAngle < -5 || headingAngle > 5){
    angleStatus = true;
  }
  else{
    angleStatus = false;
  }
  
  if (uwb0 > 300){
    // Move at full speed if far away
    if (angleStatus ==true && headingAngle > 5){
      changeSpeed(127, 64);
//      Serial.println("Moving Right...");
    }
    else if (angleStatus == true && headingAngle < -5){
      changeSpeed(64,127);
//      Serial.println("Moving Left...");
    }
    else{
      moveForward(127);
//      Serial.println("Moving Forward...");
    }
  }
  else if (uwb0 > 100 && uwb0 <= 300){
    // Move at 75% speed if nearby
    if (angleStatus ==true && headingAngle > 5){
      changeSpeed(96, 64);
    }
    else if (angleStatus == true && headingAngle < -5){
      changeSpeed(64,96);
    }
    else{
      moveForward(96);
    }
  }
  else{
    moveForward(0); 
  }
//  displayspeed();
//  Serial.print("Heading Angle: ");
//  Serial.print(headingAngle);
//  delay(timingWindow);
}

void moveForward(int motorSpeed){
  roboclaw.ForwardM1(address,motorSpeed);
  roboclaw.ForwardM2(address,motorSpeed);
}

void changeAngle(int angle){
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  
  int32_t enc1= roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(address, &status2, &valid2);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status3, &valid3);
  int32_t speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
  
  int encoderCounts = angle*ratioAnglesToCounts;

  if (angle > 0){
    // Continues moving forward while angling left
//    roboclaw.SpeedAccelDeccelPositionM1M2(address, 0, 1000, 0, enc1, 0, 4000, 0, enc2 + round(angleParam*ratioAnglesToCounts), 0);
    roboclaw.ForwardM1(address,64);
    roboclaw.ForwardM2(address,128);
//    Serial.print("Moving left");
  }
  else{
    // Continues moving forward while angling right
//    roboclaw.SpeedAccelDeccelPositionM1M2(address, 0, 4000, 0, enc1 + round(angleParam*ratioAnglesToCounts), 0, 1000, 0, enc2, 0);
    roboclaw.ForwardM1(address,128);
    roboclaw.ForwardM2(address,64);
  }
}

void changeSpeed(int motorSpeed1, int motorSpeed2){
  // Continues moving forward while angling left
//  if (motorSpeed1 < motorSpeed2){
//    for (int i = motorSpeed2; i >= 64; i--) {
//      roboclaw.ForwardM1(address,i);
//      roboclaw.ForwardM2(address,motorSpeed2);
//    }
//  }
//  if (motorSpeed2 < motorSpeed1){
//    for (int i = motorSpeed1; i >= 64; i--) {
//      roboclaw.ForwardM1(address,motorSpeed1);
//      roboclaw.ForwardM2(address,i);
//    }
//  }
  roboclaw.ForwardM1(address,motorSpeed1);
  roboclaw.ForwardM2(address,motorSpeed2);
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
