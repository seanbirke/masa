//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial

// Connect pins 10 and 11 to signal 1 and 2 labeled S1 and S2 first columns on the Roboclaw Motor shield
// Make sure to ground by connected ground pin to S5 "-" column
SoftwareSerial serial(10,11);	

RoboClaw roboclaw(&serial,10000);

#define address 0x80

//Velocity PID coefficients.
#define Kp 1.0 // Can change values
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

int uwb0 = 0;
int uwb1 = 0;
int ratioAnglesToCounts = 100;
int timingWindow = 500;
int headingAngle;

void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(9600); // Make sure in serial monitor to change the baud rate to see the data
  roboclaw.begin(38400);
  
  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps);  
}

void loop() {
  headingAngle = random(-15, 15);

  roboclaw.ResetEncoders(address);

  if (Serial.available() > 0) {
    // uwb0 distance
    String data = Serial.readStringUntil('\n').substring(2, 5);
    uwb0 = data.toInt();
    Serial.println(uwb0);
//    // uwb1 distance
//    data = Serial.readStringUntil('\n').substring(2, 5);
//    uwb1 = data.toInt();
//    Serial.println(uwb1);
//    // heading angle of tag
//    data = Serial.readStringUntil('\n').substring(2, 5);
//    headingAngle = data.toInt();
//    Serial.println(headingAngle);
  }
  
  if (headingAngle < -5 || headingAngle > 5){
      changeAngle(headingAngle);
  }
  if (uwb0 > 300){
    // Move at full speed if far away
    moveForward(127);
  }
  else if (uwb0 > 100 && uwb0 <= 300){
    // Move at 75% speed if nearby
    moveForward(96);

  }
  else{
    moveForward(0); 
  }
//  displayspeed();
//  Serial.print("Heading Angle: ");
//  Serial.print(headingAngle);
//  delay(timingWindow);
  
//  roboclaw.SpeedM2(address,2000);


//  roboclaw.SpeedDistanceM1M2(address, speed1, distance1, speed2, distance2, flag)
  
//  for(uint8_t i = 0;i<100;i++){
//    displayspeed();
//    delay(10);
//  }
//
//  roboclaw.SpeedM1(address,M1000);
//  for(uint8_t i = 0;i<100;i++){
//    displayspeed();
//    delay(10);
//  }

//  roboclaw.SpeedAccelDeccelPositionM1(address,0,12000,0,11000,1);
//  roboclaw.SpeedAccelDeccelPositionM1(address,0,12000,0,1000,0);
//  roboclaw.SpeedAccelDeccelPositionM1(address,32000,12000,32000,11000,0);
//  roboclaw.SpeedAccelDeccelPositionM1(address,32000,12000,32000,1000,0);
//  long last = millis();
//  while(millis()-last<5000){
//    displayspeed();
//    delay(50);
//  }
//
//  roboclaw.ForwardM1(address,64); //start Motor1 forward at half speed; range of values is 0-127, with 0 being stopped and 127 full speed
//  roboclaw.BackwardM2(address,64); //start Motor2 backward at half speed
//  delay(2000);
//
//  roboclaw.BackwardM1(address,64);
//  roboclaw.ForwardM2(address,64);
//  delay(2000);
//
//  roboclaw.ForwardBackwardM1(address,96); //start Motor1 forward at half speed
//  roboclaw.ForwardBackwardM2(address,32); //start Motor2 backward at half speed
//  delay(2000);
//
//  roboclaw.ForwardBackwardM1(address,32);
//  roboclaw.ForwardBackwardM2(address,96);
//  delay(2000);
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
  
  if (angle < 0){
    // Continues moving forward while angling left
    roboclaw.SpeedAccelDeccelPositionM1M2(address, 0, speed1-500, 0, enc1 + encoderCounts, 0, speed2, 0, enc2 + encoderCounts, 0);
  }
  else{
    // Continues moving forward while angling right
    roboclaw.SpeedAccelDeccelPositionM1M2(address, 0, speed1, 0, enc1 + encoderCounts, 0, speed2-500, 0, enc2 + encoderCounts, 0);
  }
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
