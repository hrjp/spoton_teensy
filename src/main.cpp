#include <Arduino.h>
#include<ps3i2clib.h>
#include<Servo.h>
//#include "Vector.h"
//#include "PID_lib.h"
#include <Kinematics.hpp>
#include <SpotServo.hpp>

//Servo s1;
PS3I2C ps(0x74);
SpotServo FL_Shoulder;
void setup() {
  Wire.begin();
  Wire.setSDA(34);
  Wire.setSCL(33);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  //s1.attach(2);
  //s1.write(0);
  double shoulder_liedown = 0.0;
  FL_Shoulder.Initialize(2, 90+ shoulder_liedown , 90, -81.0, FL, Shoulder, 800, 2100,-90.0,90.0); // 0 | 0: STRAIGHT | 90: OUT | -90 IN
}

void loop() {
  ps.update();
  int a=map(ps.A_Ry(),0,255,-90,90);

  FL_Shoulder.update_clk();
  FL_Shoulder.SetGoal(a,300.0);
  //s1.write(a);
  Serial.println(FL_Shoulder.GetPoseEstimate());
  // put your main code here, to run repeatedly:
}