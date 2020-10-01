#include <Arduino.h>
#include<ps3i2clib.h>
#include<Servo.h>
//#include "Vector.h"
//#include "PID_lib.h"
#include"cout.h"
#include <Kinematics.hpp>
#include <SpotServo.hpp>

//Servo s1;
PS3I2C ps(0x74);

SpotServo FL_Shoulder, FL_Elbow, FL_Wrist;
double FL_angles[3]={0,0,0};
Kinematics ik;

void setup() {
  Wire.begin();
  Wire.setSDA(34);
  Wire.setSCL(33);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  //s1.attach(5);
  //s1.write(90);
  //s1.write(0);
  ik.Initialize(46.3, 130, 127.5);
  double shoulder_liedown = 0.0;
  FL_Shoulder.Initialize(32, 90+ shoulder_liedown , 90, -81.0, FL, Shoulder, 800, 2100,-90.0,90.0); // 0 | 0: STRAIGHT | 90: OUT | -90 IN
  double elbow_liedown = 0.0;
  FL_Elbow.Initialize(31, elbow_liedown, 0, -1.0, FL, Elbow, 800, 2100, -90.0, 90.0);        // 4 | 0: STRAIGHT | 90: BACK
  double wrist_liedown = 0.0;
  FL_Wrist.Initialize(30, wrist_liedown, 0, 3.0, FL, Wrist, 1600, 2400, -120.0, -60.0);    // 8 | 0: STRAIGHT | -90: FORWARD
}

//kata-hizi 46.3mm
//hizi-tekubi 130mm
//tekubi-saki 127.5mm

void loop() {
  ps.update();
  double a=map(ps.A_Ry(),0,255,90,220);
  double b=map(ps.A_Rx(),0,255,-50,50);
  //int a=map(ps.A_Ry(),0,255,-120,-60);
  FL_Shoulder.update_clk();
  FL_Elbow.update_clk();
  FL_Wrist.update_clk();
  //double domain=ik.GetDomain(a,0,0);
  //ik.LeftIK(a,0,0,domain,FL_angles);
  ik.GetJointAngles(a,b,0,Right,FL_angles);
  FL_angles[0]*=0*DEG_TO_RAD;
  FL_angles[1]+=-0*DEG_TO_RAD;
  FL_angles[2]+=0*DEG_TO_RAD;
  FL_Shoulder.SetGoal(FL_angles[0]*RAD_TO_DEG,1000.0);
  FL_Elbow.SetGoal(FL_angles[1]*RAD_TO_DEG,1000.0);
  FL_Wrist.SetGoal(-FL_angles[2]*RAD_TO_DEG,1000.0);
  
  cout<<"kata="<<FL_angles[0]*RAD_TO_DEG<<" hizi="<<FL_angles[1]*RAD_TO_DEG<<" tekubu="<<FL_angles[2]*RAD_TO_DEG<<","<<endl;
  //Serial.println(FL_Wrist.GetPoseEstimate());
  // put your main code here, to run repeatedly:
}