#include <Arduino.h>
#include<ps3i2clib.h>
#include<Servo.h>
//#include "Vector.h"
//#include "PID_lib.h"
#include"cout.h"
#include <Vector3.h>
#include <Kinematics.hpp>
#include <SpotServo.hpp>
#include <LegMotion.h>


//Servo s1;
PS3I2C ps(0x74);

SpotServo FL_Shoulder, FL_Elbow, FL_Wrist;
SpotServo BL_Shoulder, BL_Elbow, BL_Wrist;
SpotServo FR_Shoulder, FR_Elbow, FR_Wrist;
SpotServo BR_Shoulder, BR_Elbow, BR_Wrist;
double FL_angles[3]={0,0,0};
double BL_angles[3]={0,0,0};
double FR_angles[3]={0,0,0};
double BR_angles[3]={0,0,0};
Kinematics ik;
LegMotion legmotion(0.5);

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
  BL_Shoulder.Initialize(29, 90+ shoulder_liedown , 90, -9.0, BL, Shoulder, 800, 2100,90.0,-90.0); // 0 | 0: STRAIGHT | 90: OUT | -90 IN
  FR_Shoulder.Initialize(26, 0+ shoulder_liedown , 0, -11, BL, Shoulder, 800, 2100,90.0,-90.0); // 0 | 0: STRAIGHT | 90: OUT | -90 IN
  BR_Shoulder.Initialize(12, 0+ shoulder_liedown , 0, -85, BL, Shoulder, 800, 2100,-90.0,90.0); // 0 | 0: STRAIGHT | 90: OUT | -90 IN
  double elbow_liedown = 0.0;
  FL_Elbow.Initialize(31, elbow_liedown, 0, -1.0, FL, Elbow, 800, 2100, -90.0, 90.0);        // 4 | 0: STRAIGHT | 90: BACK
  BL_Elbow.Initialize(28, elbow_liedown, 0, 0, FL, Elbow, 800, 2100, -90.0, 90.0);        // 4 | 0: STRAIGHT | 90: BACK
  FR_Elbow.Initialize(25, elbow_liedown, 0, 0, FL, Elbow, 800, 2100, -90.0, 90.0);        // 4 | 0: STRAIGHT | 90: BACK
  BR_Elbow.Initialize(11, elbow_liedown, 0, -1, FL, Elbow, 800, 2100, -90.0, 90.0);        // 4 | 0: STRAIGHT | 90: BACK
  double wrist_liedown = 0.0;
  FL_Wrist.Initialize(30, wrist_liedown, 0, 3.0, FL, Wrist, 1600, 2400, -120.0, -60.0);    // 8 | 0: STRAIGHT | -90: FORWARD
  BL_Wrist.Initialize(27, wrist_liedown, 0, -37.0, FL, Wrist, 1600, 2400, -120.0, -60.0);    // 8 | 0: STRAIGHT | -90: FORWARD
  FR_Wrist.Initialize(24, wrist_liedown, 0, 50, FL, Wrist, 1600, 2400, -60.0, -120.0);    // 8 | 0: STRAIGHT | -90: FORWARD
  BR_Wrist.Initialize(10, wrist_liedown, 0, 46, FL, Wrist, 1600, 2400, -60.0, -120.0);    // 8 | 0: STRAIGHT | -90: FORWARD
}

//kata-hizi 46.3mm
//hizi-tekubi 130mm
//tekubi-saki 127.5mm

void loop() {
  ps.update();
  FL_Shoulder.update_clk();
  FL_Elbow.update_clk();
  FL_Wrist.update_clk();
  BL_Shoulder.update_clk();
  BL_Elbow.update_clk();
  BL_Wrist.update_clk();
  FR_Shoulder.update_clk();
  FR_Elbow.update_clk();
  FR_Wrist.update_clk();
  BR_Shoulder.update_clk();
  BR_Elbow.update_clk();
  BR_Wrist.update_clk();
  
  double a=140;//map(ps.A_Ry(),0,255,90,220);
  double b=0;//map(ps.A_Rx(),0,255,100,-100);
  double c=50;//map(ps.A_Lx(),0,255,0,100);
  
  /*
  double a=map(ps.A_Ry(),0,255,90,220);
  double b=map(ps.A_Rx(),0,255,100,-100);
  double c=map(ps.A_Lx(),0,255,0,100);
  */
  double px=map(ps.A_Rx(),0,255,100,-100);
  double py=map(ps.A_Ry(),0,255,100,-100);
  double psize=sqrt(px*px+py*py);
  double pangle=atan2(-px,py);
  psize=0.05*py;
  int d=map(ps.A_Ry(),0,255,-180,180);
  const double omega=0.5;
  const double radius=40;
  //double sita=(long(omega*millis()/1000.0*3600)%3600)*0.1*DEG_TO_RAD;
  //double z=160+radius*sin(sita);
  //double x=0+radius*cos(sita);
  Vector3 FL_Motion=legmotion.getLegPositon(psize,radius,0,0);
  Vector3 BL_Motion=legmotion.getLegPositon(psize,radius,0,180);
  Vector3 FR_Motion=legmotion.getLegPositon(psize,radius,0,180);
  Vector3 BR_Motion=legmotion.getLegPositon(psize,radius,0,0);


  
  //double domain=ik.GetDomain(a,0,0);
  //ik.LeftIK(a,0,0,domain,FL_angles);
  ik.GetJointAngles(FL_Motion.x,FL_Motion.y+46.3,FL_Motion.z+160,Right,FL_angles);
  ik.GetJointAngles(BL_Motion.x,BL_Motion.y+46.3,BL_Motion.z+160,Right,BL_angles);
  ik.GetJointAngles(FR_Motion.x,-(FR_Motion.y+46.3),FR_Motion.z+160,Left,FR_angles);
  ik.GetJointAngles(BR_Motion.x,-(BR_Motion.y+46.3),BR_Motion.z+160,Left,BR_angles);
  //ik.GetJointAngles(b,c,a,Right,FL_angles);
  //ik.GetJointAngles(b,-c,a,Left,FR_angles);
  //ik.GetJointAngles(x,c,z,Right,FL_angles);
  FL_angles[0]+=180*DEG_TO_RAD;
  FL_angles[1]*=-1;//+=-0*DEG_TO_RAD;
  FL_angles[2]+=-180*DEG_TO_RAD;
  BL_angles[0]+=180*DEG_TO_RAD;
  BL_angles[1]*=-1;//+=-0*DEG_TO_RAD;
  BL_angles[2]+=-180*DEG_TO_RAD;
  FR_angles[0]+=180*DEG_TO_RAD;
  FR_angles[2]+=-180*DEG_TO_RAD;
  BR_angles[0]+=180*DEG_TO_RAD;
  BR_angles[2]+=-180*DEG_TO_RAD;

  FL_Shoulder.SetGoal(FL_angles[0]*RAD_TO_DEG,1000.0);
  FL_Elbow.SetGoal(FL_angles[1]*RAD_TO_DEG,1000.0);
  FL_Wrist.SetGoal(FL_angles[2]*RAD_TO_DEG,1000.0);
  BL_Shoulder.SetGoal(BL_angles[0]*RAD_TO_DEG,1000.0);
  BL_Elbow.SetGoal(BL_angles[1]*RAD_TO_DEG,1000.0);
  BL_Wrist.SetGoal(BL_angles[2]*RAD_TO_DEG,1000.0);
  FR_Shoulder.SetGoal(FR_angles[0]*RAD_TO_DEG,1000.0);
  FR_Elbow.SetGoal(FR_angles[1]*RAD_TO_DEG,1000.0);
  FR_Wrist.SetGoal(FR_angles[2]*RAD_TO_DEG,1000.0);
  BR_Shoulder.SetGoal(BR_angles[0]*RAD_TO_DEG,1000.0);
  BR_Elbow.SetGoal(BR_angles[1]*RAD_TO_DEG,1000.0);
  BR_Wrist.SetGoal(BR_angles[2]*RAD_TO_DEG,1000.0);

  cout<<"kata="<<FL_angles[0]*RAD_TO_DEG<<" hizi="<<FL_angles[1]*RAD_TO_DEG<<" tekubu="<<FL_angles[2]*RAD_TO_DEG<<",";
  //cout<<FL_Motion.x;
  //Serial.print(BR_Wrist.GetPoseEstimate());
  cout<<endl;
  // put your main code here, to run repeatedly:
}