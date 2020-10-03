#include"LegMotion.h"
#include <Arduino.h>
#include <math.h>

LegMotion::LegMotion(double min_rps_){
    min_rps=min_rps_;
}

double LegMotion::getRad(double rps){
    //double code=rps>0?1.0:-1.0;
    //return (long(abs(rps)*double(millis())/1000.0*36000)%36000)*0.1*DEG_TO_RAD*code;
    double dt=(micros()-pretime)/1000000.0;
    pretime=micros();
    nowrad+=rps*dt;
    if(nowrad>1.0){
        nowrad=0.0;
    }
    else if(nowrad<0.0){
        nowrad=1.0;
    }
    return nowrad*2.0*M_PI;
}

Vector3 LegMotion::getLegPositon(double rps,double radius,double direction,double phase){
    Vector3 pos;
    
    if(abs(rps)<min_rps){
        return pos;
    }
    pos.z=radius*sin(getRad(rps)+phase*DEG_TO_RAD);
    pos.x=radius*cos(getRad(rps)+phase*DEG_TO_RAD);//*cos(radius*DEG_TO_RAD);
    pos.y=0;//radius*cos(getRad(rps)+phase*DEG_TO_RAD)*sin(radius*DEG_TO_RAD);
    return pos;
}

Vector3 LegMotion::getLegPositon2(double rps,double radius,double groundtime,double direction,double phase=0.0){
    phase/=360.0;
    double t=getRad(rps)/(M_PI*2.0)+phase;
    if(t>1.0){
        t-=1.0;
    }
    double t1=groundtime;
    Vector3 pos;
    if(abs(rps)<min_rps){
        return pos;
    }
    if(t<t1){
        pos.z=0;
        pos.x=2.0*radius*t/t1-radius;
    }
    else{
        pos.x=radius*cos((t-t1)/(1-t1)*M_PI);
        pos.z=-radius*sin((t-t1)/(1-t1)*M_PI)*2.0;
    }
    return pos;


}
