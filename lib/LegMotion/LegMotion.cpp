#include"LegMotion.h"
#include <Arduino.h>
#include <math.h>

LegMotion::LegMotion(double min_rps_){
    min_rps=min_rps_;
}

double LegMotion::getRad(double rps){
    double code=rps>0?1.0:-1.0;
    return (long(abs(rps)*millis()/1000.0*3600)%3600)*0.1*DEG_TO_RAD*code;
}

Vector3 LegMotion::getLegPositon(double rps,double radius,double direction,double phase){
    Vector3 pos;
    
    if(abs(rps)<min_rps){
        return pos;
    }
    pos.z=radius*sin(getRad(rps)+phase*DEG_TO_RAD);
    pos.x=radius*cos(getRad(rps)+phase*DEG_TO_RAD)*cos(radius*DEG_TO_RAD);
    pos.y=radius*cos(getRad(rps)+phase*DEG_TO_RAD)*sin(radius*DEG_TO_RAD);
    return pos;
}

