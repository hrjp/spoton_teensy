#include "JumpMotion.h"

JumpMotion::JumpMotion(){}

JumpMotion::JumpMotion(std::vector<Vector3>& motion_array_,std::vector<int>& motion_time_){
    motion_array=motion_array_;
    motion_time=motion_time_;
}

void JumpMotion::update(std::vector<Vector3>& motion_array_,std::vector<int>& motion_time_){
    motion_array=motion_array_;
    motion_time=motion_time_;
    if(playing_state){
        if((millis()-s_time)>motion_time[state_num]){
            state_num++;
            s_time=millis();
            Serial.print(motion_time[state_num]);
            Serial.print("num=");
            Serial.print(state_num);
            if(state_num<motion_array.size()){
                ret_pos=motion_array[state_num];
                Serial.print("BBB");
            }
            else{
                playing_state=false;
                Serial.print("CCC");
                state_num=0;
            }
            
        }
    }
}

void JumpMotion::start(){
    playing_state=true;
    ret_pos=motion_array[state_num];
    s_time=millis();
    state_num=0;
    Serial.print("DDD");
}

bool JumpMotion::playing(){
    return playing_state;
}

Vector3 JumpMotion::getPos(){
    return ret_pos;
}