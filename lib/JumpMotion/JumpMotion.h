#pragma once
#include <Arduino.h>
#include <Vector3.h>
#include<vector>
using namespace std;

class JumpMotion{
private:
    std::vector<Vector3> motion_array;
    std::vector<int> motion_time;
    bool playing_state;
    int state_num;
    unsigned long s_time;
    Vector3 ret_pos;
public:
    JumpMotion();
    JumpMotion(std::vector<Vector3>& motion_array_,std::vector<int>& motion_time_);
    void update(std::vector<Vector3>& motion_array_,std::vector<int>& motion_time_);
    bool playing();
    void start();
    Vector3 getPos();
};
