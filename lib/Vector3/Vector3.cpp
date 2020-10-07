#include"Vector3.h"

Vector3::Vector3(){}

Vector3::Vector3(double x_,double y_,double z_){
    x=x_;
    y=y_;
    z=z_;
}

Vector3 Vector3::operator*(double cons){
    Vector3 ans;
    ans.x=cons*x;
    ans.y=cons*y;
    ans.z=cons*z;
    return ans;
}