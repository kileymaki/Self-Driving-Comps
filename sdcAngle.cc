#include <stdio.h>
#include "sdcAngle.hh"

const double PI = 3.14159265359;

//Negative Y angles are bigger than postive Y angles
sdcAngle::sdcAngle(double a){
  angle = fmod(a,(2*PI));
  if (angle < 0.0) {
    angle += 2.0 * PI;
  }
}

sdcAngle::sdcAngle(){
  angle = 0;
}

bool sdcAngle::IsFrontFacing(){
  return this->angle >= 3*PI/2 || this->angle <= PI/2;
}

bool sdcAngle::WithinMargin(double x){
  // Not to be used with angles x<0 or x>PI
  return this->angle >= 2*PI-x || this->angle <= x;
}

sdcAngle sdcAngle::FindMargin(sdcAngle a){
    return sdcAngle(fabs(this->angle - a.angle));
}

sdcAngle sdcAngle::GetMidAngle(sdcAngle a){
    sdcAngle sum = *this + a;
    double avg;
    if(sum.angle > PI){
        avg = (sum.angle - 2 * PI) / 2.;
    }else{
        avg = sum.angle / 2.;
    }
    return sdcAngle(avg);
}

bool sdcAngle::IsBackFacing(){
  return this->angle < 3*PI/2 && this->angle > PI/2;
}

sdcAngle operator+(const sdcAngle& a, const sdcAngle& b){
  return sdcAngle(a.angle + b.angle);
}

sdcAngle operator+(const sdcAngle& a, const double& b){
  return sdcAngle(a.angle + b);
}

sdcAngle operator+(const double& a, const sdcAngle& b){
  return sdcAngle(a + b.angle);
}

sdcAngle operator-(const sdcAngle& a, const sdcAngle& b){
  return sdcAngle(a.angle - b.angle);
}

sdcAngle operator-(const sdcAngle& a, const double& b){
 return sdcAngle(a.angle - b);
}

sdcAngle operator/(const sdcAngle& a, const double& b){
  return sdcAngle(a.angle / b);
}

bool operator<(const sdcAngle& a, const sdcAngle& b){
  return (a.angle-b.angle) > PI;
}

bool operator<(const sdcAngle& a, const double& b){
  return a.angle < b;
}

bool operator>(const sdcAngle& a, const sdcAngle& b){
  return (a.angle-b.angle) < PI && (a.angle-b.angle) > 0;
}

bool operator>(const sdcAngle& a, const double& b){
  return a.angle > b;
}

bool operator==(const sdcAngle& a, const sdcAngle& b){
  return a.angle == b.angle;
}

std::ostream& operator<<(std::ostream& os, const sdcAngle& a){
  os << a.angle;
  return os;
}
