#include "angle.hh"

const double PI = 3.14159265359;

//Negative Y angles are bigger than postive Y angles
angle::angle(double a){
  Angle = fmod(a,(2*PI));
}
angle::angle(){
  Angle = 0;
}

bool angle::isFrontFacing(){
  return this->Angle >= 3*PI/2 || this->Angle <= PI/2;
}

bool angle::withinMargin(double x){
  // Not to be used with angles x<0 or x>PI
  return this->Angle >= 2*PI-x || this->Angle <= x;
}

bool angle::isBackFacing(){
  return this->Angle < 3*PI/2 && this->Angle > PI/2;
}

angle operator+(const angle& a, const angle& b){
  return angle(a.Angle + b.Angle);
}

 angle operator-(const angle& a, const angle& b){
  return angle(a.Angle - b.Angle);
}

bool operator<(const angle& a, const angle& b){
  return (a.Angle-b.Angle) > PI;
}

bool operator>(const angle& a, const angle& b){
  return (a.Angle-b.Angle) < PI && (a.Angle-b.Angle) > 0;
}

bool operator==(const angle& a, const angle& b){
  return a.Angle == b.Angle;
}
