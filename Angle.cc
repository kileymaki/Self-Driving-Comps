#include <stdio.h>
#include "Angle.hh"

const double PI = 3.14159265359;

//Negative Y angles are bigger than postive Y angles
Angle::Angle(double a){
  angle = fmod(a,(2*PI));
  if (angle < 0.0) {
    angle += 2.0 * PI;
  }
}

Angle::Angle(){
  angle = 0;
}

bool Angle::isFrontFacing(){
  return this->angle >= 3*PI/2 || this->angle <= PI/2;
}

bool Angle::withinMargin(double x){
  // Not to be used with angles x<0 or x>PI
  return this->angle >= 2*PI-x || this->angle <= x;
}

bool Angle::isBackFacing(){
  return this->angle < 3*PI/2 && this->angle > PI/2;
}

Angle operator+(const Angle& a, const Angle& b){
  return Angle(a.angle + b.angle);
}

Angle operator+(const Angle& a, const double& b){
  return Angle(a.angle + b);
}

Angle operator+(const double& a, const Angle& b){
  return Angle(a + b.angle);
}

Angle operator-(const Angle& a, const Angle& b){
  return Angle(a.angle - b.angle);
}

Angle operator-(const Angle& a, const double& b){
 return Angle(a.angle - b);
}

bool operator<(const Angle& a, const Angle& b){
  return (a.angle-b.angle) > PI;
}

bool operator>(const Angle& a, const Angle& b){
  return (a.angle-b.angle) < PI && (a.angle-b.angle) > 0;
}

bool operator==(const Angle& a, const Angle& b){
  return a.angle == b.angle;
}

std::ostream& operator<<(std::ostream& os, const Angle& a){
  os << a.angle << std::endl;
  return os;
}
