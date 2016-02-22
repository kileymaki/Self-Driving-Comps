/*
 * sdcAngle is a custom class used to wrap doubles as angle objects that are
 * easier to mathematically manipulate. The class sets a range for values from 0 to 2PI,
 * ensuring we have a concrete range of values to expect. The class also features quality
 * of life methods, both adding new methods as well as overloading basic functions.
 */


#include <stdio.h>
#include "sdcAngle.hh"

const double PI = 3.14159265359;

/*
 * Contrains the parametr to a value between 0 and 2*PI
 */
sdcAngle::sdcAngle(double a){
  angle = fmod(a,(2*PI));
  if (angle < 0.0) {
    angle += 2.0 * PI;
  }
}

sdcAngle::sdcAngle(){
  angle = 0;
}

/*
 * Returns true if this angle is in quadrants 1 or 4
 */
bool sdcAngle::IsFrontFacing(){
  return this->angle >= 3*PI/2 || this->angle <= PI/2;
}

/*
 * Returns true if the given angle has an angle size
 * of less than the given parameter. Both positive and negative
 * angles that match this criteria return true
 */
bool sdcAngle::WithinMargin(double x){
  if(x<0) return false;
  if(x>=PI) return true;
  return this->angle >= 2*PI-x || this->angle <= x;
}

/*
 * Gets the angle between this angle and the given
 * angle
 */
sdcAngle sdcAngle::FindMargin(sdcAngle a){
    if (fabs(this->angle - a.angle) > PI) {
        if (this->angle > a.angle) {
            return sdcAngle(fabs((this->angle - 2*PI) - a.angle));
        } else {
            return sdcAngle(fabs(this->angle - (a.angle - 2*PI)));
        }
    }
    return sdcAngle(fabs(this->angle - a.angle));
}

/*
 * Gets the angle directly between this angle and
 * the given angle
 */
sdcAngle sdcAngle::GetMidAngle(sdcAngle a){
    sdcAngle sum = *this + a;
    double avg;
    // In cases where the sum is greater than PI, the standard
    // average calculation fails and so instead use the negative
    // representation of the angles
    if(sum.angle > PI){
        avg = (sum.angle - 2 * PI) / 2.;
    }else{
        avg = sum.angle / 2.;
    }
    return sdcAngle(avg);
}

/*
 * Returns true if this angle is in quadrants 2 or 3
 */
bool sdcAngle::IsBackFacing(){
  return this->angle < 3*PI/2 && this->angle > PI/2;
}

///////////////////////////
// OVERWRITTEN OPERATORS //
///////////////////////////

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
