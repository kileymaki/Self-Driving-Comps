#ifndef _sdcAngle_hh_
#define _sdcAngle_hh_

#include <math.h>
#include <iostream>

class sdcAngle {
public:
    sdcAngle(double a);
    sdcAngle();
    double angle;
    bool isFrontFacing();
    bool isBackFacing();
    bool withinMargin(double x);
    friend sdcAngle operator+(const sdcAngle& a, const sdcAngle& b);
    friend sdcAngle operator+(const sdcAngle& a, const double& b);
    friend sdcAngle operator+(const double& a, const sdcAngle& b);
    friend sdcAngle operator-(const sdcAngle& a, const sdcAngle& b);
    friend sdcAngle operator-(const sdcAngle& a, const double& b);
    friend sdcAngle operator/(const sdcAngle& a, const double& b);
    friend bool operator<(const sdcAngle& a, const sdcAngle& b);
    friend bool operator>(const sdcAngle& a, const sdcAngle& b);
    friend bool operator==(const sdcAngle& a, const sdcAngle& b);
    friend std::ostream& operator<<(std::ostream& os, const sdcAngle& a);
};

#endif
