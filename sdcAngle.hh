#ifndef _sdcAngle_hh_
#define _sdcAngle_hh_

#include <math.h>
#include <iostream>

class sdcAngle {
public:
    sdcAngle(double a);
    sdcAngle();
    double angle;
    bool IsFrontFacing();
    bool IsBackFacing();
    bool WithinMargin(double x);
    sdcAngle FindMargin(sdcAngle a);
    sdcAngle GetMidAngle(sdcAngle a);
    friend sdcAngle operator+(const sdcAngle& a, const sdcAngle& b);
    friend sdcAngle operator+(const sdcAngle& a, const double& b);
    friend sdcAngle operator+(const double& a, const sdcAngle& b);
    friend sdcAngle operator-(const sdcAngle& a, const sdcAngle& b);
    friend sdcAngle operator-(const sdcAngle& a, const double& b);
    friend sdcAngle operator/(const sdcAngle& a, const double& b);
    friend bool operator<(const sdcAngle& a, const sdcAngle& b);
    friend bool operator<(const sdcAngle& a, const double& b);
    friend bool operator>(const sdcAngle& a, const sdcAngle& b);
    friend bool operator>(const sdcAngle& a, const double& b);
    friend bool operator==(const sdcAngle& a, const sdcAngle& b);
    friend std::ostream& operator<<(std::ostream& os, const sdcAngle& a);
};

#endif
