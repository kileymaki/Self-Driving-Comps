#ifndef _Angle_hh_
#define _Angle_hh_
#include <math.h>
#include <iostream>

class Angle {
public:
    Angle(double a);
    Angle();
    double angle;
    bool isFrontFacing();
    bool isBackFacing();
    bool withinMargin(double x);
    friend Angle operator+(const Angle& a, const Angle& b);
    friend Angle operator+(const Angle& a, const double& b);
    friend Angle operator+(const double& a, const Angle& b);
    friend Angle operator-(const Angle& a, const Angle& b);
    friend Angle operator-(const Angle& a, const double& b);
    friend bool operator<(const Angle& a, const Angle& b);
    friend bool operator>(const Angle& a, const Angle& b);
    friend bool operator==(const Angle& a, const Angle& b);
    friend std::ostream& operator<<(std::ostream& os, const Angle& a);
};

#endif
