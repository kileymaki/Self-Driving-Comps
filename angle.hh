#ifndef _Angle_hh_
#define _Angle_hh_
#include <math.h>

class Angle {
public:
    Angle(double a);
    Angle();
    double angle;
    bool isFrontFacing();
    bool isBackFacing();
    bool withinMargin(double x);
    friend Angle operator+(const Angle& a, const Angle& b);
    friend Angle operator-(const Angle& a, const Angle& b);
    friend bool operator<(const Angle& a, const Angle& b);
    friend bool operator>(const Angle& a, const Angle& b);
    friend bool operator==(const Angle& a, const Angle& b);
};

#endif
