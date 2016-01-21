#ifndef _angle_hh_
#define _angle_hh_
#include <math.h>

class angle {
public:
    angle(double a);
    angle();
    double Angle;
    bool isFrontFacing();
    bool isBackFacing();
    bool withinMargin(double x);
    friend angle operator+(const angle& a, const angle& b);
    friend angle operator-(const angle& a, const angle& b);
    friend bool operator<(const angle& a, const angle& b);
    friend bool operator>(const angle& a, const angle& b);
    friend bool operator==(const angle& a, const angle& b);
};

#endif
