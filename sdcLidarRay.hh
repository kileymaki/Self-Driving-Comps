#ifndef _sdcLidarRay_hh_
#define _sdcLidarRay_hh_

#include "sdcAngle.hh"

class sdcLidarRay {
public:
    sdcLidarRay();
    sdcLidarRay(sdcAngle angle, double dist);

    double GetLateralDist();
    double GetLongitudinalDist();

    sdcAngle angle;
    double dist;
};

#endif
