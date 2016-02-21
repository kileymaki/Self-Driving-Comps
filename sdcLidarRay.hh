#ifndef _sdcLidarRay_hh_
#define _sdcLidarRay_hh_

#include <gazebo/common/common.hh>
#include "sdcAngle.hh"

class sdcLidarRay {
public:
    sdcLidarRay();
    sdcLidarRay(sdcAngle angle, double dist);

    double GetLateralDist();
    double GetLongitudinalDist();
    gazebo::math::Vector2d GetAsPoint();

    sdcAngle angle;
    double dist;
};

#endif
