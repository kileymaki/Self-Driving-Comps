#ifndef _sdcLidarSensorInfo_hh_
#define _sdcLidarSensorInfo_hh_

#include "sdcAngle.hh"

class sdcLidarSensorInfo {
public:
    sdcLidarSensorInfo();
    sdcLidarSensorInfo(sdcAngle minAngle, double resolution, double maxRange, int numRays);

    sdcAngle minAngle;
    double resolution;
    double maxRange;
    int numRays;
    int lastUpdate;
};

#endif
