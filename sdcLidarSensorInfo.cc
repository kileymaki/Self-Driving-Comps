//This class provides a central point for access to lidar sensor info.

#include <stdio.h>
#include "sdcLidarSensorInfo.hh"

sdcLidarSensorInfo::sdcLidarSensorInfo(){}

sdcLidarSensorInfo::sdcLidarSensorInfo(sdcAngle minAngle, double resolution, double maxRange, int numRays){
    this->minAngle = minAngle;
    this->resolution = resolution;
    this->maxRange = maxRange;
    this->numRays = numRays;
    this->lastUpdate = 0;
}
