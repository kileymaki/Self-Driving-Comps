#include <stdio.h>
#include "sdcLidarRay.hh"

sdcLidarRay::sdcLidarRay(){
    this->angle = sdcAngle(0);
    this->dist = 0;
}

sdcLidarRay::sdcLidarRay(sdcAngle angle, double dist){
    this->angle = angle;
    this->dist = dist;
}

double sdcLidarRay::GetLateralDist(){
    return fabs(sin(this->angle.angle) * dist);
}

double sdcLidarRay::GetLongitudinalDist(){
    return fabs(cos(this->angle.angle) * dist);
}
