//This class wraps the lidar sensor data into a single class.
//Also provides methods to calculate the x and y distance relative to the car.

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
    return sin(this->angle.angle) * dist * (-1);
}

double sdcLidarRay::GetLongitudinalDist(){
    return cos(this->angle.angle) * dist;
}

gazebo::math::Vector2d sdcLidarRay::GetAsPoint(){
    return gazebo::math::Vector2d(this->GetLateralDist(), this->GetLongitudinalDist());
}
