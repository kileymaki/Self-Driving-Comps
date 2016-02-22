//This class wraps the lidar sensor data into a single class.
//Also provides methods to calculate the x and y distance relative to the car.

#include <stdio.h>
#include "sdcLidarRay.hh"

/*
 * Default constructor, sets angle and distance to zero
 */
sdcLidarRay::sdcLidarRay(){
    this->angle = sdcAngle(0);
    this->dist = 0;
}

/*
 * Constructs a simple lidar ray with the given angle and distance
 */
sdcLidarRay::sdcLidarRay(sdcAngle angle, double dist){
    this->angle = angle;
    this->dist = dist;
}

/*
 * Gets the lateral distance of this ray. When compared to the lidar
 * sensor producing these rays, negative values are to the left and
 * positive are to the right
 */
double sdcLidarRay::GetLateralDist(){
    return sin(this->angle.angle) * dist * (-1);
}

/*
 * Gets the longitudinal distance of this ray. When compared to the lidar sensor,
 * larger values are further away and smaller values are closer
 */
double sdcLidarRay::GetLongitudinalDist(){
    return cos(this->angle.angle) * dist;
}

/*
 * Wraps lateral and longitudinal distance into a point and returns it
 */
gazebo::math::Vector2d sdcLidarRay::GetAsPoint(){
    return gazebo::math::Vector2d(this->GetLateralDist(), this->GetLongitudinalDist());
}
