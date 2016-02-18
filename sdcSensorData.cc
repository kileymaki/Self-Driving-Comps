#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "sdcSensorData.hh"

using namespace gazebo;

// Angle information for each lidar
sdcAngle sdcSensorData::frontMinAngle = sdcAngle(0);
double sdcSensorData::frontAngleResolution = 0;
int sdcSensorData::frontLidarLastUpdate = 0;

sdcAngle sdcSensorData::backMinAngle = sdcAngle(0);
double sdcSensorData::backAngleResolution = 0;

// The rays from each lidar
std::vector<double>* sdcSensorData::frontLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::backLidarRays = new std::vector<double>();

std::vector<double>* sdcSensorData::topLeftLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::topRightLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::topForwardLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::topBackwardLidarRays = new std::vector<double>();

std::vector<double>* sdcSensorData::sideLeftFrontLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::sideLeftBackLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::sideRightFrontLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::sideRightBackLidarRays = new std::vector<double>();

// Camera variables
bool sdcSensorData::stopSignInLeftCamera = false;
bool sdcSensorData::stopSignInRightCamera = false;
int sdcSensorData::lanePosition = 0;

/*
 * Initializes the lidar in the given position to store its minimum angle, as well as the resolution
 */
void sdcSensorData::InitLidar(LidarPos lidar, double minAngle, double angleResolution){
    switch (lidar) {
        case FRONT:
        frontMinAngle = sdcAngle(minAngle);
        frontAngleResolution = angleResolution;
        std::cout << "MIN ANGLE\t" << minAngle << "\t" << frontMinAngle << "\t" << frontAngleResolution << std::endl;
        break;

        case BACK:
        backMinAngle = sdcAngle(minAngle);
        backAngleResolution = angleResolution;
        break;

        case TOP_FORWARD:
        break;

        case TOP_RIGHT:
        break;

        case TOP_BACKWARD:
        break;

        case TOP_LEFT:
        break;

        case SIDE_LEFT_FRONT:
        break;

        case SIDE_LEFT_BACK:
        break;

        case SIDE_RIGHT_FRONT:
        break;

        case SIDE_RIGHT_BACK:
        break;

        default:
        // The given enum matches more than one lidar sensor, to be safe we'll ignore it
        break;
    }
}

/*
 * Updates the lidar in the given position to hold the given rays
 */
void sdcSensorData::UpdateLidar(LidarPos lidar, std::vector<double>* newRays){
    switch (lidar) {
        case FRONT:
        frontLidarRays = newRays;
        frontLidarLastUpdate = (frontLidarLastUpdate + 1) % 100000000;
        break;

        case BACK:
        backLidarRays = newRays;
        break;

        case TOP_FORWARD:
        topForwardLidarRays = newRays;
        break;

        case TOP_RIGHT:
        topRightLidarRays = newRays;
        break;

        case TOP_BACKWARD:
        topBackwardLidarRays = newRays;
        break;

        case TOP_LEFT:
        topLeftLidarRays = newRays;
        break;

        case SIDE_LEFT_FRONT:
        sideLeftFrontLidarRays = newRays;
        break;

        case SIDE_LEFT_BACK:
        sideLeftBackLidarRays = newRays;
        break;

        case SIDE_RIGHT_FRONT:
        sideRightFrontLidarRays = newRays;
        break;

        case SIDE_RIGHT_BACK:
        sideRightBackLidarRays = newRays;
        break;

        default:
        // The given enum matches more than one lidar sensor, to be safe we'll ignore it
        break;
    }
}

/*
 * Retrieve any camera data that we might find helpful
 */
void sdcSensorData::UpdateCameraData(int lanePos) {
    lanePosition = lanePos;
}

int sdcSensorData::LanePosition() {
    return lanePosition;
}

/*
 * Retrieve a copy of the rays for the lidar in the given position
 */
std::vector<double> sdcSensorData::GetLidarRays(LidarPos lidar){
    std::vector<double> lidarRaysCopy;

    switch (lidar) {
        case FRONT:
        lidarRaysCopy = (*frontLidarRays);
        break;

        case BACK:
        lidarRaysCopy = (*backLidarRays);
        break;

        case TOP:
        lidarRaysCopy.insert(lidarRaysCopy.end(),topForwardLidarRays->begin(),topForwardLidarRays->end());
        lidarRaysCopy.insert(lidarRaysCopy.end(),topRightLidarRays->begin(),topRightLidarRays->end());
        lidarRaysCopy.insert(lidarRaysCopy.end(),topBackwardLidarRays->begin(),topBackwardLidarRays->end());
        lidarRaysCopy.insert(lidarRaysCopy.end(),topLeftLidarRays->begin(),topLeftLidarRays->end());
        break;

        case SIDE_LEFT:
        lidarRaysCopy.insert(lidarRaysCopy.end(),sideLeftBackLidarRays->begin(),sideLeftBackLidarRays->end());
        lidarRaysCopy.insert(lidarRaysCopy.end(),sideLeftFrontLidarRays->begin(),sideLeftFrontLidarRays->end());
        break;

        case SIDE_RIGHT:
        lidarRaysCopy.insert(lidarRaysCopy.end(),sideRightFrontLidarRays->begin(),sideRightFrontLidarRays->end());
        lidarRaysCopy.insert(lidarRaysCopy.end(),sideRightBackLidarRays->begin(),sideRightBackLidarRays->end());
        break;

        case TOP_FORWARD:
        lidarRaysCopy = (*topForwardLidarRays);
        break;

        case TOP_RIGHT:
        lidarRaysCopy = (*topRightLidarRays);
        break;

        case TOP_BACKWARD:
        lidarRaysCopy = (*topBackwardLidarRays);
        break;

        case TOP_LEFT:
        lidarRaysCopy = (*topLeftLidarRays);
        break;

        case SIDE_LEFT_FRONT:
        lidarRaysCopy = (*sideLeftFrontLidarRays);
        break;

        case SIDE_LEFT_BACK:
        lidarRaysCopy = (*sideLeftBackLidarRays);
        break;

        case SIDE_RIGHT_FRONT:
        lidarRaysCopy = (*sideRightFrontLidarRays);
        break;

        case SIDE_RIGHT_BACK:
        lidarRaysCopy = (*sideRightBackLidarRays);
        break;
    }

    return lidarRaysCopy;
}

/*
 * Return a vector of pairs (ray angle, ray length) which represents objects in view of front lidar
 */
std::vector<sdcLidarRay> sdcSensorData::GetBlockedFrontRays(){
    std::vector<sdcLidarRay> objectsInFront;
    for (int i = 0; i < frontLidarRays->size(); i++) {
        if (!std::isinf((*frontLidarRays)[i])) {
            sdcAngle angle = sdcAngle(frontMinAngle + i*frontAngleResolution);
            // std::cout << "GetBlocked\t" << i << "\t" << angle << std::endl;
            objectsInFront.push_back(sdcLidarRay(angle, (*frontLidarRays)[i]));
        }
    }
    return objectsInFront;
}

/*
 * Return a vector of pairs (ray angle, ray length) which represents objects in view of back lidar
 */
std::vector<sdcLidarRay> sdcSensorData::GetBlockedBackRays(){
    std::vector<sdcLidarRay> objectsInBack;
    for (int i = 0; i < backLidarRays->size(); i++) {
        if (!std::isinf((*backLidarRays)[i])) {
            sdcAngle angle = sdcAngle(i*backAngleResolution+backMinAngle);
            objectsInBack.push_back(sdcLidarRay(angle, (*backLidarRays)[i]));
        }
    }
    return objectsInBack;
}

/*
 * Returns a vector of pairs that each contain a pair with the minimum and maximum angle
 * corresponding to an object in the front lidar, and the minimum distance the object is
 * away
 */
std::vector<sdcVisibleObject> sdcSensorData::GetObjectsInFront(){
    std::vector<sdcVisibleObject> objectList;

    std::vector<sdcLidarRay> blockedRays = GetBlockedFrontRays();
    if(blockedRays.size() == 0) return objectList;

    double distMargin = 0.5;
    double angleMargin = 0.005;

    bool ignorePrev = true;

    sdcAngle objMinAngle;
    double objFirstDist;

    double objMinDist;

    sdcAngle prevAngle;
    double prevDist;

    for (int i = 0; i < blockedRays.size(); i++) {
        sdcAngle curAngle = blockedRays[i].angle;
        double curDist = blockedRays[i].dist;

        if(!ignorePrev){
            objMinDist = curDist < objMinDist ? curDist : objMinDist;

            if(!((curAngle - prevAngle).withinMargin(angleMargin) && fabs(curDist - prevDist) < distMargin)){
                std::cout << "AddedOjbect1\t" << objMinAngle << "\t" << curAngle << std::endl;
                objectList.push_back(sdcVisibleObject(sdcLidarRay(objMinAngle, objFirstDist), sdcLidarRay(prevAngle, prevDist), objMinDist));
                ignorePrev = true;
            }
        }else{
            ignorePrev = false;
            objMinAngle = curAngle;
            objMinDist = curDist;
            objFirstDist = curDist;
        }

        prevAngle = curAngle;
        prevDist = curDist;
    }

    std::cout << "AddedOjbect2\t" << objMinAngle << "\t" << prevAngle << std::endl;
    objectList.push_back(sdcVisibleObject(sdcLidarRay(objMinAngle, objFirstDist), sdcLidarRay(prevAngle, prevDist), objMinDist));
    return objectList;
}

// New gps system using 2d vector
double sdcSensorData::gpsX = 0;
double sdcSensorData::gpsY = 0;
sdcAngle sdcSensorData::gpsYaw = sdcAngle(0);

void sdcSensorData::UpdateGPS(double x, double y, double yaw){
    gpsX = x;
    gpsY = y;
    gpsYaw = sdcAngle(yaw);
}

math::Vector2d sdcSensorData::GetPosition(){
    return math::Vector2d(gpsX, gpsY);
}

sdcAngle sdcSensorData::GetYaw(){
    return gpsYaw;
}
