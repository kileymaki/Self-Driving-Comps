/*
 * This class provides a central location for all data that sdcCar uses during simulation. It
 * collects data from our sensors and stores it, and provides getter and setter methods to
 * the car and sensors respectively. This class also handles converting sensor data into
 * more intuitive information, such as taking blocked sensor rays and determining what are
 * objects.
 */

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
std::map<LidarPos, sdcLidarSensorInfo> sdcSensorData::lidarInfoMap = std::map<LidarPos, sdcLidarSensorInfo>();

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
int sdcSensorData::stopSignFrameCount = 0;
double sdcSensorData::sizeOfStopSign = 0;
bool sdcSensorData::stopSignInLeftCamera = false;
bool sdcSensorData::stopSignInRightCamera = false;
int sdcSensorData::lanePosition = 0;
double sdcSensorData::newSteerMagnitude = 10;

/*
 * Initializes the lidar in the given position to store its minimum angle, as well as the resolution
 */
void sdcSensorData::InitLidar(LidarPos lidar, double minAngle, double angleResolution, double maxRange, int numRays){
    switch (lidar) {
        case TOP:
        case SIDE_LEFT:
        case SIDE_RIGHT:
        // Don't init for enums that correspond to more than one sensor
        break;
        default:
        lidarInfoMap[lidar] = sdcLidarSensorInfo(sdcAngle(minAngle), angleResolution, maxRange, numRays);
    }
}

/*
 * Updates the lidar in the given position to hold the given rays
 */
void sdcSensorData::UpdateLidar(LidarPos lidar, std::vector<double>* newRays){
    lidarInfoMap[lidar].lastUpdate = (lidarInfoMap[lidar].lastUpdate + 1) % 100000000;
    switch (lidar) {
        case FRONT:
        frontLidarRays = newRays;
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

void sdcSensorData::UpdateSteeringMagnitude(double steerMag) {
    newSteerMagnitude = steerMag;
}

double sdcSensorData::GetNewSteeringMagnitude() {
    return newSteerMagnitude;
}

/*
 * Get the last update tick for the given lidar
 */
int sdcSensorData::GetLidarLastUpdate(LidarPos lidar){
    return lidarInfoMap[lidar].lastUpdate;
}

/*
 * Get the number of rays for the given lidar
 */
int sdcSensorData::GetLidarNumRays(LidarPos lidar){
    return lidarInfoMap[lidar].numRays;
}

/*
 * Get the minimum angle for the given lidar
 */
sdcAngle sdcSensorData::GetLidarMinAngle(LidarPos lidar){
    return lidarInfoMap[lidar].minAngle;
}

/*
 * Get the angle between individual rays for the given lidar
 */
double sdcSensorData::GetLidarAngleResolution(LidarPos lidar){
    return lidarInfoMap[lidar].resolution;
}

/*
 * Get the range for the given lidar
 */
double sdcSensorData::GetLidarMaxRange(LidarPos lidar){
    return lidarInfoMap[lidar].maxRange;
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
            sdcAngle angle = sdcAngle(lidarInfoMap[FRONT].minAngle + i*lidarInfoMap[FRONT].resolution);
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
 * Returns a vector of objects in the front lidar constructed with their
 * left and right bounding lidar ray, as well as the minimum distance to the
 * object
 */
std::vector<sdcVisibleObject> sdcSensorData::GetObjectsInFront(){
    std::vector<sdcVisibleObject> objectList;

    // With no blocked rays, there are no objects to record
    std::vector<sdcLidarRay> blockedRays = GetBlockedFrontRays();
    if(blockedRays.size() == 0) return objectList;

    double distMargin = 1;
    double angleMargin = 0.01;

    sdcAngle objMinAngle;
    double objFirstDist;
    double objMinDist;

    sdcAngle prevAngle;
    double prevDist;
    bool ignorePrev = true;

    // Parse the blocked rays into separate objects
    for (int i = 0; i < blockedRays.size(); i++) {
        sdcAngle curAngle = blockedRays[i].angle;
        double curDist = blockedRays[i].dist;

        if(!ignorePrev){
            objMinDist = curDist < objMinDist ? curDist : objMinDist;

            // If either the checked angles or distance fall outside the margins, the rays are looking at a new object
            if(!((curAngle - prevAngle).WithinMargin(angleMargin) && fabs(curDist - prevDist) < distMargin)){
                // Record the object just found
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

    // Since objects are recorded on the trailing end of the loop, this will make sure the last object is properly added
    objectList.push_back(sdcVisibleObject(sdcLidarRay(objMinAngle, objFirstDist), sdcLidarRay(prevAngle, prevDist), objMinDist));
    return objectList;
}

// GPS variables
double sdcSensorData::gpsX = 0;
double sdcSensorData::gpsY = 0;
sdcAngle sdcSensorData::gpsYaw = sdcAngle(0);

/*
 * Update the gps information
 */
void sdcSensorData::UpdateGPS(double x, double y, double yaw){
    gpsX = x;
    gpsY = y;
    gpsYaw = sdcAngle(yaw);
}

/*
 * Get the current sensor readings for position
 */
math::Vector2d sdcSensorData::GetPosition(){
    return math::Vector2d(gpsX, gpsY);
}

/*
 * Get the current sensor readings for orientation
 */
sdcAngle sdcSensorData::GetYaw(){
    return gpsYaw;
}
