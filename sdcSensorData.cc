#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "sdcSensorData.hh"

using namespace gazebo;

bool sdcSensorData::frontIsAllInfVar = true;
std::vector<double>* sdcSensorData::frontAnglesNotAtInf = new std::vector<double>();
double sdcSensorData::frontRayRange = std::numeric_limits<double>::infinity();

bool sdcSensorData::leftIsAllInfVar = true;
std::vector<double>* sdcSensorData::leftAnglesNotAtInf = new std::vector<double>();
double sdcSensorData::leftRayRange = std::numeric_limits<double>::infinity();

bool sdcSensorData::rightIsAllInfVar = true;
std::vector<double>* sdcSensorData::rightAnglesNotAtInf = new std::vector<double>();
double sdcSensorData::rightRayRange = std::numeric_limits<double>::infinity();

bool sdcSensorData::forwardIsAllInfVar = true;
std::vector<double>* sdcSensorData::forwardAnglesNotAtInf = new std::vector<double>();
double sdcSensorData::forwardRayRange = std::numeric_limits<double>::infinity();

bool sdcSensorData::backwardIsAllInfVar = true;
std::vector<double>* sdcSensorData::backwardAnglesNotAtInf = new std::vector<double>();
double sdcSensorData::backwardRayRange = std::numeric_limits<double>::infinity();

std::vector<double>* sdcSensorData::frontLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::leftLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::rightLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::forwardLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::backwardLidarRays = new std::vector<double>();

void sdcSensorData::UpdateFrontLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){

    frontLidarRays = newRays;

    unsigned int frontRayCount = frontLidarRays->size();

    frontRayRange = (*frontLidarRays)[320];

    frontIsAllInfVar = true;
    frontAnglesNotAtInf->clear();
    for (unsigned int i = 0; i < frontRayCount; ++i)
    {
        if(!std::isinf((*frontLidarRays)[i])){
            frontIsAllInfVar = false;
            frontAnglesNotAtInf->push_back(minAngle.operator+(*new math::Angle(i*angleResolution)).Radian());
        }
    }
}

void sdcSensorData::UpdateLeftLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    leftLidarRays = newRays;

    unsigned int leftRayCount = leftLidarRays->size();

    leftRayRange = (*leftLidarRays)[320];

    leftIsAllInfVar = true;
    leftAnglesNotAtInf->clear();
    for (unsigned int i = 0; i < leftRayCount; ++i)
    {
        if(!std::isinf((*leftLidarRays)[i])){
            leftIsAllInfVar = false;
            leftAnglesNotAtInf->push_back(minAngle.operator+(*new math::Angle(i*angleResolution)).Radian());
        }
    }
}
void sdcSensorData::UpdateRightLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    rightLidarRays = newRays;

    unsigned int rightRayCount = rightLidarRays->size();

    rightRayRange = (*rightLidarRays)[320];

    rightIsAllInfVar = true;
    rightAnglesNotAtInf->clear();
    for (unsigned int i = 0; i < rightRayCount; ++i)
    {
        if(!std::isinf((*rightLidarRays)[i])){
            rightIsAllInfVar = false;
            rightAnglesNotAtInf->push_back(minAngle.operator+(*new math::Angle(i*angleResolution)).Radian());
        }
    }
}
void sdcSensorData::UpdateForwardLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    forwardLidarRays = newRays;

    forwardLidarRays = newRays;

    unsigned int forwardRayCount = forwardLidarRays->size();

    forwardRayRange = (*forwardLidarRays)[320];

    forwardIsAllInfVar = true;
    forwardAnglesNotAtInf->clear();
    for (unsigned int i = 0; i < forwardRayCount; ++i)
    {
        if(!std::isinf((*forwardLidarRays)[i])){
            forwardIsAllInfVar = false;
            forwardAnglesNotAtInf->push_back(minAngle.operator+(*new math::Angle(i*angleResolution)).Radian());
        }
    }
}
void sdcSensorData::UpdateBackwardLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    backwardLidarRays = newRays;

    unsigned int backwardRayCount = backwardLidarRays->size();

    backwardRayRange = (*backwardLidarRays)[320];

    backwardIsAllInfVar = true;
    backwardAnglesNotAtInf->clear();
    for (unsigned int i = 0; i < backwardRayCount; ++i)
    {
        if(!std::isinf((*backwardLidarRays)[i])){
            backwardIsAllInfVar = false;
            backwardAnglesNotAtInf->push_back(minAngle.operator+(*new math::Angle(i*angleResolution)).Radian());
        }
    }
}

bool sdcSensorData::FrontIsAllInf(){
    return frontIsAllInfVar;
}

std::vector<double>* sdcSensorData::FrontGetNonInfAngles(){
    return frontAnglesNotAtInf;
}

std::vector<double> sdcSensorData::GetFrontLidarRays(){
    std::vector<double> frontLidarRaysCopy = (*frontLidarRays);
    return frontLidarRaysCopy;
}

std::vector<double> sdcSensorData::GetTopLidarRays(){
    std::vector<double> topLidarRaysCopy;

    topLidarRaysCopy.insert(topLidarRaysCopy.end(),forwardLidarRays->begin(),forwardLidarRays->end());
    topLidarRaysCopy.insert(topLidarRaysCopy.end(),rightLidarRays->begin(),rightLidarRays->end());
    topLidarRaysCopy.insert(topLidarRaysCopy.end(),backwardLidarRays->begin(),backwardLidarRays->end());
    topLidarRaysCopy.insert(topLidarRaysCopy.end(),leftLidarRays->begin(),leftLidarRays->end());
    return topLidarRaysCopy;
}

double sdcSensorData::GetRangeInFront(){
    return frontRayRange;
}


// New gps system using 2d vector
math::Vector2d sdcSensorData::coordinate;

void sdcSensorData::UpdateGPS(math::Vector2d newCoordinate){
    coordinate = newCoordinate;
}

math::Vector2d sdcSensorData::GetCurrentCoord(){
    return coordinate;
}

double sdcSensorData::GetLatitude(){
    return coordinate[1];
}

double sdcSensorData::GetLongitude(){
    return coordinate[0];
}

/* Old coordinate system
math::Angle* sdcSensorData::targetLon = new math::Angle(0);
math::Angle* sdcSensorData::lat = new math::Angle(0);
math::Angle* sdcSensorData::lon = new math::Angle(0);

 void sdcSensorData::UpdateGPS(math::Angle* newLat, math::Angle* newLon){
 lat = newLat;
 lon = newLon;
 }

double sdcSensorData::GetLongitude(){
    return lon->Degree();
}

double sdcSensorData::GetLatitude(){
    return lat->Degree();
}
*/
