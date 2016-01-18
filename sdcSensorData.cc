#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "sdcSensorData.hh"

using namespace gazebo;

bool sdcSensorData::isAllInfVar = true;
std::vector<double>* sdcSensorData::anglesNotAtInf = new std::vector<double>();
double sdcSensorData::rayRange = std::numeric_limits<double>::infinity();

std::vector<double>* sdcSensorData::lidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::leftLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::rightLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::forwardLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::backwardLidarRays = new std::vector<double>();

void sdcSensorData::UpdateLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    
    lidarRays = newRays;
    
    unsigned int rayCount = lidarRays->size();
    
    rayRange = (*lidarRays)[320];
//    printf("Ray Range: %f\n", rayRange);
    
    isAllInfVar = true;
    anglesNotAtInf->clear();
    for (unsigned int i = 0; i < rayCount; ++i)
    {
        if(!std::isinf((*lidarRays)[i])){
            isAllInfVar = false;
            anglesNotAtInf->push_back(minAngle.operator+(*new math::Angle(i*angleResolution)).Radian());
        }
    }
}

void sdcSensorData::UpdateLeftLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    leftLidarRays = newRays;
}
void sdcSensorData::UpdateRightLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    rightLidarRays = newRays;
}
void sdcSensorData::UpdateForwardLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    forwardLidarRays = newRays;
}
void sdcSensorData::UpdateBackwardLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    backwardLidarRays = newRays;
}

bool sdcSensorData::IsAllInf(){
    return isAllInfVar;
}

std::vector<double>* sdcSensorData::GetNonInfAngles(){
    return anglesNotAtInf;
}

std::vector<double> sdcSensorData::GetLidarRays(){
    std::vector<double> lidarRaysCopy = (*lidarRays);
    return lidarRaysCopy;
}

double sdcSensorData::GetRangeInFront(){
    return rayRange;
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