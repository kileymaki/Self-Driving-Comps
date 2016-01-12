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

void sdcSensorData::UpdateLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    
    lidarRays = newRays;
    
    unsigned int rayCount = lidarRays->size();
    
    rayRange = (*lidarRays)[320];
    printf("Ray Range: %f\n", rayRange);
    
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

bool sdcSensorData::IsAllInf(){
    return isAllInfVar;
}

std::vector<double>* sdcSensorData::GetNonInfAngles(){
    return anglesNotAtInf;
}

double sdcSensorData::GetRangeInFront(){
    return rayRange;
}


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