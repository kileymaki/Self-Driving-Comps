#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "sdcLaserSensor.hh"

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(sdcLaserSensor)

bool sdcLaserSensor::isAllInfVar = true;
std::vector<double>* sdcLaserSensor::anglesNotAtInf = new std::vector<double>();
sensors::RaySensorPtr parentSensor;

void sdcLaserSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    // Get the parent sensor.
    this->parentSensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
    
    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "Couldn't find a laser\n";
        return;
    }
    
    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&sdcLaserSensor::OnUpdate, this));
    
    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);
}
    
// Called by the world update start event
void sdcLaserSensor::OnUpdate(){
    isAllInfVar = true;
    
    math::Angle minAngle = this->parentSensor->AngleMin();
    double angleResolution = this->parentSensor->GetAngleResolution();
    
    int rayCount = this->parentSensor->GetRayCount();
    
    double rayRange = this->parentSensor->GetRange(100);
    printf("Ray Range: %f\n", rayRange);
    
    anglesNotAtInf->clear();
    for (unsigned int i = 0; i < rayCount; ++i)
    {
        if(!std::isinf(this->parentSensor->GetRange(i))){
            isAllInfVar = false;
            anglesNotAtInf->push_back(minAngle.operator+(*new math::Angle(i*angleResolution)).Radian());
        }
    }
}

    
    // Pointer to the update event connection
event::ConnectionPtr updateConnection;
    
bool sdcLaserSensor::IsAllInf(){
    return isAllInfVar;
}

std::vector<double>* sdcLaserSensor::GetNonInfAngles(){
    return anglesNotAtInf;
}
