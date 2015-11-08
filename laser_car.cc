#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "laser_car.hh"

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(CarLaser)

bool CarLaser::isAllInfVar = true;
std::vector<double>* CarLaser::anglesNotAtInf = new std::vector<double>();
sensors::RaySensorPtr parentSensor;

void CarLaser::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    // Get the parent sensor.
    this->parentSensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
    
    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "Couldn't find a laser also where does this go?\n";
        return;
    }
    
    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&CarLaser::OnUpdate, this));
    
    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);
}
    
// Called by the world update start event
void CarLaser::OnUpdate(){
    isAllInfVar = true;
    
    math::Angle minAngle = this->parentSensor->AngleMin();
    double angleResolution = this->parentSensor->GetAngleResolution();
    
    int rayCount = this->parentSensor->GetRayCount();
    
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
    
bool CarLaser::IsAllInf(){
    return isAllInfVar;
}

std::vector<double>* CarLaser::GetNonInfAngles(){
    return anglesNotAtInf;
}
