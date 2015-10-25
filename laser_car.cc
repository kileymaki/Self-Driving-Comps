#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include "laser_car.hh"

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(CarLaser)

void CarLaser::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
    {
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
void CarLaser::OnUpdate()
    {
        int rayCount = this->parentSensor->GetRayCount();
        for (unsigned int i = 0; i < rayCount; ++i)
        {
            if(!std::isinf(this->parentSensor->GetRange(i))){
                isAllInfVar = false;
                return;
            }
        }
        isAllInfVar = true;
    }

bool CarLaser::isAllInfVar = true;
sensors::RaySensorPtr parentSensor;
    
    // Pointer to the update event connection
event::ConnectionPtr updateConnection;
    
bool CarLaser::IsAllInf(){
    return isAllInfVar;
}