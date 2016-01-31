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

// Pointer to the update event connection
event::ConnectionPtr updateConnection;
sensors::RaySensorPtr parentSensor;

////// LIDAR ON FRONT OF CAR
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

    std::vector<double>* rays = new std::vector<double>();
    for (unsigned int i = 0; i < this->parentSensor->GetRayCount(); ++i){
        rays->push_back(this->parentSensor->GetRange(i));
    }
    sdcSensorData::UpdateFrontLidar(this->parentSensor->AngleMin(), this->parentSensor->GetAngleResolution(), rays);
}
