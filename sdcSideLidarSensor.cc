//Class that handles registering and updating the side lidar sensors

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "sdcSideLidarSensor.hh"

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(sdcSideLidarSensor)

// Pointer to the update event connection
event::ConnectionPtr updateConnection;
sensors::RaySensorPtr parentSensor;

////// LIDAR ON SIDE OF CAR
void sdcSideLidarSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
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
    this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&sdcSideLidarSensor::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);

    std::string name = this->parentSensor->GetName();
    if(name == "side_left_front_laser"){
        this->lidarPos = SIDE_LEFT_FRONT;
    } else if(name == "side_left_back_laser"){
        this->lidarPos = SIDE_LEFT_BACK;
    } else if(name == "side_right_front_laser"){
        this->lidarPos = SIDE_RIGHT_FRONT;
    } else if(name == "side_right_back_laser"){
        this->lidarPos = SIDE_RIGHT_BACK;
    }

    sdcSensorData::InitLidar(this->lidarPos, this->parentSensor->AngleMin().Radian(), this->parentSensor->GetAngleResolution(), this->parentSensor->GetRangeMax(), this->parentSensor->GetRayCount());
}

// Called by the world update start event
void sdcSideLidarSensor::OnUpdate(){
    std::vector<double>* rays = new std::vector<double>();
    for (unsigned int i = 0; i < this->parentSensor->GetRayCount(); ++i){
        rays->push_back(this->parentSensor->GetRange(i));
    }

    sdcSensorData::UpdateLidar(this->lidarPos, rays);
}
