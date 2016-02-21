//Class that handles registering and updating the top lidar sensor array.

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "sdcTopLidarSensor.hh"

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(sdcTopLidarSensor)

// Pointer to the update event connection
event::ConnectionPtr updateConnection;
sensors::RaySensorPtr parentSensor;

////// LIDAR ON TOP OF CAR
void sdcTopLidarSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
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
    this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&sdcTopLidarSensor::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);


    std::string name = this->parentSensor->GetName();
    if(name == "top_forward_laser"){
        this->lidarPos = TOP_FORWARD;
    } else if(name == "top_right_laser"){
        this->lidarPos = TOP_RIGHT;
    } else if(name == "top_backward_laser"){
        this->lidarPos = TOP_BACKWARD;
    } else if(name == "top_left_laser"){
        this->lidarPos = TOP_LEFT;
    }

    sdcSensorData::InitLidar(this->lidarPos, this->parentSensor->AngleMin().Radian(), this->parentSensor->GetAngleResolution(), this->parentSensor->GetRangeMax(), this->parentSensor->GetRayCount());
}

// Called by the world update start event
void sdcTopLidarSensor::OnUpdate(){
    //std::cout << this->parentSensor->GetName() << std::endl;
    std::vector<double>* rays = new std::vector<double>();
    for (unsigned int i = 0; i < this->parentSensor->GetRayCount(); ++i){
        rays->push_back(this->parentSensor->GetRange(i));
    }

    sdcSensorData::UpdateLidar(this->lidarPos, rays);
}
