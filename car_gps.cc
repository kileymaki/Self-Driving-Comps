#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "car_gps.hh"

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(CarGps)

sensors::GpsSensorPtr parentSensor;

void CarGps::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    // Get the parent sensor.
    this->parentSensor =
    boost::dynamic_pointer_cast<sensors::GpsSensor>(_sensor);
    
    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "Couldn't find a gps\n";
        return;
    }
    
    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&CarGps::OnUpdate, this));
    
    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);
}

// Called by the world update start event
void CarGps::OnUpdate(){
    math::Angle lat = this->parentSensor->Latitude();
    math::Angle lon = this->parentSensor->Longitude();
    
    printf("LAT:\t%f\nLON:\t%f\n\n",lat.Degree(), lon.Degree());
}


// Pointer to the update event connection
event::ConnectionPtr updateConnection;
