//Class that handles registering and updating the gps sensor.

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "sdcGpsSensor.hh"

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(sdcGpsSensor);

void sdcGpsSensor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
    this->gpsLink = _model->GetLink(_sdf->Get<std::string>("gps"));
    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(boost::bind(&sdcGpsSensor::OnUpdate, this)));
}

// Called by the world update start event
void sdcGpsSensor::OnUpdate(){
    math::Pose pose = this->gpsLink->GetWorldPose();
    sdcSensorData::UpdateGPS(pose.pos.x, pose.pos.y, pose.rot.GetYaw());
}


// Pointer to the update event connection
event::ConnectionPtr updateConnection;
