#ifndef _sdcTopLidarSensor_hh
#define _sdcTopLidarSensor_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "sdcSensorData.hh"

namespace gazebo
{
    class GAZEBO_VISIBLE sdcTopLidarSensor : public SensorPlugin
    {
    public:
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
        void OnUpdate();

    private:
        sensors::RaySensorPtr parentSensor;
        event::ConnectionPtr updateConnection;

        LidarPos lidarPos;
    };
}

#endif
