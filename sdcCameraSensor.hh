#ifndef _sdcCameraSensor_hh
#define _sdcCameraSensor_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include <iostream>

#include "sdcSensorData.hh"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace gazebo
{
    class GAZEBO_VISIBLE sdcCameraSensor : public SensorPlugin
    {
        public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
        public: void OnUpdate();

        private: sensors::MultiCameraSensorPtr parentSensor;
        private: event::ConnectionPtr updateConnection;
    };
}

#endif
