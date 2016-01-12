//
//  sdcGpsSensor.hh
//
//
//  Created by selfcar on 10/25/15.
//
//

#ifndef _sdcGpsSensor_hh
#define _sdcGpsSensor_hh

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
    class GAZEBO_VISIBLE sdcGpsSensor : public SensorPlugin
    {
        public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
        
        public: void OnUpdate();
        
        private: sensors::GpsSensorPtr parentSensor;
        private: event::ConnectionPtr updateConnection;
    };
}

#endif
