//
//  sdcLaserSensor.hh
//  
//
//  Created by selfcar on 10/25/15.
//
//

#ifndef _sdcLaserSensor_hh
#define _sdcLaserSensor_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

namespace gazebo
{
    class GAZEBO_VISIBLE sdcLaserSensor : public SensorPlugin
    {
        public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
            
        public: void OnUpdate();
            
        public: static bool IsAllInf();
        public: static std::vector<double>* GetNonInfAngles();
        public: static double GetRangeInFront();
        
        private: static double rayRange;
        
        private: sensors::RaySensorPtr parentSensor;
        private: event::ConnectionPtr updateConnection;
            
        private: static bool isAllInfVar;
            
        private: static std::vector<double>* anglesNotAtInf;
    };
}

#endif
