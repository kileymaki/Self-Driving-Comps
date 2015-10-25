//
//  laser_car.hh
//  
//
//  Created by selfcar on 10/25/15.
//
//

#ifndef _laser_car_hh
#define _laser_car_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
    class GAZEBO_VISIBLE CarLaser : public SensorPlugin
    {
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
        
    public: void OnUpdate();
        
    public: static bool IsAllInf();
        
    private: sensors::RaySensorPtr parentSensor;
    private: event::ConnectionPtr updateConnection;
        
    private: static bool isAllInfVar;
    };
}

#endif
