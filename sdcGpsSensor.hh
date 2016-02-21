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
    class GAZEBO_VISIBLE sdcGpsSensor : public ModelPlugin
    {
        public:
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void OnUpdate();
            math::Pose pose;

        private:
            physics::LinkPtr gpsLink;
            sensors::GpsSensorPtr parentSensor;
            std::vector<event::ConnectionPtr> connections;
    };
}

#endif
