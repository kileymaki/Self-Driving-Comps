/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* Desc: A 4-wheeled vehicle
 * Author: Nate Koenig
 */

#ifndef _sdcCar_hh_
#define _sdcCar_hh_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"
#include "sdcLaserSensor.hh"
#include "sdcGpsSensor.hh"

namespace gazebo
{
    class GAZEBO_VISIBLE sdcCar : public ModelPlugin
    {
        /// \brief Constructor
        public: sdcCar();

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        public: virtual void Init();

        private: void OnUpdate();

        private: void OnVelMsg(ConstPosePtr &_msg);

        private: std::vector<event::ConnectionPtr> connections;

        private: physics::ModelPtr model;
        private: physics::LinkPtr chassis;
        private: std::vector<physics::JointPtr> joints;
        private: physics::JointPtr gasJoint, brakeJoint;
        private: physics::JointPtr steeringJoint;

        private: math::Vector3 velocity;

        private: transport::NodePtr node;
        private: transport::SubscriberPtr velSub;

        private: double frontPower, rearPower;
        private: double maxSpeed;
        private: double wheelRadius;

        private: double steeringRatio;
        private: double tireAngleRange;
        private: double maxGas, maxBrake;

        private: double aeroLoad;
        private: double swayForce;
        
        /*
         * Begin Comps Defined Stuff
         */
        private: void Drive();
        
        private: void ApplyMovementForce(double amt);
        private: void Accel(double amt = 0.5);
        private: void Brake(double amt = 1);
        private: void Steer(double angle);
        private: bool IsMovingForwards();
        private: void CheckIfOnCollisionCourse();

        private: double gas;
        private: double brake;
        private: double steeringAngle;
        private: double yaw;
        private: double lon;
    };
}
#endif
