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
#include "Angle.hh"


namespace gazebo
{

    class GAZEBO_VISIBLE sdcCar : public ModelPlugin
    {
        /// \brief Constructor
         public: sdcCar();

         virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
         virtual void Init();

         private: void OnUpdate();

         void OnVelMsg(ConstPosePtr &_msg);

         std::vector<event::ConnectionPtr> connections;

         physics::ModelPtr model;
         physics::LinkPtr chassis;
         std::vector<physics::JointPtr> joints;
         physics::JointPtr gasJoint, brakeJoint;
         physics::JointPtr steeringJoint;

         math::Vector3 velocity;

         transport::NodePtr node;
         transport::SubscriberPtr velSub;

         double frontPower, rearPower;
         double maxSpeed;
         double wheelRadius;

         double steeringRatio;
         double tireAngleRange;
         double maxGas, maxBrake;

         double aeroLoad;
         double swayForce;

        /*
         * Begin Comps Defined Stuff
         */
         void frontLidarUpdate();
         void Drive();
         void Steer();
         void MatchTargetSpeed();


         void SetTargetDirection(Angle direction);
         void SetTargetSteeringAmount(double a);
         void SetTargetSpeed(double s);

         void ApplyMovementForce(double amt);
         void Accel(double amt = 0.5);
         void Brake(double amt = 1);

         bool IsMovingForwards();
         double GetSpeed();
         Angle GetDirection();
         void DetectIntersection();
         Angle AngleToTarget(math::Vector2d target);



         void DriveStraightThenStop();
         void WalledDriving();
         void DriveStraightThenTurn();
         void WaypointDriving(std::vector<math::Vector2d> waypoints);
         void Follow();



         double gas; //variable that accelerates the car
         double brake; //variable that brakes the car

         double yaw;
         double lon;

         int waypointProgress;

         int atIntersection;
         int maxCarSpeed;
         double maxCarReverseSpeed;

         bool turning;
         Angle targetDirection;
         double targetSteeringAmount;
         double steeringAmount;
         double targetSpeed;

         double estimatedSpeed;
         double lastPosition;
         double currentPosition;
         int speedCounter;
         double startTime;
         double endTime;

         double x;
         double y;

         //fl stands for Front Lidar
         std::vector<double> fl;
         int flRayLengths;
         int flCenterRight;
         int flSideRight;
         int flCenterLeft;
         int flSideLeft;
         int flNumRays;
         int flWeight;
         std::vector<std::vector<int>> flViews;

    };

    class Waypoint
    {
    public:
      int waypointType;

      Waypoint(int waypointType);
    };
}
#endif
