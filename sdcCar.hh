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
#include "sdcSensorData.hh"
#include "sdcAngle.hh"
#include "sdcWaypoint.hh"


namespace gazebo {

    class GAZEBO_VISIBLE sdcCar : public ModelPlugin {
        // Constructor for sdcCar
        public: sdcCar();

        // These methods are called by Gazebo during the loading and initializing
        // stages of world building and populating
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Init();

        // Bound to Gazebo's world update, gets called every tick of the simulation
        private: void OnUpdate();

        // Holds the bound connection to Gazebo's update, necessary in order to properly
        // receive updates
        std::vector<event::ConnectionPtr> connections;

        // The Gazebo model representation of the car
        physics::ModelPtr model;
        // Contains the wheel joints that get operated on each tick for movement
        std::vector<physics::JointPtr> joints;
        // A link to the chassis of the car, mainly used for access to physics variables
        // related to the car's state
        physics::LinkPtr chassis;

        // The velocity of the car
        math::Vector3 velocity;

        // These variables are mostly set in the SDF for the car and relate to the
        // physical parameters of the vehicle
        double frontPower, rearPower;
        double maxSpeed;
        double wheelRadius;

        double steeringRatio;
        double tireAngleRange;

        double aeroLoad;
        double swayForce;

        //////////////////////////////////////////
        // Begin Non-Gazebo Related Definitions //
        //////////////////////////////////////////

        // The different states the car can be in. The logic and behavior of
        // the car will change depending on which state it's in, with various
        // sensor readings affecting the decision to transition states
        enum CarState { stop, waypoint, intersection, follow, avoidance, parking};

        // The different states the car can be in while performing a front
        // perpendicular park
        enum ParkingState { stopPark, frontPark, straightPark, backPark, donePark };

        ///////////////////////////
        // SDC-defined variables //
        ///////////////////////////

        // The current state of the car
        CarState currentState;
        ParkingState currentParkingState;

        double gas; //variable that accelerates the car
        double brake; //variable that brakes the car

        // Scalars for accelrating and braking
        double accelRate;
        double brakeRate;

        // Position/rotation variables
        double yaw;
        double lon;

        int waypointProgress;

        int atIntersection;
        int maxCarSpeed;
        double maxCarReverseSpeed;

        bool turning;
        bool reversing;
        sdcAngle targetDirection;
        double targetSteeringAmount;
        double steeringAmount;
        double targetSpeed;
        sdcAngle targetParkingAngle;
        bool parkingAngleSet;

        // for Follow
        double estimatedSpeed;
        double lastPosition;
        double currentPosition;

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

        /////////////////////////
        // SDC-defined methods //
        /////////////////////////

         // The 'Brain' Methods
        void Drive();
        void MatchTargetDirection();
        void MatchTargetSpeed();

        void DetectIntersection();

        // Driving algorithms
        void WalledDriving();
        void LanedDriving();
        void GridTurning();
        void TurnAround();
        void WaypointDriving(std::vector<sdcWaypoint> waypoints);
        void Follow();
        void PerpendicularPark();

        // Helper methods
        void frontLidarUpdate();
        std::vector<sdcWaypoint> generateWaypoints(sdcWaypoint dest);

        sdcAngle AngleToTarget(math::Vector2d target);
        static bool ObjectDirectlyAhead();

        bool IsMovingForwards();
        double GetSpeed();
        sdcAngle GetDirection();
        sdcAngle GetOrientation();

        // Control methods
        void Accelerate(double amt = 1, double rate = 1.0);
        void Brake(double amt = 1, double rate = 1.0);
        void Stop();
        void Reverse();
        void StopReverse();

        void SetTargetDirection(sdcAngle direction);
        void SetTargetSteeringAmount(double a);
        void SetTargetSpeed(double s);

        void SetAccelRate(double rate = 1.0);
        void SetBrakeRate(double rate = 1.0);
    };
}
#endif
