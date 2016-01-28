//
//  sdcSensorData.hh
//
//
//  Created by selfcar on 10/25/15.
//
//

#ifndef _sdcSensorData_hh
#define _sdcSensorData_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

namespace gazebo
{
    class sdcSensorData
    {
        // Lidar variables and methods
        public: static bool IsAllInf();
        public: static std::vector<double>* GetNonInfAngles();
        public: static std::vector<double> GetLidarRays();
        public: static double GetRangeInFront();
        private: static double rayRange;
        private: static bool isAllInfVar;
        private: static std::vector<double>* anglesNotAtInf;

        private: static std::vector<double>* lidarRays;
        private: static std::vector<double>* leftLidarRays;
        private: static std::vector<double>* rightLidarRays;
        private: static std::vector<double>* forwardLidarRays;
        private: static std::vector<double>* backwardLidarRays;

        public: static void UpdateLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        public: static void UpdateLeftLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        public: static void UpdateRightLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        public: static void UpdateForwardLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        public: static void UpdateBackwardLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);

        // GPS variables and methods
        public: static math::Vector2d GetCurrentCoord();
        public: static double GetLongitude();
        public: static double GetLatitude();
        //private: static math::Angle* targetLon;
        //private: static math::Angle* lat;
        //private: static math::Angle* lon;
        private: static math::Vector2d coordinate;

        //public: static void UpdateGPS(math::Angle* newLat, math::Angle* newLon);
        public: static void UpdateGPS(math::Vector2d newCoordinate);
    };
}

#endif
