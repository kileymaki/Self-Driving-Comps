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
        public: static bool FrontIsAllInf();
        public: static std::vector<double>* FrontGetNonInfAngles();
        public: static std::vector<double> GetFrontLidarRays();
        public: static double GetRangeInFront();
        public: static std::vector<double> GetTopLidarRays();
        public: static std::vector<std::pair<int,double>> GetObjectsInFront();
        private: static double frontRayRange;
        private: static bool frontIsAllInfVar;
        private: static std::vector<double>* frontAnglesNotAtInf;
        private: static double leftRayRange;
        private: static bool leftIsAllInfVar;
        private: static std::vector<double>* leftAnglesNotAtInf;
        private: static double rightRayRange;
        private: static bool rightIsAllInfVar;
        private: static std::vector<double>* rightAnglesNotAtInf;
        private: static double forwardRayRange;
        private: static bool forwardIsAllInfVar;
        private: static std::vector<double>* forwardAnglesNotAtInf;
        private: static double backwardRayRange;
        private: static bool backwardIsAllInfVar;
        private: static std::vector<double>* backwardAnglesNotAtInf;

        private: static std::vector<double>* frontLidarRays;
        private: static std::vector<double>* leftLidarRays;
        private: static std::vector<double>* rightLidarRays;
        private: static std::vector<double>* forwardLidarRays;
        private: static std::vector<double>* backwardLidarRays;

    public:
        static bool stopSignInLeftCamera;
        static bool stopSignInRightCamera;

        public: static void UpdateFrontLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
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
