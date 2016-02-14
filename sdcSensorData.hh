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
#include "Angle.hh"

namespace gazebo
{
    class sdcSensorData
    {
        // Lidar variables and methods
        public: static bool FrontIsAllInf();
        public: static std::vector<double>* FrontGetNonInfAngles();
        public: static std::vector<double> GetFrontLidarRays();
        public: static std::vector<double> GetBackLidarRays();
        public: static double GetRangeInFront();
        public: static std::vector<double> GetTopLidarRays();
        public: static std::vector<double> GetSideLidarRays();
        public: static std::vector<double> GetRightFrontSideRays();
        public: static std::vector<double> GetRightBackSideRays();
        public: static std::vector<double> GetLeftFrontSideRays();
        public: static std::vector<double> GetLeftBackSideRays();
        public: static std::vector<std::pair<Angle,double>> GetObjectsInFront();

        private: static double frontRayRange;
        private: static bool frontIsAllInfVar;
        private: static std::vector<double>* frontAnglesNotAtInf;
        static Angle frontMinAngle;
        static double frontAngleResolution;

        private: static double backRayRange;
        private: static bool backIsAllInfVar;
        private: static std::vector<double>* backAnglesNotAtInf;
        private: static double leftFrontRayRange;
        private: static bool leftFrontIsAllInfVar;
        private: static std::vector<double>* leftFrontAnglesNotAtInf;
        private: static double rightFrontRayRange;
        private: static bool rightFrontIsAllInfVar;
        private: static std::vector<double>* rightFrontAnglesNotAtInf;
        private: static double leftBackRayRange;
        private: static bool leftBackIsAllInfVar;
        private: static std::vector<double>* leftBackAnglesNotAtInf;
        private: static double rightBackRayRange;
        private: static bool rightBackIsAllInfVar;
        private: static std::vector<double>* rightBackAnglesNotAtInf;

        private: static std::vector<double>* frontLidarRays;
        private: static std::vector<double>* backLidarRays;
        private: static std::vector<double>* leftTopLidarRays;
        private: static std::vector<double>* rightTopLidarRays;
        private: static std::vector<double>* forwardTopLidarRays;
        private: static std::vector<double>* backwardTopLidarRays;
        static std::vector<double>* leftFrontSideLidarRays;
        static std::vector<double>* leftBackSideLidarRays;
        static std::vector<double>* rightFrontSideLidarRays;
        static std::vector<double>* rightBackSideLidarRays;

    public:
        static bool stopSignInLeftCamera;
        static bool stopSignInRightCamera;

        public: static void UpdateFrontLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        public: static void UpdateBackLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        public: static void UpdateLeftLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        public: static void UpdateRightLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        public: static void UpdateForwardLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        public: static void UpdateBackwardLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        static void UpdateSideLeftFrontLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        static void UpdateSideLeftBackLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        static void UpdateSideRightFrontLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
        static void UpdateSideRightBackLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);

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
