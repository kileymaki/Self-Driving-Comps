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
#include "sdcAngle.hh"
#include "sdcLidarRay.hh"

namespace gazebo
{
    // An enumeration of all positions of Lidar sensors, including a few (TOP, SIDE_LEFT, SIDE_RIGHT) that
    // correspond to a collection of sensors
    enum LidarPos {FRONT, BACK, TOP, TOP_FORWARD, TOP_RIGHT, TOP_BACKWARD, TOP_LEFT, SIDE_LEFT, SIDE_RIGHT, SIDE_LEFT_FRONT, SIDE_LEFT_BACK, SIDE_RIGHT_FRONT, SIDE_RIGHT_BACK};

    class sdcSensorData
    {
        // Lidar variables and methods
    public:
        static void InitLidar(LidarPos lidar, double minAngle, double angleResolution);
        static void UpdateLidar(LidarPos lidar, std::vector<double>* newRays);
        static std::vector<double> GetLidarRays(LidarPos lidar);
        static void UpdateCameraData(int lanePos);
        static int LanePosition();

        static std::vector<sdcLidarRay> GetBlockedFrontRays();
        static std::vector<sdcLidarRay> GetBlockedBackRays();
        static std::vector<std::pair<sdcLidarRay, sdcLidarRay>> GetObjectsInFront();

        static sdcAngle frontMinAngle;
        static double frontAngleResolution;

        static sdcAngle backMinAngle;
        static double backAngleResolution;

    private:
        static std::vector<double>* frontLidarRays;
        static std::vector<double>* backLidarRays;

        static std::vector<double>* topLeftLidarRays;
        static std::vector<double>* topRightLidarRays;
        static std::vector<double>* topForwardLidarRays;
        static std::vector<double>* topBackwardLidarRays;

        static std::vector<double>* sideLeftFrontLidarRays;
        static std::vector<double>* sideLeftBackLidarRays;
        static std::vector<double>* sideRightFrontLidarRays;
        static std::vector<double>* sideRightBackLidarRays;

    public:
        static bool stopSignInLeftCamera;
        static bool stopSignInRightCamera;
        static int lanePosition;

        // GPS variables and methods
        static math::Vector2d GetCurrentCoord();
        static double GetLongitude();
        static double GetLatitude();
        //private: static math::Angle* targetLon;
        //private: static math::Angle* lat;
        //private: static math::Angle* lon;
        static math::Vector2d coordinate;

        //public: static void UpdateGPS(math::Angle* newLat, math::Angle* newLon);
        static void UpdateGPS(math::Vector2d newCoordinate);
    };
}

#endif
