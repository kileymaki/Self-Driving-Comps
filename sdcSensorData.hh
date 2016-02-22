#ifndef _sdcSensorData_hh
#define _sdcSensorData_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include <map>
#include "sdcAngle.hh"
#include "sdcLidarSensorInfo.hh"
#include "sdcLidarRay.hh"
#include "sdcVisibleObject.hh"

namespace gazebo
{
    // An enumeration of all positions of Lidar sensors, including a few (TOP, SIDE_LEFT, SIDE_RIGHT) that
    // correspond to a collection of sensors
    enum LidarPos {FRONT, BACK, TOP, TOP_FORWARD, TOP_RIGHT, TOP_BACKWARD, TOP_LEFT, SIDE_LEFT, SIDE_RIGHT, SIDE_LEFT_FRONT, SIDE_LEFT_BACK, SIDE_RIGHT_FRONT, SIDE_RIGHT_BACK};

    class sdcSensorData
    {
        // Lidar variables and methods
    public:
        static void InitLidar(LidarPos lidar, double minAngle, double angleResolution, double maxRange, int numRays);
        static void UpdateLidar(LidarPos lidar, std::vector<double>* newRays);
        static std::vector<double> GetLidarRays(LidarPos lidar);
        static void UpdateCameraData(int lanePos);
        static int LanePosition();
        static void UpdateSteeringMagnitude(double steerMag);

        static double GetNewSteeringMagnitude();

        static std::vector<sdcLidarRay> GetBlockedFrontRays();
        static std::vector<sdcLidarRay> GetBlockedBackRays();
        static std::vector<sdcVisibleObject> GetObjectsInFront();

        static int GetLidarLastUpdate(LidarPos lidar);
        static int GetLidarNumRays(LidarPos lidar);
        static sdcAngle GetLidarMinAngle(LidarPos lidar);
        static double GetLidarAngleResolution(LidarPos lidar);
        static double GetLidarMaxRange(LidarPos lidar);

        static sdcAngle backMinAngle;
        static double backAngleResolution;

        static int frontLidarLastUpdate;

        static int stopSignFrameCount;
        static double sizeOfStopSign;
        static bool stopSignInLeftCamera;
        static bool stopSignInRightCamera;
        static int lanePosition;
        static double newSteerMagnitude;

        // GPS variables and methods
        static double gpsX;
        static double gpsY;
        static sdcAngle gpsYaw;

        static math::Vector2d GetPosition();
        static sdcAngle GetYaw();
        static void UpdateGPS(double x, double y, double yaw);

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

        static std::map<LidarPos, sdcLidarSensorInfo> lidarInfoMap;
    };
}

#endif
