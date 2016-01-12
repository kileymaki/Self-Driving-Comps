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
        public: static bool IsAllInf();
        public: static std::vector<double>* GetNonInfAngles();
        public: static double GetRangeInFront();
            
        private: static double rayRange;
            
        private: static bool isAllInfVar;
        private: static std::vector<double>* anglesNotAtInf;
        
        
        private: static std::vector<double>* lidarRays;
        public: static void UpdateLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays);
    };
}

#endif
