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

#ifndef _sdcDriver_hh_
#define _sdcDriver_hh_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"
#include "sdcLaserSensor.hh"
#include "sdcGpsSensor.hh"
#include "sdcCar.hh"

namespace gazebo
{
    class sdcDriver
    {
        private: sdcCar *car;
        
        public: sdcDriver(sdcCar *_car);
        
        private: void CheckIfOnCollisionCourse();
        private: void DriveStraightThenStop();
        
        public: void OnUpdate();
    };
}
#endif
