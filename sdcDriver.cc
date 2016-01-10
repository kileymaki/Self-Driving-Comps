#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "sdcDriver.hh"
#include <vector>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(sdcDriver)

sdcDriver::sdcDriver(sdcCar *_car)
{
    this->car = _car;
}