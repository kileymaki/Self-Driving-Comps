#include <stdio.h>
#include "sdcWaypoint.hh"


enum WaypointType {
  WaypointType_DriveStraight,
  WaypointType_TurnLeft,
  WaypointType_TurnRight,
  WaypointType_Stop
};



sdcWaypoint::sdcWaypoint(int waypointType, std::pair<double,double> pos) {
    this->waypointType = WaypointType_DriveStraight;
    this->pos = pos;
}
