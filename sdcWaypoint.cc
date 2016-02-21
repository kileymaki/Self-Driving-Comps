//This class provides a structure for waypoint objects used in waypointdriving found in sdcCar

#include <stdio.h>
#include "sdcWaypoint.hh"


enum WaypointType {
  WaypointType_DriveStraight,
  WaypointType_TurnLeft,
  WaypointType_TurnRight,
  WaypointType_Stop
};

sdcWaypoint::sdcWaypoint() {
    this->waypointType = 0;
    this->pos = std::make_pair(0,0);
}

sdcWaypoint::sdcWaypoint(int waypointType, std::pair<double,double> pos) {
    this->waypointType = waypointType;
    this->pos = pos;
}
