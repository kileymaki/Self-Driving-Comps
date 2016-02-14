#include <stdio.h>
#include "Waypoint.hh"


enum WaypointType {
  // Waypoint to visit, lowest priority
  WaypointType_Target,
  // Waypoints created programmatically, should be given higher priority
  WaypointType_DriveStraight,
  WaypointType_TurnLeft,
  WaypointType_TurnRight,
  WaypointType_Stop
};



Waypoint::Waypoint(int waypointType, math::Vector2d pos) {
    this->waypointType = WaypointType_DriveStraight;
    this->pos = pos;
}
