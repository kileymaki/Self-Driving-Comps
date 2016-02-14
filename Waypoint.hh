#ifndef _Waypoint_hh_
#define _Waypoint_hh_
#include <math.h>
#include <cmath>
#include <iostream>
#include <vector>

class Waypoint
{
public:
  int waypointType;
  math::Vector2d pos;

  Waypoint(int waypointType,math::Vector2d pos);
};


#endif
