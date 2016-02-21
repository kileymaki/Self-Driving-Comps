#ifndef _Waypoint_hh_
#define _Waypoint_hh_
#include <iostream>
#include <vector>

class sdcWaypoint
{
public:
  int waypointType;
  std::pair<double,double> pos;

  sdcWaypoint();
  sdcWaypoint(int waypointType, std::pair<double,double> pos);
};


#endif
