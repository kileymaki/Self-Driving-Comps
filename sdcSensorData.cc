#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "sdcSensorData.hh"

using namespace gazebo;

bool sdcSensorData::frontIsAllInfVar = true;
std::vector<double>* sdcSensorData::frontAnglesNotAtInf = new std::vector<double>();
double sdcSensorData::frontRayRange = std::numeric_limits<double>::infinity();
Angle sdcSensorData::frontMinAngle = Angle(0);
double sdcSensorData::frontAngleResolution = 0;

bool sdcSensorData::backIsAllInfVar = true;
std::vector<double>* sdcSensorData::backAnglesNotAtInf = new std::vector<double>();
double sdcSensorData::backRayRange = std::numeric_limits<double>::infinity();

bool sdcSensorData::leftFrontIsAllInfVar = true;
std::vector<double>* sdcSensorData::leftFrontAnglesNotAtInf = new std::vector<double>();
double sdcSensorData::leftFrontRayRange = std::numeric_limits<double>::infinity();

bool sdcSensorData::rightFrontIsAllInfVar = true;
std::vector<double>* sdcSensorData::rightFrontAnglesNotAtInf = new std::vector<double>();
double sdcSensorData::rightFrontRayRange = std::numeric_limits<double>::infinity();

bool sdcSensorData::leftBackIsAllInfVar = true;
std::vector<double>* sdcSensorData::leftBackAnglesNotAtInf = new std::vector<double>();
double sdcSensorData::leftBackRayRange = std::numeric_limits<double>::infinity();

bool sdcSensorData::rightBackIsAllInfVar = true;
std::vector<double>* sdcSensorData::rightBackAnglesNotAtInf = new std::vector<double>();
double sdcSensorData::rightBackRayRange = std::numeric_limits<double>::infinity();

std::vector<double>* sdcSensorData::frontLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::backLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::leftTopLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::rightTopLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::forwardTopLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::backwardTopLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::leftFrontSideLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::leftBackSideLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::rightFrontSideLidarRays = new std::vector<double>();
std::vector<double>* sdcSensorData::rightBackSideLidarRays = new std::vector<double>();

bool sdcSensorData::stopSignInLeftCamera = false;
bool sdcSensorData::stopSignInRightCamera = false;

/*
 * Update front lidar rays
 */
void sdcSensorData::UpdateFrontLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    frontMinAngle = Angle(minAngle.Radian());
    frontAngleResolution = angleResolution;
    frontLidarRays = newRays;
    // unsigned int frontRayCount = frontLidarRays->size();
    //
    // frontRayRange = (*frontLidarRays)[320];
    //
    // frontIsAllInfVar = true;
    // frontAnglesNotAtInf->clear();
    // for (unsigned int i = 0; i < frontRayCount; ++i)
    // {
    //     if(!std::isinf((*frontLidarRays)[i])){
    //         frontIsAllInfVar = false;
    //         frontAnglesNotAtInf->push_back((minAngle+(*new math::Angle(i*angleResolution))).Radian());
    //     }
    // }
}

/*
 * Update back lidar rays
 */
 void sdcSensorData::UpdateBackLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){

     backLidarRays = newRays;

     unsigned int backRayCount = backLidarRays->size();

     backIsAllInfVar = true;
     backAnglesNotAtInf->clear();
     for (unsigned int i = 0; i < backRayCount; ++i)
     {
         if(!std::isinf((*backLidarRays)[i])){
             backIsAllInfVar = false;
             backAnglesNotAtInf->push_back((minAngle+(*new math::Angle(i*angleResolution))).Radian());
         }
     }
 }

/*
 * Update methods for top sensors
 */
void sdcSensorData::UpdateLeftLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    leftTopLidarRays = newRays;
}
void sdcSensorData::UpdateRightLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    rightTopLidarRays = newRays;
}
void sdcSensorData::UpdateForwardLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    forwardTopLidarRays = newRays;
}
void sdcSensorData::UpdateBackwardLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    backwardTopLidarRays = newRays;
}

/*
 * Update side lidars rays
 */
void sdcSensorData::UpdateSideLeftFrontLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    leftFrontSideLidarRays = newRays;

    unsigned int leftFrontSideRayCount = leftFrontSideLidarRays->size();

    leftFrontIsAllInfVar = true;
    leftFrontAnglesNotAtInf->clear();
    for (unsigned int i = 0; i < leftFrontSideRayCount; ++i)
    {
        if(!std::isinf((*leftFrontSideLidarRays)[i])){
            leftFrontIsAllInfVar = false;
            leftFrontAnglesNotAtInf->push_back((minAngle+(*new math::Angle(i*angleResolution))).Radian());
        }
    }
}
void sdcSensorData::UpdateSideLeftBackLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    leftBackSideLidarRays = newRays;

    unsigned int leftBackSideRayCount = leftBackSideLidarRays->size();

    leftBackIsAllInfVar = true;
    leftBackAnglesNotAtInf->clear();
    for (unsigned int i = 0; i < leftBackSideRayCount; ++i)
    {
        if(!std::isinf((*leftBackSideLidarRays)[i])){
            leftBackIsAllInfVar = false;
            leftBackAnglesNotAtInf->push_back((minAngle+(*new math::Angle(i*angleResolution))).Radian());
        }
    }
}
void sdcSensorData::UpdateSideRightFrontLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    rightFrontSideLidarRays = newRays;

    unsigned int rightFrontSideRayCount = rightFrontSideLidarRays->size();

    rightFrontIsAllInfVar = true;
    rightFrontAnglesNotAtInf->clear();
    for (unsigned int i = 0; i < rightFrontSideRayCount; ++i)
    {
        if(!std::isinf((*rightFrontSideLidarRays)[i])){
            rightFrontIsAllInfVar = false;
            rightFrontAnglesNotAtInf->push_back((minAngle+(*new math::Angle(i*angleResolution))).Radian());
        }
    }
}
void sdcSensorData::UpdateSideRightBackLidar(math::Angle minAngle, double angleResolution, std::vector<double>* newRays){
    rightBackSideLidarRays = newRays;

    unsigned int rightBackSideRayCount = rightBackSideLidarRays->size();

    rightBackIsAllInfVar = true;
    rightBackAnglesNotAtInf->clear();
    for (unsigned int i = 0; i < rightBackSideRayCount; ++i)
    {
        if(!std::isinf((*rightBackSideLidarRays)[i])){
            rightBackIsAllInfVar = false;
            rightBackAnglesNotAtInf->push_back((minAngle+(*new math::Angle(i*angleResolution))).Radian());
        }
    }
}

bool sdcSensorData::FrontIsAllInf(){
    return frontIsAllInfVar;
}

std::vector<double>* sdcSensorData::FrontGetNonInfAngles(){
    return frontAnglesNotAtInf;
}

/*
 * Returns a vector of the front lidar rays
 */
std::vector<double> sdcSensorData::GetFrontLidarRays(){
    std::vector<double> frontLidarRaysCopy = (*frontLidarRays);
    return frontLidarRaysCopy;
}

/*
 * Returns a vector of the back lidar rays
 */
std::vector<double> sdcSensorData::GetBackLidarRays(){
    std::vector<double> backLidarRaysCopy = (*backLidarRays);
    return backLidarRaysCopy;
}

/*
 * Returns a vector of the top lidar rays
 */
std::vector<double> sdcSensorData::GetTopLidarRays(){
    std::vector<double> topLidarRaysCopy;

    topLidarRaysCopy.insert(topLidarRaysCopy.end(),forwardTopLidarRays->begin(),forwardTopLidarRays->end());
    topLidarRaysCopy.insert(topLidarRaysCopy.end(),rightTopLidarRays->begin(),rightTopLidarRays->end());
    topLidarRaysCopy.insert(topLidarRaysCopy.end(),backwardTopLidarRays->begin(),backwardTopLidarRays->end());
    topLidarRaysCopy.insert(topLidarRaysCopy.end(),leftTopLidarRays->begin(),leftTopLidarRays->end());
    return topLidarRaysCopy;
}

/*
 * Returns the ray directly in front of the car; index=320
 */
double sdcSensorData::GetRangeInFront(){
    return frontRayRange;
}

/*
 * Returns a vector of the side lidar rays (currently incorrect)
 */
std::vector<double> sdcSensorData::GetSideLidarRays(){
    std::vector<double> sideLidarRaysCopy;

    sideLidarRaysCopy.insert(sideLidarRaysCopy.end(),rightFrontSideLidarRays->begin(),rightFrontSideLidarRays->end());
    sideLidarRaysCopy.insert(sideLidarRaysCopy.end(),rightBackSideLidarRays->begin(),rightBackSideLidarRays->end());
    sideLidarRaysCopy.insert(sideLidarRaysCopy.end(),leftBackSideLidarRays->begin(),leftBackSideLidarRays->end());
    sideLidarRaysCopy.insert(sideLidarRaysCopy.end(),leftFrontSideLidarRays->begin(),leftFrontSideLidarRays->end());
    return sideLidarRaysCopy;
}

/*
 * Returns a vector for the front right side lidar rays
 */
std::vector<double> sdcSensorData::GetRightFrontSideRays(){
    std::vector<double> rightFrontSideRaysCopy = (*rightFrontSideLidarRays);
    return rightFrontSideRaysCopy;
}

/*
 * Returns a vector for the front right side lidar rays
 */
std::vector<double> sdcSensorData::GetRightBackSideRays(){
    std::vector<double> rightBackSideRaysCopy = (*rightBackSideLidarRays);
    return rightBackSideRaysCopy;
}

/*
 * Returns a vector for the front right side lidar rays
 */
std::vector<double> sdcSensorData::GetLeftFrontSideRays(){
    std::vector<double> leftFrontSideRaysCopy = (*leftFrontSideLidarRays);
    return leftFrontSideRaysCopy;
}

/*
 * Returns a vector for the front right side lidar rays
 */
std::vector<double> sdcSensorData::GetLeftBackSideRays(){
    std::vector<double> leftBackSideRaysCopy = (*leftBackSideLidarRays);
    return leftBackSideRaysCopy;
}


/*
 * Return a vector of pairs (ray, ray length) which represents objects in view of front lidar
 */
std::vector<std::pair<Angle,double>> sdcSensorData::GetObjectsInFront(){
  std::vector<std::pair<Angle,double>> objectsInFront;
  for (int i = 0; i < frontLidarRays->size(); i++) {
    if (!std::isinf((*frontLidarRays)[i])) {
      Angle angle = Angle(i*frontAngleResolution+frontMinAngle);
      objectsInFront.push_back(std::make_pair(angle, (*frontLidarRays)[i]));
    }
  }
  return objectsInFront;
  // Print rays and ray lengths
  // std::cout << "Objects in front: " << std::endl;
  // for(int i = 0; i < objectsInFront.size(); i++){
  //     std::cout << "Pair: " << std::get<0>(objectsInFront[i]) << ", " << std::get<1>(objectsInFront[i]) << std::endl;
  // }
}

// New gps system using 2d vector
math::Vector2d sdcSensorData::coordinate;

void sdcSensorData::UpdateGPS(math::Vector2d newCoordinate){
    coordinate = newCoordinate;
}

math::Vector2d sdcSensorData::GetCurrentCoord(){
    return coordinate;
}

double sdcSensorData::GetLatitude(){
    return coordinate[1];
}

double sdcSensorData::GetLongitude(){
    return coordinate[0];
}

/* Old coordinate system
math::Angle* sdcSensorData::targetLon = new math::Angle(0);
math::Angle* sdcSensorData::lat = new math::Angle(0);
math::Angle* sdcSensorData::lon = new math::Angle(0);

 void sdcSensorData::UpdateGPS(math::Angle* newLat, math::Angle* newLon){
 lat = newLat;
 lon = newLon;
 }

double sdcSensorData::GetLongitude(){
    return lon->Degree();
}

double sdcSensorData::GetLatitude(){
    return lat->Degree();
}
*/
