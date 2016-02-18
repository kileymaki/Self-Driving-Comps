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

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "sdcCar.hh"
#include <vector>
#include <exception>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(sdcCar)

// SDC-defined constants
const double PI = 3.14159265359;

const double DIRECTION_MARGIN_OF_ERROR = 0.00855;
const double STEERING_MARGIN_OF_ERROR = 0.05;
const int LIDAR_DETECTION_MARGIN_OF_ERROR = 2;

const double STEERING_ADJUSTMENT_RATE = 0.02;

// How much we can turn the "steering wheel"
const double STEERING_RANGE = 5 * PI;

const double CAR_WIDTH = 0.8;
const double CAR_LENGTH = 2.0;

const sdcAngle NORTH = sdcAngle(PI/2);
const sdcAngle SOUTH = sdcAngle(3*PI/2);
const sdcAngle EAST = sdcAngle(0);
const sdcAngle WEST = sdcAngle(PI);
const std::vector<std::pair<double,double>> GRID_INTERSECTIONS = {{0,0},{0,50},{0,100},{0,150},{0,200},{0,250},{50,0},{50,50},{50,100},{50,150},{50,200},{50,250},{100,0},{100,50},{100,100},{100,150},{100,200},{100,250},{150,0},{150,50},{150,100},{150,150},{150,200},{150,250},{200,0},{200,50},{200,100},{200,150},{200,200},{200,250},{250,0},{250,50},{250,100},{250,150},{250,200},{250,250}};

// const math::Vector2d WAYPOINT_POS = {10,10};
// const std::vector<math::Vector2d> WAYPOINT_POS_VEC = {{150,100},{150,150}};
const sdcWaypoint WAYPOINT = sdcWaypoint(1,{150,100});
const std::vector<sdcWaypoint> WAYPOINT_VEC = {WAYPOINT};


std::vector<double> turningVector;


////////////////////////////////
////////////////////////////////
// BEGIN THE BRAIN OF THE CAR //
////////////////////////////////
////////////////////////////////

/*
 * Handles all logic for driving, is called every time the car receives an update
 * request from Gazebo
 */
void sdcCar::Drive()
{
    // Do collision detection
    if (this->ObjectDirectlyAhead()){
        // this->currentState = avoidance;
    }

    if(this->frontLidarLastUpdate != sdcSensorData::frontLidarLastUpdate){
        std::vector<sdcVisibleObject> v = sdcSensorData::GetObjectsInFront();
        this->UpdateFrontObjects(v);
        this->frontLidarLastUpdate = sdcSensorData::frontLidarLastUpdate;
    }

    // std::cout << this->currentState << std::endl;
    // Possible states: stop, waypoint, intersection, follow, avoidance
    switch(this->currentState)
    {
        // Final state, car is finished driving
        case stop:
        this->Stop();
        break;

        // Default state; drive straight to target location
        case waypoint:
        // Handle lane driving
        this->LanedDriving();
        this->Accelerate();
        // this->WaypointDriving(WAYPOINT_VEC);
        break;

        // At a stop sign, performing a turn
        case intersection:
        break;

        // Follows object that is going in same direction/towards same target
        case follow:
        this->Follow();
        // Handle lane driving
        break;

        // Smarter way to avoid objects; stopping, swerving, etc.
        case avoidance:
        // Cases: stop, swerve, go around
        this->Stop();
        break;

        // Parks the car
        case parking:
        this->PerpendicularPark();
        // this->ParallelPark();
        break;
    }

    // Attempts to turn towards the target direction
    this->MatchTargetDirection();
    // Attempts to match the target speed
    this->MatchTargetSpeed();

    //if (sdcSensorData::stopSignInLeftCamera && sdcSensorData::stopSignInRightCamera) {
    //  this->Stop();
    //} else {
    //  this->SetTargetSpeed(4);
    //}

    //this->WalledDriving();
    //this->DetectIntersection();
}

/*
 * Handles turning based on the value of targetDirection. Calculates both which direction
 * to turn and by how much, as well as turning the actual wheel
 */
void sdcCar::MatchTargetDirection(){
    sdcAngle directionAngleChange = this->GetDirection() - this->targetDirection;
    // If the car needs to turn, set the target steering amount
    if (!directionAngleChange.withinMargin(DIRECTION_MARGIN_OF_ERROR)) {
        // Possible different approach to steering:
        // 1.67 is the distance between wheels in the sdf
        // double proposedSteeringAmount = asin(1.67/steeringRadius);

        double proposedSteeringAmount = fmax(fmin(-30*tan(directionAngleChange.angle/-2), 30), -30);
        // When reversing, steering directions are inverted
        if(!this->reversing){
            this->SetTargetSteeringAmount(proposedSteeringAmount);
        }else{
            this->SetTargetSteeringAmount(-proposedSteeringAmount);
        }
    }

    // Check if the car needs to steer, and apply a small turn in the corresponding direction
    if (!(std::abs(this->targetSteeringAmount - this->steeringAmount) < STEERING_MARGIN_OF_ERROR)) {
        this->turning = true;
        if (this->steeringAmount < this->targetSteeringAmount) {
            this->steeringAmount = this->steeringAmount + STEERING_ADJUSTMENT_RATE;
        }else{
            this->steeringAmount = this->steeringAmount - STEERING_ADJUSTMENT_RATE;
        }
    } else {
        this->turning = false;
    }
}

/*
 * Attempts to match the current target speed
 */
void sdcCar::MatchTargetSpeed(){
    int dirConst = this->reversing ? -1 : 1;
    if(this->GetSpeed() < this->targetSpeed){
        this->gas = 1.0 * dirConst;
        this->brake = 0.0;
    } else if(this->GetSpeed() > this->targetSpeed){
        this->gas = 0.0;
        if(this->reversing != this->IsMovingForwards()){
            this->brake = -2.0 * dirConst;
        } else {
            this->brake = 0.0;
        }
    }
}

/*
 * Drive from point to point in the given list
 */
void sdcCar::WaypointDriving(std::vector<sdcWaypoint> waypoints) {
    int progress = this->waypointProgress;
    //std::vector<math::Vector2d> waypoints = waypoints;
    // std::cout << progress << " / " << waypoints.size() << " " << (progress < waypoints.size()) << std::endl;
    if(progress < waypoints.size()){
        math::Vector2d nextTarget = {waypoints[progress].pos.first,waypoints[progress].pos.second};
        sdcAngle targetAngle = AngleToTarget(nextTarget);
        this->SetTargetDirection(targetAngle);

        // std::cout << targetAngle << std::endl;

        this->Accelerate();

        double distance = sqrt(pow(waypoints[progress].pos.first - this->x,2) + pow(waypoints[progress].pos.second - this->y,2));
        //sdcSensorData::GetCurrentCoord().Distance(nextTarget);
        // std::cout << distance << std::endl;
        if (distance < 8) {
            ++progress;
        }
    } else {
        this->currentState = stop;
    }
    this->waypointProgress = progress;
}

/*
 * Uses camera data to detect lanes and sets targetDirection to stay as close
 * as possible to the midpoint.
 */
void sdcCar::LanedDriving() {
    int lanePos = sdcSensorData::LanePosition();
    if (!(lanePos > 320 || lanePos < -320)) {
        // It's beautiful don't question it
        sdcAngle laneWeight = sdcAngle(tan(lanePos/(PI*66.19))/10);
        this->SetTargetDirection(this->GetDirection() + laneWeight);
    }
}

/*
 * Drive with walled roads
 * LIDAR 0-319 is right, 320-619 is left.
 */
void sdcCar::WalledDriving(){
    //When driving down our current grid and the car stabilizes, centerRight and centerLeft are between 260-262 and drops down to 209 on intersections.
    if(this->atIntersection==0){
        this->SetTargetDirection(this->GetDirection() + sdcAngle(this->flWeight*PI/320));
    }
}

/*
 * Car follows an object directly in front of it and slows down to stop when it starts to get close
 */
void sdcCar::Follow() {
  if(this->flNumRays == 0) return;
  double distance = fl[320];
  if(std::isinf(distance)){
      lastPosition = 20.0;
      estimatedSpeed = fmin(6, estimatedSpeed + .01);
  } else {
      double deltaDistance = distance - lastPosition;
      lastPosition = distance;
      double estimatedSpeedData = deltaDistance * 1000 + this->GetSpeed();
      double alpha = fmax((distance * .005), (.1 - distance * -.005));
      estimatedSpeed = fmin(6, (alpha * estimatedSpeedData) + ((1 - alpha) * estimatedSpeed));
  }
  this->SetTargetSpeed(estimatedSpeed);
  this->SetTargetDirection(this->GetDirection() - sdcAngle(this->flWeight*PI/320));
}

void sdcCar::GridTurning(){
    std::string curDir;
    if((this->GetDirection() - WEST).withinMargin(PI/4)){
        curDir = "WEST";
    } else if((this->GetDirection() - SOUTH).withinMargin(PI/4)){
        curDir = "SOUTH";
    } else if((this->GetDirection() - EAST).withinMargin(PI/4)){
        curDir = "EAST";
    } else {
        curDir = "NORTH";
    }

    math::Vector2d waypoint = math::Vector2d(200,100);
    double x = this->x;
    double y = this->y;
    //The direction we need to head in to get to the destination
    std::string dirX;
    std::string dirY;
    if(waypoint[0] - 5 < x && waypoint[0] + 5 > x){
        dirX = "";
    } else if (waypoint[0] > x){
        dirX = "EAST";
    } else {
        dirX = "WEST";
    }
    if(waypoint[1] - 5 < y && waypoint[1] + 5 > y){
        dirY = "";
    } else if(waypoint[1] > y){
        dirY = "NORTH";
    } else {
        dirY = "SOUTH";
    }

    if(dirX == "" && dirY == ""){
        std::cout << "WE MADE IT" << std::endl;
        this->atIntersection = 3;
    } else {
        std::cout << "x " << dirX << std::endl;
        std::cout << "y " << dirY << std::endl;
        std::cout << "curDir " << curDir << std::endl;
        if(dirX == curDir || dirY == curDir){}
        else {
            this->TurnAround();
        }
    }

    // SetTargetDirection(this->GetDirection() + PI/2);
}

/*
 * Goes around the block and returns to the original point but now it is facing the other way
 */
void sdcCar::TurnAround(){
    if(turningVector.empty())
        turningVector = {PI/2,-PI/2,-PI/2,-PI/2};
    SetTargetDirection(this->GetDirection() + turningVector.back());
    turningVector.pop_back();
}

/*
 * Perpendicular back parking algorithm
 */
void sdcCar::PerpendicularPark(){
    // Get rays that detect whether the back left and right bumpers of the car
    // will collide with other objects
    std::vector<double> backLidar = sdcSensorData::GetLidarRays(BACK);
    std::vector<double> rightBackSideLidar = sdcSensorData::GetLidarRays(SIDE_RIGHT_BACK);
    std::vector<double> leftBackSideLidar = sdcSensorData::GetLidarRays(SIDE_LEFT_BACK);
    std::vector<double> rightFrontSideLidar = sdcSensorData::GetLidarRays(SIDE_RIGHT_FRONT);
    std::vector<double> leftFrontSideLidar = sdcSensorData::GetLidarRays(SIDE_LEFT_FRONT);
    switch(this->currentParkingState)
    {
        case donePark:
        // std::cout << "in done park" << std::endl;
        this->StopReverse();
        this->Stop();
        this->currentState = stop;
        break;

        case frontPark:
        // std::cout << "in front park" << std::endl;
        if(backLidar.size() != 0){
            for(int i = 0; i < backLidar.size(); i++){
                if(backLidar[i] < 2.0){
                    this->StopReverse();
                    this->SetTargetSpeed(0.5);
                    sdcAngle margin = this->GetOrientation() - this->targetParkingAngle;
                    if(rightFrontSideLidar.size() > 0 && leftFrontSideLidar.size() > 0 && rightBackSideLidar.size() && leftBackSideLidar.size()){
                            double rightSideMargins = std::abs(rightFrontSideLidar[80] - rightBackSideLidar[80]);
                            double leftSideMargins = std::abs(leftFrontSideLidar[80] - leftBackSideLidar[80]);
                        if((margin.withinMargin(0.005) && rightSideMargins < 0.05 && leftSideMargins < 0.05)){
                            this->parkingAngleSet = false;
                            this->currentParkingState = straightPark;
                        }
                    }
                    this->SetTargetDirection(targetParkingAngle);
                    break;
                } else {
                    this->currentParkingState = backPark;
                    break;
                }
            }
        }
        break;

        case straightPark:
        // std::cout << "in straight park" << std::endl;
        this->Reverse();
        this->SetTargetSpeed(0.5);
        if(backLidar[320] < 2){
            this->currentParkingState = donePark;
        }
        break;

        case stopPark:
        this->Stop();
        this->currentParkingState = frontPark;
        break;

        case backPark:
        // std::cout << "in back park" << std::endl;
        // If the car is too close to anything on the back or sides, stop and fix it
        if(backLidar.size() != 0){
            for(int i = 0; i < backLidar.size(); i++){
                if(backLidar[i] < 0.5){
                    this->currentParkingState = stopPark;
                }
            }
        }
        if(leftBackSideLidar.size() != 0){
            for(int i = 0; i < leftBackSideLidar.size(); i++){
                if(leftBackSideLidar[i] < 0.25){
                    this->currentParkingState = stopPark;
                }
            }
        }
        if(rightBackSideLidar.size() != 0){
            for(int i = 0; i < rightBackSideLidar.size(); i++){
                if(rightBackSideLidar[i] < 0.25){
                    this->currentParkingState = stopPark;
                }
            }
        }
        // Sets a target angle for the car for when it's done parking
        if(!parkingAngleSet){
            this->targetParkingAngle = (this->GetOrientation() - (3*PI)/2);
            this->SetTargetDirection(2*PI - this->targetParkingAngle);
            this->parkingAngleSet = true;
            break;
        } else {
            this->SetTargetDirection(2*PI - this->targetParkingAngle);
            this->Reverse();
            this->SetTargetSpeed(0.5);
        }
        // Check to see if current direction is the same as targetParkingAngle
        sdcAngle margin = this->GetOrientation() - this->targetParkingAngle;
        if(rightFrontSideLidar.size() > 0 && leftFrontSideLidar.size() > 0 && rightBackSideLidar.size() && leftBackSideLidar.size()){
            double rightSideMargins = std::abs(rightFrontSideLidar[80] - rightBackSideLidar[80]);
            double leftSideMargins = std::abs(leftFrontSideLidar[80] - leftBackSideLidar[80]);
            if((margin.withinMargin(0.005) && rightSideMargins < 0.05 && leftSideMargins < 0.05)){
                this->parkingAngleSet = false;
                this->currentParkingState = straightPark;
            }
        }
        break;
    }
}

/*
 * Parallel park algorithm
 */
void sdcCar::ParallelPark(){


    switch(this->currentParallelState)
    {
        case rightBack:
        {
            if(!parkingAngleSet){
                std::cout << "Current direction: " << this->GetOrientation() << std::endl;
                this->targetParkingAngle = this->GetOrientation() + PI/2;
                std::cout << "Current parking angle: " << this->targetParkingAngle << std::endl;
                this->SetTargetDirection(this->GetOrientation() - PI/2);
                std::cout << "Current target direction: " << this->targetDirection << std::endl;
                this->parkingAngleSet = true;
            } else {
                std::cout << "Current direction: " << this->GetOrientation() << std::endl;
                std::cout << "Current parking angle: " << this->targetParkingAngle << std::endl;
                std::cout << "Current target direction: " << this->targetDirection << std::endl;
                this->Reverse();
                this->SetTargetSpeed(0.5);
                break;
            }
        }

        case leftBack:
        break;

        case rightForward:
        break;

        case straightForward:
        break;

        case doneParallel:
        break;
    }
}

/*
 * Uses the front LIDAR sensor to detect an intersection.
 * Based off of how many fields of view we have an what we can see we try to turn.
 * When we are almost at an intersection slow down.
 */
void sdcCar::DetectIntersection(){
    /*if(this->atIntersection == 0 && this->flViews.size() == 4 && this->flSideRight == 0 && this->flSideLeft == 0){
        this->SetTargetSpeed(2);
        this->atIntersection = 1;
    }else*/ if (this->atIntersection == 0 /*&& this->flViews.size() > 1 && this->flViews.size() < 4 && (this->flSideLeft != 0 && this->flSideRight != 0)*/){
        this->SetTargetSpeed(1);
        this->atIntersection = 1;
        this->GridTurning();
    } else if (this->atIntersection == 1 && (this->GetDirection() - this->targetDirection).withinMargin(PI/16)){
        this->SetTargetSpeed(5);
        this->atIntersection = 0;
    }
}

//Generates a series of waypoints to get to the desired destination
std::vector<sdcWaypoint> sdcCar::GenerateWaypoints(sdcWaypoint dest){
    std::pair<double,double> firstIntr;
    std::vector<sdcWaypoint> waypoints;

    //Get the current direction
    if((this->yaw - WEST).withinMargin(PI/4)){
        this->currentDir = west;
    } else if((this->yaw - SOUTH).withinMargin(PI/4)){
        this->currentDir = south;
    } else if((this->yaw - EAST).withinMargin(PI/4)){
        this->currentDir = east;
    } else {
        this->currentDir = north;
    }
    std::cout << this->currentDir << std::endl;

    std::cout << "curPos: " << this->x << " " << this->y << std::endl;

    //Generates the coordinates for the first intersection the car will hit
    switch(this->currentDir){

        case west:
            firstIntr = {-1000,0};
            for(int i = 0; i < GRID_INTERSECTIONS.size();++i){
                if(this->y < GRID_INTERSECTIONS[i].second+5 && this->y > GRID_INTERSECTIONS[i].second-5 && GRID_INTERSECTIONS[i].first < this->x - 10 && GRID_INTERSECTIONS[i].first > firstIntr.first)
                    firstIntr = GRID_INTERSECTIONS[i];
            }
            break;

        case east:
            firstIntr = {1000,0};
            for(int i = 0; i < GRID_INTERSECTIONS.size();++i){
                if(this->y < GRID_INTERSECTIONS[i].second+5 && this->y > GRID_INTERSECTIONS[i].second-5 && GRID_INTERSECTIONS[i].first > this->x + 10 && GRID_INTERSECTIONS[i].first < firstIntr.first){
                    firstIntr = GRID_INTERSECTIONS[i];
                }
            }
            break;

        case north:
            firstIntr = {0,1000};
            for(int i = 0; i < GRID_INTERSECTIONS.size();++i){
                if(this->x < GRID_INTERSECTIONS[i].first+5 && this->x > GRID_INTERSECTIONS[i].first-5 && GRID_INTERSECTIONS[i].second > this->y + 10 && GRID_INTERSECTIONS[i].second < firstIntr.second)
                    firstIntr = GRID_INTERSECTIONS[i];
            }
            break;

        case south:
            firstIntr = {0,-1000};
            for(int i = 0; i < GRID_INTERSECTIONS.size();++i){
                if(this->x < GRID_INTERSECTIONS[i].first+5 && this->x > GRID_INTERSECTIONS[i].first-5 && GRID_INTERSECTIONS[i].second < this->y - 10 && GRID_INTERSECTIONS[i].second > firstIntr.second)
                    firstIntr = GRID_INTERSECTIONS[i];
            }
            break;
    }


    std::cout << "first interection: " << firstIntr.first << " " << firstIntr.second << std::endl;

    //Identifies what direction the destination is from the first intersection
    switch(this->currentDir){
        case west:
            if(dest.pos.first < firstIntr.first)
                destDir = forward;
            else if (dest.pos.first == firstIntr.first)
                destDir = aligned;
            else
                destDir = backward;
            if(dest.pos.second > firstIntr.second)
                destDirSide = right;
            else if (dest.pos.second == firstIntr.second)
                destDirSide = aligned;
            else
                destDirSide = left;
            break;

        case east:
            if(dest.pos.first > firstIntr.first)
                destDir = forward;
            else if (dest.pos.first == firstIntr.first)
                destDir = aligned;
            else
                destDir = backward;
            if(dest.pos.second < firstIntr.second)
                destDirSide = right;
            else if (dest.pos.second == firstIntr.second)
                destDirSide = aligned;
            else
                destDirSide = left;
            break;

        case north:
            if(dest.pos.second > firstIntr.second)
                destDir = forward;
            else if (dest.pos.second == firstIntr.second)
                destDir = aligned;
            else
                destDir = backward;
            if(dest.pos.first > firstIntr.first)
                destDirSide = right;
            else if (dest.pos.first == firstIntr.first)
                destDirSide = aligned;
            else
                destDirSide = left;
            break;

        case south:
            if(dest.pos.second < firstIntr.second)
                destDir = forward;
            else if (dest.pos.second == firstIntr.second)
                destDir = aligned;
            else
                destDir = backward;
            if(dest.pos.first < firstIntr.first)
                destDirSide = right;
            else if (dest.pos.first == firstIntr.first)
                destDirSide = aligned;
            else
                destDirSide = left;
            break;
    }

    //Generates the waypoint vector
    switch(destDir){
        case aligned:
            switch(destDirSide){
                case aligned:
                    waypoints.push_back(dest);
                    break;
                case right:
                    switch(this->currentDir){
                        case north:
                            break;
                        case south:
                            break;
                        case east:
                            break;
                        case west:
                            break;
                    }
                    break;
                case left:
                    switch(this->currentDir){
                        case north:
                            break;
                        case south:
                            break;
                        case east:
                            break;
                        case west:
                            break;
                    }
                    break;
            }
            break;

        case forward:
            switch(destDirSide){
                case aligned:
                    waypoints.push_back(dest);
                    break;
                case right:
                    int waypointType;
                    switch(this->currentDir){
                        case north:
                        case south:
                            waypoints.push_back(sdcWaypoint(2,std::pair<double,double>(firstIntr.first,dest.pos.second)));
                            waypoints.push_back(dest);
                            break;
                        case east:
                        case west:
                            waypoints.push_back(sdcWaypoint(2,std::pair<double,double>(dest.pos.first,firstIntr.second)));
                            waypoints.push_back(dest);
                            break;
                    }
                    break;
                case left:
                    switch(this->currentDir){
                        case north:
                        case south:
                            waypoints.push_back(sdcWaypoint(1,std::pair<double,double>(firstIntr.first,dest.pos.second)));
                            waypoints.push_back(dest);
                            break;
                        case east:
                        case west:
                            waypoints.push_back(sdcWaypoint(1,std::pair<double,double>(dest.pos.first,firstIntr.second)));
                            waypoints.push_back(dest);
                            break;
                    }
                    break;
            }
            break;

        case backward:
            switch(destDirSide){
                case aligned:
                    break;
                case right:
                    switch(this->currentDir){
                        case north:
                            break;
                        case south:
                            break;
                        case east:
                            break;
                        case west:
                            break;
                    }
                    break;
                case left:
                    switch(this->currentDir){
                        case north:
                            break;
                        case south:
                            break;
                        case east:
                            break;
                        case west:
                            break;
                    }
                    break;
            }
            break;
    }

    // sdcWaypoint retW = {1,firstIntr};
    // std::vector<sdcWaypoint> ret = {retW};
    for(int i =0; i < waypoints.size();++i){
        std::cout << waypoints[i].waypointType << " " << waypoints[i].pos.first << " " << waypoints[i].pos.second << std::endl;
    }
    return waypoints;
}

////////////////////
// HELPER METHODS //
////////////////////

//Updates Front LIDAR data
void sdcCar::FrontLidarUpdate(){
  this->flViews.clear();
  this->fl = sdcSensorData::GetLidarRays(FRONT);
  this->flRayLengths = 0;
  this->flSideRight = 404;
  this->flCenterRight = 404;
  this->flCenterLeft = -404;
  this->flSideLeft = -404;
  this->flNumRays = this->fl.size();
  this->flWeight = 0;
  std::vector<int> leftView;
  std::vector<int> rightView;

  //With this loop we track the areas of view the car has.
  for (int i = 0; i < this->flNumRays; ++i) {
      if(!std::isinf(this->fl[i])){
          if(i>319){
              --this->flWeight;
              this->flRayLengths += this->fl[i];
              leftView.push_back(i);
              if(this->flCenterLeft <  std::abs(i-639))
                  this->flCenterLeft = std::abs(i-639);
              this->flSideLeft = std::abs(i-639);

          } else {
              ++this->flWeight;
              this->flRayLengths += this->fl[i];
              rightView.push_back(i);
              this->flCenterRight = i;
              if(this->flSideRight > i)
                  this->flSideRight = i;
          }
      } else {
        if (leftView.size()!=0){
          this->flViews.push_back(leftView);
          leftView.clear();
        }
        if(rightView.size()!=0){
          this->flViews.push_back(rightView);
          rightView.clear();
        }
      }
  }
  if (leftView.size()!=0){
    this->flViews.push_back(leftView);
    leftView.clear();
  }
  if(rightView.size()!=0){
    this->flViews.push_back(rightView);
    rightView.clear();
  }
}

void sdcCar::UpdateFrontObjects(std::vector<sdcVisibleObject> newObjects){
    if(this->frontObjects.size() == 0){
        // if(newObjects.size() > 0) std::cout << "Added " << newObjects.size() << " new object(s)" << std::endl;
        this->frontObjects = newObjects;
        return;
    }

    std::vector<bool> isOldObjectMissing;
    std::vector<bool> isBrandNewObject;
    for(int i = 0; i < newObjects.size(); i++){
        isBrandNewObject.push_back(true);
    }

    for (int i = 0; i < this->frontObjects.size(); i++) {
        sdcVisibleObject oldObj = this->frontObjects[i];
        isOldObjectMissing.push_back(true);

        for (int j = 0; j < newObjects.size(); j++) {
            if(!isBrandNewObject[j]) continue;
            sdcVisibleObject newObj = newObjects[j];

            if(oldObj.IsSameObject(newObj)){
                oldObj.Update(newObj);
                this->frontObjects[i] = oldObj;
                isOldObjectMissing[i] = false;
                isBrandNewObject[j] = false;
                break;
            }
        }
    }

    for(int i = isOldObjectMissing.size() - 1; i >= 0; i--){
        if(isOldObjectMissing[i]){
            // std::cout << "Erased missing object" << std::endl;
            this->frontObjects.erase(this->frontObjects.begin() + i);
        }
    }

    for(int i = 0; i < newObjects.size(); i++){
        if(isBrandNewObject[i]){
            // std::cout << "Added new object" << std::endl;
            this->frontObjects.push_back(newObjects[i]);
        }
    }
}

/*
 * Returns true if the current velocity angle matches the direction the car
 * is facing
 */
bool sdcCar::IsMovingForwards(){
    sdcAngle velAngle = GetDirection();
    sdcAngle carAngle = this->yaw;
    return (carAngle - velAngle).isFrontFacing();
}

/*
 * Gets the speed of the car
 */
double sdcCar::GetSpeed(){
    return sqrt(pow(this->velocity.x,2) + pow(this->velocity.y,2));
}

/*
 * Gets the current direction the car is travelling
 */
sdcAngle sdcCar::GetDirection(){
    math::Vector3 velocity = this->velocity;
    return sdcAngle(atan2(velocity.y, velocity.x));
}

/*
 * Gets the direction the car is facing
 */
sdcAngle sdcCar::GetOrientation(){
    return this->yaw;
}

/*
 * Returns the angle from the car's current position to a target position
 */
sdcAngle sdcCar::AngleToTarget(math::Vector2d target) {
    math::Vector2d position = sdcSensorData::GetPosition();
    math::Vector2d targetVector = math::Vector2d(target.x - position.x, target.y - position.y);
    return sdcAngle(atan2(targetVector.y, targetVector.x));
}

/*
 * Returns true if there is an object ahead of the car that might collide with us if we
 * continue driving straight ahead
 */
bool sdcCar::ObjectDirectlyAhead() {
    std::vector<sdcLidarRay> blockedRays = sdcSensorData::GetBlockedFrontRays();
    for (int i = 0; i < blockedRays.size(); i++) {
        if (fabs(blockedRays[i].GetLateralDist()) < CAR_WIDTH / 2. + 0.25){
            return true;
        }
    }
    return false;
}


///////////////////////////
// BEGIN CONTROL METHODS //
///////////////////////////

/*
 * Speeds up the car by the given amount (in m/s) at the given rate
 *
 * Default amt: 1.0
 * Default rate: 1.0
 */
void sdcCar::Accelerate(double amt, double rate){
    this->SetTargetSpeed(this->GetSpeed() + amt);
    this->SetAccelRate(rate);
}

/*
 * Slows down the car by the given amount (in m/s) at the given rate
 *
 * Default amt: 1.0
 * Default rate: 1.0
 */
void sdcCar::Brake(double amt, double rate){
    this->SetTargetSpeed(this->GetSpeed() - amt);
    this->SetBrakeRate(rate);
}

/*
 * Sets the target speed to 0 m/s
 */
void sdcCar::Stop(){
    this->SetTargetSpeed(0);
}

/*
 * Move the car in reverse. Target speed will now be matched with the car going
 * backwards and target direction should be the direction of velocity desired, NOT
 * the direction the front of the car is facing
 */
void sdcCar::Reverse(){
    this->reversing = true;
}

/*
 * Stop reversing the car.
 */
void sdcCar::StopReverse(){
    this->reversing = false;
}

/*
 * Sets the rate of acceleration for the car. The rate is a scalar for the
 * force applies to accelerate the car
 *
 * Default rate: 1.0, can't be negative
 */
void sdcCar::SetAccelRate(double rate){
    this->accelRate = fmax(rate, 0.0);
}

/*
 * Sets the rate of braking for the car. The rate is a scalar for the
 * force applied to brake the car
 *
 * Default rate: 1.0, can't be negative
 */
void sdcCar::SetBrakeRate(double rate){
    this->brakeRate = fmax(rate, 0.0);
}

/*
 * Sets a target direction for the car
 */
void sdcCar::SetTargetDirection(sdcAngle direction){
    this->targetDirection = direction;
}

/*
 * Sets a target steering amount for the steering wheel
 */
void sdcCar::SetTargetSteeringAmount(double a){
    this->targetSteeringAmount = a;
}

/*
 * Sets the target speed for the car, as well as resetting the brake
 * and accel rates to default. Methods wishing to change those parameters
 * should make sure to do so AFTER a call to this method
 */
void sdcCar::SetTargetSpeed(double s){
  this->targetSpeed = fmax(fmin(s, this->maxCarSpeed), 0);
  this->SetAccelRate();
  this->SetBrakeRate();
}

//////////////////////////////////////////////////////////////
// GAZEBO METHODS - GAZEBO CALLS THESE AT APPROPRIATE TIMES //
//////////////////////////////////////////////////////////////

/*
 * Called when initially loading the car model from the sdf. Links the car
 * to the OnUpdate methods so we can receive updates
 */
void sdcCar::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store the model and chassis of the car for later access
    this->model = _model;
    this->chassis = this->model->GetLink(_sdf->Get<std::string>("chassis"));

    // Get all the wheel joints
    this->joints[0] = this->model->GetJoint(_sdf->Get<std::string>("front_left"));
    this->joints[1] = this->model->GetJoint(_sdf->Get<std::string>("front_right"));
    this->joints[2] = this->model->GetJoint(_sdf->Get<std::string>("back_left"));
    this->joints[3] = this->model->GetJoint(_sdf->Get<std::string>("back_right"));

    // Pull some parameters that are defined in the sdf
    this->maxSpeed = _sdf->Get<double>("max_speed");
    this->aeroLoad = _sdf->Get<double>("aero_load");
    this->tireAngleRange = _sdf->Get<double>("tire_angle_range");
    this->frontPower = _sdf->Get<double>("front_power");
    this->rearPower = _sdf->Get<double>("rear_power");
    this->wheelRadius = _sdf->Get<double>("wheel_radius");

    // Tell Gazebo to call OnUpdate whenever the car needs an update
    this->connections.push_back(event::Events::ConnectWorldUpdateBegin(boost::bind(&sdcCar::OnUpdate, this)));
}

/*
 * Called when the car and world are being (re)initialized.
 */
void sdcCar::Init()
{
    // Compute the angle ratio between the steering wheel and the tires
    this->steeringRatio = STEERING_RANGE / this->tireAngleRange;

    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = sdcAngle(pose.rot.GetYaw());
    this->x = pose.pos.x;
    this->y = pose.pos.y;
    GenerateWaypoints(sdcWaypoint(3,{150,150}));
}

/*
 * Called whenever Gazebo needs an update for this model
 */
void sdcCar::OnUpdate()
{
    // Get the current velocity of the car
    this->velocity = this->chassis->GetWorldLinearVel();
    // Get the cars current position
    math::Vector2d pose = sdcSensorData::GetPosition();
    this->x = pose.x;
    this->y = pose.y;

    // Get the cars current rotation
    this->yaw = sdcSensorData::GetYaw();

    // Call our Drive function, which is the brain for the car
    this->FrontLidarUpdate();
    this->Drive();




    // Compute the angle of the front wheels.
    double wheelAngle = this->steeringAmount / this->steeringRatio;

    // Compute the rotational velocity of the wheels
    double jointVel = (this->gas-this->brake * this->maxSpeed) /
                    this->wheelRadius;

    // Set velocity and max force for each wheel
    this->joints[0]->SetVelocityLimit(1, -jointVel);
    this->joints[0]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->frontPower);

    this->joints[1]->SetVelocityLimit(1, -jointVel);
    this->joints[1]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->frontPower);

    this->joints[2]->SetVelocityLimit(1, -jointVel);
    this->joints[2]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->rearPower);

    this->joints[3]->SetVelocityLimit(1, -jointVel);
    this->joints[3]->SetForce(1, -(this->gas * this->accelRate + this->brake * this->brakeRate) * this->rearPower);

    // Set the front-left wheel angle
    this->joints[0]->SetLowStop(0, wheelAngle);
    this->joints[0]->SetHighStop(0, wheelAngle);
    this->joints[0]->SetLowStop(0, wheelAngle);
    this->joints[0]->SetHighStop(0, wheelAngle);

    // Set the front-right wheel angle
    this->joints[1]->SetHighStop(0, wheelAngle);
    this->joints[1]->SetLowStop(0, wheelAngle);
    this->joints[1]->SetHighStop(0, wheelAngle);
    this->joints[1]->SetLowStop(0, wheelAngle);

    //  aerodynamics
    this->chassis->AddForce(math::Vector3(0, 0, this->aeroLoad * this->velocity.GetSquaredLength()));

    // Sway bars
    math::Vector3 bodyPoint;
    math::Vector3 hingePoint;
    math::Vector3 axis;

    // Physics calculations
    for (int ix = 0; ix < 4; ++ix)
    {
        hingePoint = this->joints[ix]->GetAnchor(0);
        bodyPoint = this->joints[ix]->GetAnchor(1);

        axis = this->joints[ix]->GetGlobalAxis(0).Round();
        double displacement = (bodyPoint - hingePoint).Dot(axis);

        float amt = displacement * this->swayForce;
        if (displacement > 0)
        {
            if (amt > 15)
                amt = 15;

            math::Pose p = this->joints[ix]->GetChild()->GetWorldPose();
            this->joints[ix]->GetChild()->AddForce(axis * -amt);
            this->chassis->AddForceAtWorldPosition(axis * amt, p.pos);

            p = this->joints[ix^1]->GetChild()->GetWorldPose();
            this->joints[ix^1]->GetChild()->AddForce(axis * amt);
            this->chassis->AddForceAtWorldPosition(axis * -amt, p.pos);
        }
    }
}

/*
 * Constructor for the car. Sets several parameters to default values, some of
 * which will get overwritten in Load or Init and others that will be updated
 * when the car is updating
 */
sdcCar::sdcCar(){
    this->joints.resize(4);

    this->aeroLoad = 0.1;
    this->swayForce = 10;

    this->maxSpeed = 10;
    this->frontPower = 50;
    this->rearPower = 50;
    this->wheelRadius = 0.3;
    this->steeringRatio = 1.0;
    this->tireAngleRange = 1.0;

    this->gas = 0.0;
    this->brake = 0.0;
    this->accelRate = 1.0;
    this->brakeRate = 1.0;

    this->maxCarSpeed = 6;
    this->maxCarReverseSpeed = -10;

    this->currentState = waypoint;

    this->currentParkingState = backPark;
    this->currentParallelState = rightBack;

    this->steeringAmount = 0.0;
    this->targetSteeringAmount = 0.0;
    this->targetDirection = sdcAngle(0.0);
    this->turning = false;
    this->reversing = false;
    this->targetParkingAngle = sdcAngle(0.0);
    this->parkingAngleSet = false;

    this->targetSpeed = 5;

    // Used to track waypoint driving
    this->waypointProgress = 0;

    this->atIntersection = 0;

    // Used to estimate speed of followed object
    this->estimatedSpeed = 0.0;
    this->lastPosition = 0.0;
    this->currentPosition = 0.0;
}
