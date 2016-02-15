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

const double DIRECTION_MARGIN_OF_ERROR = 0.00855;
const double STEERING_MARGIN_OF_ERROR = 0.05;
const int LIDAR_DETECTION_MARGIN_OF_ERROR = 2;

const double STEERING_ADJUSTMENT_RATE = 0.02;

const double PI = 3.14159265359;

// How much we can turn the "steering wheel"
const double STEERING_RANGE = 5 * PI;

const double CAR_WIDTH = 0.8;
const double CAR_LENGTH = 2.0;

const sdcAngle NORTH = sdcAngle(PI/2);
const sdcAngle SOUTH = sdcAngle(3*PI/2);
const sdcAngle WEST = sdcAngle(PI);
const sdcAngle EAST = sdcAngle(0);

// const math::Vector2d WAYPOINT_POS = {10,10};
const std::vector<math::Vector2d> WAYPOINT_POS_VEC = {{10,10},{-10,10}};
// const Waypoint WAYPOINT = Waypoint(1,WAYPOINT_POS);
// const std::vector<Waypoint> WAYPOINT_VECTOR = {WAYPOINT};


std::vector<double> turningVector;

enum WaypointType {
  // Waypoint to visit, lowest priority
  WaypointType_Target,
  // Waypoints created programmatically, should be given higher priority
  WaypointType_DriveStraight,
  WaypointType_TurnLeft,
  WaypointType_TurnRight,
  WaypointType_Stop
};

class Waypoint {
  int waypointType;
  math::Vector2d pos;

  Waypoint(int waypointType, math::Vector2d pos) {
    this->waypointType = waypointType;
    this->pos = pos;
  }
};

sdcCar::sdcCar()
{

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

void sdcCar::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->chassis = this->model->GetLink(_sdf->Get<std::string>("chassis"));

  this->joints[0] = this->model->GetJoint(_sdf->Get<std::string>("front_left"));
  this->joints[1] = this->model->GetJoint(_sdf->Get<std::string>("front_right"));
  this->joints[2] = this->model->GetJoint(_sdf->Get<std::string>("back_left"));
  this->joints[3] = this->model->GetJoint(_sdf->Get<std::string>("back_right"));

  this->maxSpeed = _sdf->Get<double>("max_speed");
  this->aeroLoad = _sdf->Get<double>("aero_load");
  this->tireAngleRange = _sdf->Get<double>("tire_angle_range");
  this->frontPower = _sdf->Get<double>("front_power");
  this->rearPower = _sdf->Get<double>("rear_power");
  this->wheelRadius = _sdf->Get<double>("wheel_radius");

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(boost::bind(&sdcCar::OnUpdate, this)));
}

void sdcCar::Init()
{
  // Compute the angle ratio between the steering wheel and the tires
  this->steeringRatio = STEERING_RANGE / this->tireAngleRange;
}

void sdcCar::OnUpdate()
{
    // Get the current velocity of the car
    this->velocity = this->chassis->GetWorldLinearVel();
    this->x = this->chassis->GetWorldPose().pos.x;
    this->y = this->chassis->GetWorldPose().pos.y;

    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = pose.rot.GetYaw();

    this->frontLidarUpdate();
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
 * Returns true if the current velocity angle matches the direction the car
 * is facing
 */
bool sdcCar::IsMovingForwards(){
    sdcAngle velAngle = GetDirection();
    sdcAngle carAngle = sdcAngle(this->yaw);
    return (carAngle - velAngle).isFrontFacing();
}

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
 * Handles turning based on the value of targetDirection. Calculates both which direction
 * to turn and by how much, as well as turning the actual wheel
 */
void sdcCar::Steer(){
    sdcAngle directionAngleChange = this->GetDirection() - this->targetDirection;
    // If the car needs to turn, set the target steering amount
    if (!directionAngleChange.withinMargin(DIRECTION_MARGIN_OF_ERROR)) {
        // Possible different approach to steering:
        // 1.67 is the distance between wheels in the sdf
        // double proposedSteeringAmount = asin(1.67/steeringRadius);

        double proposedSteeringAmount = fmax(fmin(-21*tan(directionAngleChange.angle/-2), 21), -21);
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

//Updates Front LIDAR data
void sdcCar::frontLidarUpdate(){
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

////////////////////////////////
////////////////////////////////
// BEGIN THE BRAIN OF THE CAR //
////////////////////////////////
////////////////////////////////

/*
 * Handles all logic for driving
 */
void sdcCar::Drive()
{
    // Do collision detection
    if (this->ObjectDirectlyAhead()){
        this->currentState = avoidance;
    }

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
        this->Accelerate();
        // this->WaypointDriving(WAYPOINT_POS_VEC);
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
        break;
    }

    // Sets state to default (waypoint)
    // this->currentState = waypoint;

    // Attempts to turn towards the target direction
    this->Steer();
    // Attempts to match the target speed
    this->MatchTargetSpeed();
    // this->SetTargetSpeed(4);
    // this->Accelerate();
    //if (sdcSensorData::stopSignInLeftCamera && sdcSensorData::stopSignInRightCamera) {
    //  this->SetTargetSpeed(0);
    //} else {
    //  this->SetTargetSpeed(4);
    //}

    //this->DriveStraightThenStop();
    //this->WalledDriving();
    //this->DetectIntersection();
}



/*
 * Drive from point to point in the given list
 */
void sdcCar::WaypointDriving(std::vector<math::Vector2d> waypoints) {
    int progress = this->waypointProgress;
    //std::vector<math::Vector2d> waypoints = waypoints;
    // std::cout << progress << " / " << waypoints.size() << " " << (progress < waypoints.size()) << std::endl;
    if(progress < waypoints.size()){
        math::Vector2d nextTarget = waypoints[progress];
        sdcAngle targetAngle = AngleToTarget(nextTarget);
        this->SetTargetDirection(targetAngle);

        // std::cout << targetAngle << std::endl;

        this->Accelerate();

        double distance = sqrt(pow(waypoints[progress][0] - this->x,2) + pow(waypoints[progress][1] - this->y,2));
        //sdcSensorData::GetCurrentCoord().Distance(nextTarget);
        // std::cout << distance << std::endl;
        if (distance < 1) {
            //std::cout << "#################################/nTarget achieved#################################/n";
            ++progress;
        }
    } else {
        this->Brake();
    }
    this->waypointProgress = progress;
}

/*
 * Returns the angle from the car's current position to a target position
 */
sdcAngle sdcCar::AngleToTarget(math::Vector2d target) {
    math::Vector2d position = sdcSensorData::GetCurrentCoord();
    math::Vector2d targetVector = math::Vector2d(target.x - this->x, target.y - this->y);
    return sdcAngle(atan2(targetVector.y, targetVector.x));
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
 * Uses the front LIDAR sensor to detect an intersection.
 * Based off of how many fields of view we have an what we can see we try to turn.
 * When we are almost at an intersection slow down.
 */
void sdcCar::DetectIntersection(){
  if(this->atIntersection == 0 && this->flViews.size() == 4 && this->flSideRight == 0 && this->flSideLeft == 0){
    this->SetTargetSpeed(2);
    this->atIntersection = 1;
  }else if (this->atIntersection == 1 && this->flViews.size() > 1 && this->flViews.size() < 4 && (this->flSideLeft != 0 && this->flSideRight != 0)){
      this->SetTargetSpeed(1);
      this->atIntersection = 2;
      this->GridTurning();
  } else if (this->atIntersection == 2 && (this->GetDirection() - this->targetDirection).withinMargin(PI/16)){
      this->SetTargetSpeed(5);
      this->atIntersection = 0;
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

bool sdcCar::ObjectDirectlyAhead() {
    if(this->flNumRays == 0) return false;
    std::vector<std::pair<sdcAngle,double>> blockedRays = sdcSensorData::GetBlockedFrontRays();
    for (int i = 0; i < blockedRays.size(); i++) {
        double distanceFromCenter = sin(blockedRays[i].first.angle) * blockedRays[i].second;
        if (fabs(distanceFromCenter) < CAR_WIDTH / 2. + 0.25){
            return true;
        }
    }
    return false;
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
 * Perpendicular parking algorithm
 */
void sdcCar::PerpendicularPark(){
    // Get rays that detect whether the front left and right bumpers of the car
    // will collide with other objects
    std::vector<double> backLidar = sdcSensorData::GetLidarRays(BACK);
    switch(this->currentParkingState)
    {
        case donePark:
        std::cout << "in done park" << std::endl;
        this->StopReverse();
        this->Stop();
        this->Brake();
        break;

        case frontPark:
        this->StopReverse();
        this->SetTargetSpeed(1);
        break;

        case straightPark:
        std::cout << "in straight park" << std::endl;
        this->SetTargetSpeed(2);
        if(backLidar[320] < 5){
            this->currentParkingState = donePark;
        }
        break;

        case stopPark:
        this->Stop();
        this->Brake();
        this->currentParkingState = frontPark;
        break;

        case backPark:
        // If the car is too close to anything on the sides, stop and fix it
        if(backLidar.size() != 0 && (backLidar[0] < 0.25 || backLidar[639] < 0.25)){
            std::cout << "Back Lidars: " << backLidar[0] << backLidar[639] << std::endl;
            this->currentParkingState = stopPark;
            break;
        }
        // Sets a target angle for the car for when it's done parking
        if(!parkingAngleSet){
            this->targetParkingAngle = (this->GetDirection() - PI/2);
            this->SetTargetDirection(this->targetParkingAngle);
            this->parkingAngleSet = true;
            break;
        } else {
            this->Reverse();
            this->SetTargetSpeed(2);
        }
        // Check to see if current direction is the same as targetParkingAngle
        sdcAngle margin = this->GetDirection() - this->targetParkingAngle;
        if(margin.withinMargin(0.005)){
            this->parkingAngleSet = false;
            this->currentParkingState = straightPark;
        }
        break;
    }
}
