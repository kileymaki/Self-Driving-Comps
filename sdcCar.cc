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

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(sdcCar)

const double DIRECTION_MARGIN_OF_ERROR = 0.00855;
const double STEERING_MARGIN_OF_ERROR = 0.05;
const int LIDAR_DETECTION_MARGIN_OF_ERROR = 2;

const double STEERING_ADJUSTMENT_RATE = 0.02;

const double PI = 3.14159265359;

// int GRIDX = 100;
Angle BAD_DIRECTION = PI/2;


/////////////////////////////////////////////////
sdcCar::sdcCar()
{

    this->joints.resize(4);

    this->aeroLoad = 0.1;
    this->swayForce = 10;

    this->maxSpeed = 10;
    this->frontPower = 50;
    this->rearPower = 50;
    this->wheelRadius = 0.3;
    this->maxBrake = 0.0;
    this->maxGas = 0.0;
    this->steeringRatio = 1.0;
    this->tireAngleRange = 1.0;

    this->gas = 0.0;
    this->brake = 0.0;

    this->maxCarSpeed = 6;
    this->maxCarReverseSpeed = -10;

    this->steeringAmount = 0.0;
    this->targetSteeringAmount = 0.0;
    this->targetDirection = Angle(0.0);
    this->turning = false;

    this->targetSpeed = 6;

    // Used to track waypoint driving
    this->waypointProgress = 0;

    this->atIntersection = 0;

    // Used to estimate speed of followed object
    this->estimatedSpeed = 0.0;
    this->lastDistance = 0.0;
    this->speedCounter = 0;
}


/////////////////////////////////////////////////
void sdcCar::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  // this->physics = this->model->GetWorld()->GetPhysicsEngine();

  this->joints[0] = this->model->GetJoint(_sdf->Get<std::string>("front_left"));
  if (!this->joints[0])
  {
    gzerr << "Unable to find joint: front_left\n";
    return;
  }

  this->joints[1] = this->model->GetJoint(
      _sdf->Get<std::string>("front_right"));

  if (!this->joints[1])
  {
    gzerr << "Unable to find joint: front_right\n";
    return;
  }

  this->joints[2] = this->model->GetJoint(_sdf->Get<std::string>("back_left"));
  if (!this->joints[2])
  {
    gzerr << "Unable to find joint: back_left\n";
    return;
  }


  this->joints[3] = this->model->GetJoint(_sdf->Get<std::string>("back_right"));
  if (!this->joints[3])
  {
    gzerr << "Unable to find joint: back_right\n";
    return;
  }

  this->joints[0]->SetParam("suspension_erp", 0, 0.15);
  this->joints[0]->SetParam("suspension_cfm", 0, 0.01);

  this->joints[1]->SetParam("suspension_erp", 0, 0.15);
  this->joints[1]->SetParam("suspension_cfm", 0, 0.01);

  this->joints[2]->SetParam("suspension_erp", 0, 0.15);
  this->joints[2]->SetParam("suspension_cfm", 0, 0.01);

  this->joints[3]->SetParam("suspension_erp", 0, 0.15);
  this->joints[3]->SetParam("suspension_cfm", 0, 0.01);

  this->gasJoint = this->model->GetJoint(_sdf->Get<std::string>("gas"));
  this->brakeJoint = this->model->GetJoint(_sdf->Get<std::string>("brake"));
  this->steeringJoint = this->model->GetJoint(
      _sdf->Get<std::string>("steering"));

  if (!this->gasJoint)
  {
    gzerr << "Unable to find gas joint["
          << _sdf->Get<std::string>("gas") << "]\n";
    return;
  }

  if (!this->steeringJoint)
  {
    gzerr << "Unable to find steering joint["
          << _sdf->Get<std::string>("steering") << "]\n";
    return;
  }

  if (!this->joints[0])
  {
    gzerr << "Unable to find front_left joint["
          << _sdf->GetElement("front_left") << "]\n";
    return;
  }

  if (!this->joints[1])
  {
    gzerr << "Unable to find front_right joint["
          << _sdf->GetElement("front_right") << "]\n";
    return;
  }

  if (!this->joints[2])
  {
    gzerr << "Unable to find back_left joint["
          << _sdf->GetElement("back_left") << "]\n";
    return;
  }

  if (!this->joints[3])
  {
    gzerr << "Unable to find back_right joint["
          << _sdf->GetElement("back_right") << "]\n";
    return;
  }

  this->maxSpeed = _sdf->Get<double>("max_speed");
  this->aeroLoad = _sdf->Get<double>("aero_load");
  this->tireAngleRange = _sdf->Get<double>("tire_angle_range");
  this->frontPower = _sdf->Get<double>("front_power");
  this->rearPower = _sdf->Get<double>("rear_power");

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&sdcCar::OnUpdate, this)));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->velSub = this->node->Subscribe(std::string("~/") +
      this->model->GetName() + "/vel_cmd", &sdcCar::OnVelMsg, this);
}

/////////////////////////////////////////////////
void sdcCar::Init()
{
  this->chassis = this->joints[0]->GetParent();

  // This assumes that the largest dimension of the wheel is the diameter
  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->joints[0]->GetChild());
  math::Box bb = parent->GetBoundingBox();
  this->wheelRadius = bb.GetSize().GetMax() * 0.5;

  // The total range the steering wheel can rotate
  double steeringRange = this->steeringJoint->GetHighStop(0).Radian() -
                         this->steeringJoint->GetLowStop(0).Radian();

  // Compute the angle ratio between the steering wheel and the tires
  this->steeringRatio = steeringRange / this->tireAngleRange;

  // Maximum gas is the upper limit of the gas joint
  this->maxGas = this->gasJoint->GetHighStop(0).Radian();

  // Maximum brake is the upper limit of the gas joint
  this->maxBrake = this->gasJoint->GetHighStop(0).Radian();

  printf("SteeringRation[%f] MaxGas[%f]\n", this->steeringRatio, this->maxGas);
}

/////////////////////////////////////////////////
void sdcCar::OnUpdate()
{
  // Get the normalized gas and brake amount
//  double gas = this->gasJoint->GetAngle(0).Radian() / this->maxGas;
//  double brake = this->brakeJoint->GetAngle(0).Radian() / this->maxBrake;

  // A little force to push back on the pedals
//  this->gasJoint->SetForce(0, -0.1);
    //  this->brakeJoint->SetForce(0, -0.1);

    // Get the steering angle
    //  double steeringAngle = this->steeringJoint->GetAngle(0).Radian();

    // Get the current velocity of the car
    this->velocity = this->chassis->GetWorldLinearVel();

    //std::cout << this->GetSpeed() << "\n";

    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = pose.rot.GetYaw();




    this->Drive();





  // Compute the angle of the front wheels.
  double wheelAngle = this->steeringAmount / this->steeringRatio;

  // double idleSpeed = 0.5;

  // Compute the rotational velocity of the wheels
  double jointVel = (std::max(0.0, this->gas-this->brake) * this->maxSpeed) /
                    this->wheelRadius;

  // Set velocity and max force for each wheel
  this->joints[0]->SetVelocityLimit(1, -jointVel);
  this->joints[0]->SetForce(1, -(this->gas + this->brake) * this->frontPower);

  this->joints[1]->SetVelocityLimit(1, -jointVel);
  this->joints[1]->SetForce(1, -(this->gas + this->brake) * this->frontPower);

  this->joints[2]->SetVelocityLimit(1, -jointVel);
  this->joints[2]->SetForce(1, -(this->gas + this->brake) * this->rearPower);

  this->joints[3]->SetVelocityLimit(1, -jointVel);
  this->joints[3]->SetForce(1, -(this->gas + this->brake) * this->rearPower);

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
  this->chassis->AddForce(
      math::Vector3(0, 0, this->aeroLoad * this->velocity.GetSquaredLength()));

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

/////////////////////////////////////////////////
void sdcCar::OnVelMsg(ConstPosePtr &/*_msg*/)
{}

/*
 * Both Accel and Brake call ApplyMovementForce
 * Caps max velocity and accelerates the vehicle.
 */
void sdcCar::ApplyMovementForce(double amt){
    if(amt > 0){
        if(this->GetSpeed() > this->maxCarSpeed){
            amt = 0;
        }
        this->gas = std::min(5.0, amt);
        this->brake = 0.0;
    } else {
        this->gas = 0.0;
        if(this->IsMovingForwards()){
            this->brake = std::max(this->maxCarReverseSpeed, amt);
        } else {
            this->brake = 0.0;
        }
    }
}

bool sdcCar::IsMovingForwards(){
    Angle velAngle = GetDirection();
    Angle carAngle = Angle(this->yaw);
    return (carAngle - velAngle).isFrontFacing();
}

/*
 * Default: 0.5
 */
void sdcCar::Accel(double amt){
//    amt = 3;
    this->ApplyMovementForce(amt);
}

/*
 * Default: 1.0
 */
void sdcCar::Brake(double amt){
//    amt = 7;
    this->ApplyMovementForce(-amt);
}

void sdcCar::MatchTargetSpeed(){
  if(this->GetSpeed() < this->targetSpeed){
    this->gas = 1.0;
    this->brake = 0.0;
  }else if(this->GetSpeed() > this->targetSpeed){
    this->gas = 0.0;
    if(this->IsMovingForwards()){
        this->brake = -2.0;
    } else {
        this->brake = 0.0;
    }
  }
}

/*
 * Handles turning based on the value of targetDirection. Calculates both which direction
 * to turn and by how much, as well as turning the actual wheel
 */
void sdcCar::Steer(){
    Angle directionAngleChange = this->GetDirection() - this->targetDirection;
    // If the car needs to turn, set the target steering amount
    if (!directionAngleChange.withinMargin(DIRECTION_MARGIN_OF_ERROR)) {
        // Angle proposedSteeringAngle = Angle(7*pow(sin(std::abs(directionAngleChange.angle)/2)-1,3)+7);
        double proposedSteeringAmount = fmax(fmin(-7*tan(directionAngleChange.angle/-2), 7), -7)*2;
        this->SetTargetSteeringAmount(proposedSteeringAmount);
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

// PRELIMINARY THOUGHTS ON A SMARTER TURNING ALGORITHM

// void sdcCar::SteerToPosition(double steeringRadius, Angle targetDirection){
//     Angle directionAngleChange = this->GetDirection() - this->targetDirection;
//     if(!directionAngleChange.withinMargin(DIRECTION_MARGIN_OF_ERROR)){
//       // 1.67 is the distance between wheels in the sdf
//       double proposedSteeringAmount = asin(1.67/steeringRadius);
//       this->SetTargetSteeringAmount(proposedSteeringAmount);
//     }
//
//     // Check if the car needs to steer, and apply a small turn in the corresponding direction
//     if (!(std::abs(this->targetSteeringAmount - this->steeringAmount) < STEERING_MARGIN_OF_ERROR)) {
//         this->turning = true;
//         if (this->steeringAmount < this->targetSteeringAmount) {
//             this->steeringAmount = this->steeringAmount + STEERING_ADJUSTMENT_RATE;
//         }else{
//             this->steeringAmount = this->steeringAmount - STEERING_ADJUSTMENT_RATE;
//         }
//     } else {
//         this->turning = false;
//     }
// }

/*
 * Sets a target direction for the car
 */
void sdcCar::SetTargetDirection(Angle direction){
    this->targetDirection = direction;
}

/*
 * Sets a target steering amount for the steering wheel
 */
void sdcCar::SetTargetSteeringAmount(double a){
    this->targetSteeringAmount = a;
}

void sdcCar::SetTargetSpeed(double s){
  // Caps the target speed, currently can't reverse
  this->targetSpeed = fmax(fmin(s, this->maxCarSpeed), 0);
}

/*
 * Gets the spped of the car
 */
double sdcCar::GetSpeed(){
    return sqrt(pow(this->velocity.x,2) + pow(this->velocity.y,2));
}

/*
 * Gets the current direction of the car
 */
Angle sdcCar::GetDirection(){
    math::Vector3 velocity = this->velocity;
    return Angle(atan2(velocity.y, velocity.x));
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
//    this->TurnRightIfObjectAhead();
//    this->DriveStraightThenStop();
//    this->DriveToCoordinates(0.0005, 0.0005);

    // Combines WalledDriving with WaypointDriving;
    // if (!(sdcSensorData::IsAllInf())) {
    //     this->WalledDriving();
    // } else {
    //     std::vector<math::Vector2d> waypoints = {math::Vector2d(0.0005,0.000), math::Vector2d(-0.0005,0.000), math::Vector2d(-0.001,-0.001)};
    //     this->WaypointDriving(waypoints);
    // }


    // this->DriveStraightThenStop();
    //this->WalledDriving();
    //this->DetectIntersection();

     this->Follow();
    // Handles all turning
    this->Steer();
    this->MatchTargetSpeed();

    //if(!std::isinf(sdcSensorData::GetCurrentCoord().x)) {
    //  double temp = (this->avg*this->count)/(this->count+1);
    //  double data = this->chassis->GetWorldPose().pos.x / sdcSensorData::GetCurrentCoord().x;
    //  this->avg = temp + data/(this->count+1);
    //  this->count += 1;
    // std::cout << this->chassis->GetWorldPose().pos.x << std::endl;
    //}
}

/*
 * Drive from point to point in the given list
 */
void sdcCar::WaypointDriving(std::vector<math::Vector2d> waypoints){
    int progress = this->waypointProgress;
    //std::vector<math::Vector2d> waypoints = waypoints;
    //std::cout << progress << " / " << waypoints.size() << " " << (progress < waypoints.size()) << std::endl;
    if(progress < waypoints.size()){
        math::Vector2d nextTarget = waypoints[progress];
        Angle targetAngle = AngleToTarget(nextTarget);
        this->SetTargetDirection(targetAngle);

        //std::cout << targetAngle << std::endl;

        this->Accel();

        double distance = sdcSensorData::GetCurrentCoord().Distance(nextTarget);
        //std::cout << distance << std::endl;
        if (distance < 0.0001) {
            //std::cout << "#################################/nTarget achieved#################################/n";
            ++progress;
        }
    } else {
        this->Brake();
    }
    this->waypointProgress = progress;
}

// Returns the angle from the car's current position to a target position
Angle sdcCar::AngleToTarget(math::Vector2d target) {
    math::Vector2d position = sdcSensorData::GetCurrentCoord();
    math::Vector2d targetVector = math::Vector2d(target.x - position.x, target.y - position.y);
    return Angle(atan2(targetVector.y, targetVector.x));
}

// Drive with walled roads
//LIDAR 0-319 is right, 320-619 is left.
void sdcCar::WalledDriving(){
    std::vector<double> lidar = sdcSensorData::GetLidarRays();

    int rayLengths = 0;
    int weight = 0;
    int centerRight = 404;
    int sideRight = 404;
    int centerLeft = -404;
    int sideLeft = -404;
    int halfnumrays = lidar.size()/2;
    for (int i = 0; i < halfnumrays; ++i) {
        if(!std::isinf(lidar[i])){
            rayLengths += lidar[i];
            ++weight;
            centerRight = i;
            if(sideRight > i)
                sideRight = i;
        }
        if(!std::isinf(lidar[i+320])){
            rayLengths += lidar[i+320];
            --weight;
            if(centerLeft <  std::abs(i-319))
                centerLeft = std::abs(i-319);
            sideLeft = std::abs(i-319);
        }
    }
    //When driving down our current grid and the car stabilizes, centerRight and centerLeft are between 260-262 and drops down to 209 on intersections.
    // std::cout << "rayLengths: " << rayLengths  << "\n" << "weight: " << "\n" << weight << "\n" << "sideLeft "  << "centerLeft " << "centerRight " << "sideRight" << "\n" << sideLeft << "\t" << centerLeft << "\t" << centerRight << "\t" << sideRight << "\n" << std::endl;
    // std::cout << "Weight: ";
    // std::cout << weight << std::endl;
    //printf("Steering angle: %f\n", this->steeringAngle);
    if(this->atIntersection==0){
        // this->Accel();
        this->SetTargetDirection(this->GetDirection() + Angle(weight*PI/320));
    } //else if (this->GetSpeed() > this->maxCarSpeed){
    //     this->Brake();
    // }
}

// Drive in a straight line until it passes LON: 0.000200
void sdcCar::DriveStraightThenStop(){
     double targetLon = sdcSensorData::GetLongitude();
//     printf("targetLon: %f\n", targetLon);
     if (targetLon > 0.001) {
         this->Brake();
     } else {
         this->Accel();
     }
}

void sdcCar::DriveStraightThenTurn(){
    double targetLon = sdcSensorData::GetLongitude();
    Angle direction = this->GetDirection();
    this->Accel();
    //     printf("targetLon: %f\n", targetLon);
    if (targetLon > 0.0005) {
        this->SetTargetDirection(Angle(-PI/2));
    }
}
//Uses the front LIDAR sensor to detect an intersection.
void sdcCar::DetectIntersection(){
  std::vector<double> lidar = sdcSensorData::GetLidarRays();
  std::vector<std::vector<int>> views;
  std::vector<int> leftView;
  std::vector<int> rightView;
  int rayLengths = 0;
  int centerRight = 404;
  int sideRight = 404;
  int centerLeft = -404;
  int sideLeft = -404;
  int numrays = lidar.size();
  //With this loop we track the areas of view the car has.
  for (int i = 0; i < numrays; ++i) {
      if(!std::isinf(lidar[i])){
          if(i>319){
              rayLengths += lidar[i];
              leftView.push_back(i);
              if(centerLeft <  std::abs(i-639))
                  centerLeft = std::abs(i-639);
              sideLeft = std::abs(i-639);

          } else {
              rayLengths += lidar[i];
              rightView.push_back(i);
              centerRight = i;
              if(sideRight > i)
                  sideRight = i;
          }
      } else {
        if (leftView.size()!=0){
          views.push_back(leftView);
          leftView.clear();
        }
        if(rightView.size()!=0){
          views.push_back(rightView);
          rightView.clear();
        }
      }

  }
  if (leftView.size()!=0){
    views.push_back(leftView);
    leftView.clear();
  }
  if(rightView.size()!=0){
    views.push_back(rightView);
    rightView.clear();
  }
  //  std::cout <<"views.size(): " << views.size() << "\n" << "rayLengths: " << rayLengths  << "\n" << "sideLeft "  << "centerLeft " << "centerRight " << "sideRight" << "\n" << sideLeft << "\t" << centerLeft << "\t" << centerRight << "\t" << sideRight << std::endl;

  //Based off of how many fields of view we have an what we can see we try to turn.
  //When we are almost at an intersection slow down.
  if(this->atIntersection == 0 && views.size() == 4 && sideRight == 0 && sideLeft == 0){
    this->SetTargetSpeed(1);
    this->atIntersection = 1;
    //std::cout <<"views.size(): " << views.size() << "\n" << "rayLengths: " << rayLengths  << "\n" << "sideLeft "  << "centerLeft " << "centerRight " << "sideRight" << "\n" << sideLeft << "\t" << centerLeft << "\t" << centerRight << "\t" << sideRight << std::endl;
    //std::cout << "SLOW DOWN BUDDY!!!!" << std::endl;
    //std::cout << this->GetSpeed() << std::endl;
  }else if (this->atIntersection == 1 && views.size() > 1 && views.size() < 4 && (sideLeft != 0 && sideRight != 0)){
      this->atIntersection = 4;
      // std::cout << this->targetDirection << std::endl;
      this->SetTargetDirection(BAD_DIRECTION);
      BAD_DIRECTION = PI/2 + BAD_DIRECTION;
      // std::cout << this->targetDirection << std::endl;
      //std::cout <<"views.size(): " << views.size() << "\n" << "rayLengths: " << rayLengths  << "\n" << "sideLeft "  << "centerLeft " << "centerRight " << "sideRight" << "\n" << sideLeft << "\t" << centerLeft << "\t" << centerRight << "\t" << sideRight << std::endl;
       //std::cout << "INTERSECTION TURN!!!!" << std::endl;
       //std::cout << this->GetSpeed() << std::endl;
      // std::cout << "Longitude: " << sdcSensorData::GetLongitude() << std::endl;
      // std::cout << "Latitude: " << sdcSensorData::GetLatitude() << std::endl;
      // std::fstream gridCSV;
      // gridCSV.open("gps_units.csv", std::ios_base::app);
      // gridCSV << "\n" << GRIDX << "," << 105 << "," << sdcSensorData::GetLongitude() << "," << sdcSensorData::GetLatitude();
      // if (GRIDX == 400)
      //     GRIDX = 100;
      // else
      //     GRIDX += 100;
      // gridCSV.close();
  } else if (this->atIntersection == 2 && sideRight == 0 && sideLeft == 0)
      this->atIntersection = 3;
  else if (this->atIntersection == 3 && sideRight != 0)
      this->atIntersection = 4;
  else if (this->atIntersection == 4 && this->GetDirection().angle < this->targetDirection.angle && this->GetDirection().angle > this->targetDirection.angle - PI/8){
      //std::cout <<"views.size(): " << views.size() << "\n" << "rayLengths: " << rayLengths  << "\n" << "sideLeft "  << "centerLeft " << "centerRight " << "sideRight" << "\n" << sideLeft << "\t" << centerLeft << "\t" << centerRight << "\t" << sideRight << std::endl;
      //std::cout << "DONE TURNING!!!!!" << std::endl;
      //std::cout << this->GetSpeed() << std::endl;
      this->SetTargetSpeed(6);
      this->atIntersection = 0;
  }


    // this->atIntersection = 0;

  // std::cout << "Intersection: " << this->atIntersection << "\n" << std::endl;
}


// Car follows an object directly in front of it and slows down to stop when it starts to get close
void sdcCar::Follow() {
  std::vector<double> lidar = sdcSensorData::GetLidarRays();
  int numrays = lidar.size();

  std::vector<std::pair<int,int>> objectsInView;
  int lastIndex = -1;
  for(int i = 0; i < numrays; i++) {
    if(!std::isinf(lidar[i])){
      if (lastIndex < 0) {
        lastIndex = i;
      }
    } else {
      if (lastIndex > 0) {
        objectsInView.push_back(std::make_pair(lastIndex, i-1));
        lastIndex = -1;
      }
    }
    if(lastIndex > 0){
      objectsInView.push_back(std::make_pair(lastIndex, numrays-1));
    }
  }
  //std::cout << objectsInView[0].first << "   " << objectsInView[0].second << std::endl;
  //std::cout << typeid(objectsInView[0].first).name() << std::endl;


  if(numrays == 0) return;
  this->SetTargetSpeed(5);
  for(int i = 315; i < 326; i++){
    if (!std::isinf(lidar[i]) && (lidar[i] <= 10)) {
      this->SetTargetSpeed(1);
      break;
    }
  }

  // for (int i = 0; i < numrays; ++i) {
  //   if (i >= 315 && i <= 325 && std::isinf(lidar[i])) {
  //     this->SetTargetSpeed(6);
  //   } else if (!std::isinf(lidar[i]) && (lidar[i] <= 10)) {
  //     this->SetTargetSpeed(0);
  //   }
  // }
}
