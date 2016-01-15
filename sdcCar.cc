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

const int ARBITRARY_CUTOFF_POINT_1 = 50;

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
    this->steeringAngle = 0.0;
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
    
    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = pose.rot.GetYaw();
    
    

    
    this->Drive();
    
    
    
    
    
  // Compute the angle of the front wheels.
  double wheelAngle = this->steeringAngle / this->steeringRatio;

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
{
}

void sdcCar::ApplyMovementForce(double amt){
    if(amt > 0){
        this->gas = std::min(5.0, amt);
        this->brake = 0.0;
    } else{
        this->gas = 0.0;
        if(this->IsMovingForwards()){
            this->brake = std::max(-10.0, amt);
        } else{
            this->brake = 0.0;
        }
    }
}

bool sdcCar::IsMovingForwards(){
    math::Vector3 velocity = this->velocity;
    double velAngle = atan2(velocity.y, velocity.x);
    return std::abs(velAngle) <= 3.14159265359/2.;
}

/*
 * Default: 0.5
 */
void sdcCar::Accel(double amt){
    amt = 3;
    this->ApplyMovementForce(amt);
}

/*
 * Default: 1.0
 */
void sdcCar::Brake(double amt){
    amt = 7;
    this->ApplyMovementForce(-amt);
}

/*
 * Negative numbers turn left
 * Positive numbers turn right
 */
void sdcCar::Steer(double angle){
    this->steeringAngle = angle;
}

////////////////////////////////
////////////////////////////////
// BEGIN THE BRAIN OF THE CAR //
////////////////////////////////
////////////////////////////////

void sdcCar::Drive()
{
    //this->Accel();
    //    if(sdcLaserSensor::IsAllInf()){
    //        this->Accel();
    //    }else{
    //        this->Brake();
    //    }
    
//    this->CheckIfOnCollisionCourse();
//    this->TurnRightIfObjectAhead();
//    this->DriveStraightThenStop();
//    this->DriveToCoordinates(0.0005, 0.0005);
    this->WalledDriving();
}

// Turns right when range of rays is ARBITRARY_CUTOFF_POINT_1 or larger, continues forward if not
void sdcCar::CheckIfOnCollisionCourse(){
    std::vector<double>* nonInfAngles = sdcSensorData::GetNonInfAngles();
    //if (nonInfAngles->size() > ARBITRARY_CUTOFF_POINT_1) {
    double RayRange = sdcSensorData::GetRangeInFront();
    if (RayRange < 10.0) {
        this->Brake();
//        this->Steer(5);
    } else {
        this->Steer(0);
        this->Accel();
    }
}

// Drive with walled roads
void sdcCar::WalledDriving(){
    std::vector<double> lidar = sdcSensorData::GetLidarRays();
    if(lidar.size() > 0){
        std::cout << lidar.size() << std::endl;
        std::cout << lidar.at(0) << std::endl;
        std::cout << lidar.at(lidar.size()-1) << "\n" << std::endl;
    }
    this->Accel();
    
    int weight = 0;
    int numrays = lidar.size()/2;
    for (int i =0; i < numrays; ++i) {
        if(!std::isinf(lidar[i])){
            ++weight;
        }
        if(!std::isinf(lidar[i+320])){
            --weight;
        }
    }
    std::cout << "Weight: ";
    std::cout << weight << std::endl;
    printf("Steering angle: %f\n", this->steeringAngle);
    this->Steer(-weight/5);
    //std::cout << (*lidar).size() << std::endl;
    //std::cout << (*lidar)[320] << std::endl;
    //std::cout << (*lidar)[639] << std::endl;
}

//Car turns right when it sees an object <10 units away; when it's gone it will continue straight
void sdcCar::TurnRightIfObjectAhead(){
    double RayRange = sdcSensorData::GetRangeInFront();
    if (RayRange < 10.0) {
        this->Steer(8);
    } else {
        this->Steer(0);
        this->Accel();
    }
}


// Drive in a straight line until it passes LON: 0.000200

void sdcCar::DriveStraightThenStop(){
     double targetLon = sdcSensorData::GetLongitude();
//     printf("targetLon: %f\n", targetLon);
     if (targetLon > 0.0005) {
         this->Brake();
     } else {
         this->Accel();
     }
 }


void sdcCar::DriveToCoordinates(double lat, double lon){
    double currentLat = sdcSensorData::GetLatitude();
    double currentLon = sdcSensorData::GetLongitude();
    math::Vector2d coordinate = sdcSensorData::GetCurrentCoord();
    if (currentLon != lon) {
        if (currentLon + 0.00015 > lon) {
            this->Brake();
        } else if (currentLon < lon) {
            this->Accel();
        }
    } else if (currentLon == lon) {
        Steer(0);
    }
    if (currentLat != lat && (std::abs(currentLon - lon) < 0.0001)) {
        //double distance = coordinate.Distance(math::Vector2d(lat,lon));
        //std::cout << "Distance is:" << distance << "\n\n";
        if (currentLat > lat) {
            
            int weight = 0;
            // Do trig to find out if gps is aligned with lat

            
            this->Steer(7);
            this->Accel();
            
        } else if (currentLat < lat) {
            if ((currentLon - lon) >= 0) {
                this->Steer(0);
                this->Steer(-7);
                this->Accel();
            } else if ((currentLon -lon) < 0) {
                this->Steer(0);
                this->Steer(7);
                this->Accel();
            }
        }
    } else if (currentLat == lat) {
        this->Steer(0);
        this->Brake();
    }
}

// Work in progress to turn car 90 degress right
void sdcCar::TurnRight() {
    
}
