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


/*
 * Based on UtilityCart.cc written by Nate Koenig, sdcCar provides both
 * interaction with Gazebo's simulation environment as well as logic to
 * make it behave intelligently in a variety of situations. This is the main
 * class used in the Self-Driving Comps project.
 *
 * Physics parameters and Gazebo interfacing are based on UtilityCart.
 */

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "sdcCar.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(sdcCar)

// SDC-defined constants
const double PI = 3.14159265359;

const double DIRECTION_MARGIN_OF_ERROR = 0.00855;
const double STEERING_MARGIN_OF_ERROR = 0.05;
const int LIDAR_DETECTION_MARGIN_OF_ERROR = 2;

// How fast the car turns each update
const double STEERING_ADJUSTMENT_RATE = 0.02;

// How much we can turn the "steering wheel"
const double STEERING_RANGE = 5 * PI;

const double CAR_WIDTH = 0.8;
const double CAR_LENGTH = 2.0;

// The width of the channel in front of the car for which we count objects as
// being directly in front of the car
const double FRONT_OBJECT_COLLISION_WIDTH = CAR_WIDTH + 0.5;

const sdcAngle NORTH = sdcAngle(PI/2);
const sdcAngle SOUTH = sdcAngle(3*PI/2);
const sdcAngle EAST = sdcAngle(0);
const sdcAngle WEST = sdcAngle(PI);

//dijkstra's stuff
std::vector<int> unvisited;
std::vector<sdcIntersection> intersections;
const int size = 5;
const std::pair<double,double> destination = {0,0};

std::vector<sdcWaypoint> WAYPOINT_VEC;


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

    // If not in avoidance, check if we should start following the thing
    // in front of us. If following is done, kick out to default state
    if(this->currentState != intersection && this->currentState != avoidance){
        // If there's a stop sign, assume we're at an intersection
        if(this->ignoreStopSignsCounter == 0 && sdcSensorData::stopSignFrameCount > 5){
            this->currentState = intersection;
        }

        // If something is ahead of us, default to trying to follow it
        if (this->ObjectDirectlyAhead()){
            this->currentState = follow;
        }else if(this->currentState == follow && !this->isTrackingObject){
            this->currentState = DEFAULT_STATE;;
        }

        // Look for objects in danger of colliding with us, react appropriately
        if (this->ObjectOnCollisionCourse()){
            this->currentState = avoidance;
        }
    }

    this->ignoreStopSignsCounter = fmax(this->ignoreStopSignsCounter - 1, 0);


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
        // this->Stop();
        //this->WaypointDriving(WAYPOINT_VEC);
        break;

        // At a stop sign, performing a turn
        case intersection:
        if(this->stoppedAtSign && this->stationaryCount > 2000){
            this->currentState = DEFAULT_STATE;
            this->ignoreStopSignsCounter = 3000;
        }else if(this->stoppedAtSign && this->GetSpeed() < 0.5){
            this->stationaryCount++;
        }else if(!this->stoppedAtSign && sdcSensorData::sizeOfStopSign > 6000){
            this->Stop();
            this->stoppedAtSign = true;
            this->stationaryCount = 0;
        }

        break;

        // Follows object that is going in same direction/towards same target
        case follow:
        this->Follow();
        // Handle lane driving
        break;

        // Smarter way to avoid objects; stopping, swerving, etc.
        case avoidance:
        // Cases: stop, swerve, go around
        this->Avoidance();
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
}

/*
 * Handles turning based on the value of targetDirection. Calculates both which direction
 * to turn and by how much, as well as turning the actual wheel
 */
void sdcCar::MatchTargetDirection(){
    sdcAngle directionAngleChange = this->GetDirection() - this->targetDirection;
    // If the car needs to turn, set the target steering amount
    if (!directionAngleChange.WithinMargin(DIRECTION_MARGIN_OF_ERROR)) {
        // The steering amount scales based on how far we have to turn, with upper and lower limits
        double proposedSteeringAmount = fmax(fmin(-this->turningLimit*tan(directionAngleChange.angle/-2), this->turningLimit), -this->turningLimit);

        // When reversing, steering directions are inverted
        if(!this->reversing){
            this->SetTargetSteeringAmount(proposedSteeringAmount);
        }else{
            this->SetTargetSteeringAmount(-proposedSteeringAmount);
        }
    }

    // Check if the car needs to steer, and apply a small turn in the corresponding direction
    if (!(std::abs(this->targetSteeringAmount - this->steeringAmount) < STEERING_MARGIN_OF_ERROR)) {
        if (this->steeringAmount < this->targetSteeringAmount) {
            this->steeringAmount = this->steeringAmount + STEERING_ADJUSTMENT_RATE;
        }else{
            this->steeringAmount = this->steeringAmount - STEERING_ADJUSTMENT_RATE;
        }
    }
}

/*
 * Attempts to match the current target speed
 */
void sdcCar::MatchTargetSpeed(){
    // Invert all the values if the car should be moving backwards
    int dirConst = this->reversing ? -1 : 1;

    // If the car is moving the wrong direction or slower than the target speed, press on the gas
    if((this->reversing && this->IsMovingForwards()) || (!this->reversing && !this->IsMovingForwards()) || (this->GetSpeed() < this->targetSpeed)){
        this->gas = 1.0 * dirConst;
        this->brake = 0.0;
    } else if(this->GetSpeed() > this->targetSpeed){
        // If the car is moving faster than the target speed, brake to slow down
        this->gas = 0.0;
        if(this->reversing != this->IsMovingForwards()){
            this->brake = -2.0 * dirConst;
        } else {
            // If the car is drifting in the opposite direction it should be, don't brake
            // as this has the side effect of accelerating the car in the opposite direction
            this->brake = 0.0;
        }
    }
}

/*
 * Drive from point to point in the given list
 */
void sdcCar::WaypointDriving(std::vector<sdcWaypoint> WAYPOINT_VEC) {
    int progress = this->waypointProgress;
    if(progress < WAYPOINT_VEC.size()){
        // Pull the next waypoint and set the car to drive towards it


        this->Accelerate();

        // Check if the car is close enough to the target to move on
        double distance = sqrt(pow(WAYPOINT_VEC[progress].pos.first - this->x,2) + pow(WAYPOINT_VEC[progress].pos.second - this->y,2));
        if (distance < 7) {
            this->turning = true;
        }
        if(this->turning == true){
            this->SetTurningLimit(20);
            GridTurning(WAYPOINT_VEC[progress].waypointType);
        } else {
            math::Vector2d nextTarget = {WAYPOINT_VEC[progress].pos.first,WAYPOINT_VEC[progress].pos.second};
            sdcAngle targetAngle = AngleToTarget(nextTarget);
            this->SetTargetDirection(targetAngle);
            // this->LanedDriving();
        }
    } else if(this->isFixingParking){
        this->isFixingParking = false;
        this->currentState = parking;
        this->currentPerpendicularState = straightPark;
    } else {
        this->currentState = stop;
    }
}

/*
 * Uses camera data to detect lanes and sets targetDirection to stay as close
 * as possible to the midpoint.
 */
void sdcCar::LanedDriving() {
    int lanePos = sdcSensorData::LanePosition();
    this->SetTurningLimit(sdcSensorData::GetNewSteeringMagnitude());
    if (!(lanePos > 320 || lanePos < -320)) {
        // It's beautiful don't question it
        sdcAngle laneWeight = sdcAngle(tan(lanePos/(PI*66.19))/10);
        this->SetTargetDirection(this->GetDirection() + laneWeight);
    }
}

/*
 * Car follows an object directly in front of it and slows down to stop when it starts to get close
 */
void sdcCar::Follow() {
    // There's nothing in front of the car, so break out of follow
    if(this->frontObjects.size() == 0){
        this->isTrackingObject = false;
        this->currentState = DEFAULT_STATE;
        return;
    }

    // The default object to follow is directly in front of the car, the max range away
    sdcVisibleObject tracked = sdcVisibleObject(sdcLidarRay(0, sdcSensorData::GetLidarMaxRange(FRONT)), sdcLidarRay(0, sdcSensorData::GetLidarMaxRange(FRONT)), sdcSensorData::GetLidarMaxRange(FRONT));

    // Already tracking an object, find it again
    if(this->isTrackingObject){
        bool foundTrackedObject = false;
        for(int i = 0; i < this->frontObjects.size(); i++){
            sdcVisibleObject obj = this->frontObjects[i];
            if(obj.IsTracking()){
                tracked = obj;
                foundTrackedObject = true;
                break;
            }
        }
        if(!foundTrackedObject){
            this->isTrackingObject = false;
            return;
        }
    }else{
        // Not tracking an object, find one that's in front of the car
        // and start tracking it
        for(int i = 0; i < this->frontObjects.size(); i++){
            sdcVisibleObject obj = this->frontObjects[i];
            if(this->IsObjectDirectlyAhead(obj)){
                tracked = obj;
                tracked.SetTracking(true);
                this->isTrackingObject = true;
                this->frontObjects[i] = tracked;
                break;
            }
        }
    }

    // After the above loops, if not following anything just return
    if(!this->isTrackingObject) return;

    math::Vector2d objCenter = tracked.GetCenterPoint();
    double objSpeed = tracked.GetEstimatedYSpeed();

    // Scale our speed based on how far away the tracked object is
    // The equation is 'scaledSpeed = (objY - 10)^3 / 2000.' which
    // gives a scaled speed of 0 at y=10 and +-0.5 at y=20, y=0 respectively
    double scaledSpeed = pow(objCenter.y - 10, 3) / 2000.;

    // Adjust the target speed based on the speed of the object, our speed,
    // and the above calculated scaled speed
    double newTargetSpeed = objSpeed + this->GetSpeed() + scaledSpeed;
    this->SetTargetSpeed(newTargetSpeed);

    // If the new target speed is sufficiently low, count the car as stationary
    if(newTargetSpeed < 0.3){
        this->stationaryCount++;
    }else{
        this->stationaryCount = 0;
    }

    // If the car has been stationary for sufficiently long, stop following and start
    // trying to navigate around the object in front of it
    if(this->stationaryCount > 2000){
        this->currentState = avoidance;
        this->currentAvoidanceState = navigation;
    }

    // Set the direction of the car to be angled at the tracked object
    if(objCenter.x != 0){
        this->SetTargetDirection(this->GetOrientation() - sdcAngle(PI / 2.) + sdcAngle(atan2(objCenter.y, objCenter.x)));
    }else{
        this->SetTargetDirection(this->GetOrientation());
    }
}

/*
 * In avoidance, the car's only concern is not hitting objects. Provides a couple emergency methods, one
 * for stopping and one for swerving. Also provides a navigation case for maneuvering around objects in front
 * of the car
 */
void sdcCar::Avoidance(){
    // If there's nothing in front of the car and it's not in the middle
    // of a navigation operation, exit the avoidance state
    if(this->frontObjects.size() == 0 && !this->trackingNavWaypoint){
        this->currentState = DEFAULT_STATE;
        this->currentAvoidanceState = notAvoiding;
        return;
    }

    // Get lists of objects that are moving quickly towards us,
    // and objects that are close to us
    std::vector<sdcVisibleObject> fastObjects, furiousObjects;
    if (this->frontObjects.size() > 0) {
        for(int i = 0; i < this->frontObjects.size(); i++){
            if(this->IsObjectTooFast(this->frontObjects[i])){
                fastObjects.push_back(this->frontObjects[i]);
            }
            if(this->IsObjectTooFurious(this->frontObjects[i])){
                furiousObjects.push_back(this->frontObjects[i]);
            }
        }
    }

    // For emergency swerve, check which side the object is coming from so
    // we can go away from it
    bool isObjectOnRight = true;

    // Objects moving relatively quickly towards the car are the highest priority. If any
    // of these exist, react accordingly
    if(fastObjects.size() > 0){
        bool setState = false;
        for(int i = 0; i < fastObjects.size(); i++){
            // If the object is moving faster than the car is, or the car is moving significantly faster than the object,
            // try and swerve as there isn't enough time to stop
            double objSpeed = sqrt(pow(fastObjects[i].GetEstimatedXSpeed(),2) + pow(fastObjects[i].GetEstimatedYSpeed() - this->GetSpeed(),2));
            if(objSpeed > this->GetSpeed() || this->GetSpeed() > objSpeed + 4){
                this->currentAvoidanceState = emergencySwerve;
                if(fastObjects[i].GetCenterPoint().x < 0){
                  isObjectOnRight = false;
                }
                setState = true;
                break;
            }
        }

        // If the state hasn't been set to swerve, the car should be able to stop and thus
        // avoid a collision
        if(!setState){
            this->currentAvoidanceState = emergencyStop;
        }
    }else if(furiousObjects.size() > 0){
        // There are objects very close to the car, but not necessarily in danger of running into
        // it. Try and navigate around them
        this->currentAvoidanceState = navigation;
    }else if(this->currentAvoidanceState != navigation && this->currentAvoidanceState != emergencyStop){
        // No dangerous objects were found, and the car is not in the middle of navigating around
        // objects in front of it. Exit to default state
        this->currentAvoidanceState = notAvoiding;
        this->currentState = DEFAULT_STATE;
        return;
    }

    switch(this->currentAvoidanceState){
        // Stop, hard.
        case emergencyStop:

        this->Stop();
        this->SetBrakeRate(10);
        break;

        // Make an emergency turn and attempt to accelerate past
        // the incoming danger
        case emergencySwerve:

        if(isObjectOnRight){
          this->SetTargetDirection(this->GetOrientation() + PI/4);
        }else{
          this->SetTargetDirection(this->GetOrientation() - PI/4);
        }
        this->SetTargetSpeed(10);
        this->SetAccelRate(10);
        break;

        // Carefully maneuver around perceived obstacles
        case navigation:
        {
            // Set the target speed very low, and if the car is moving
            // sufficiently slowly increase the rate we can turn
            this->SetTargetSpeed(1);
            if(this->GetSpeed() < 2) {
                this->SetTurningLimit(30.0);
            }

            // The car is currently driving to a custom waypoint that was already determined
            // to be a safe target. Keep moving towards it
            if (this->trackingNavWaypoint) {
                sdcAngle targetAngle = AngleToTarget(this->navWaypoint);
                this->SetTargetDirection(targetAngle);

                if(sqrt(pow(this->navWaypoint.x - this->x,2) + pow(this->navWaypoint.y - this->y,2)) < 1) {
                    this->trackingNavWaypoint = false;
                    this->SetTurningLimit(10.0);
                }
            } else {
                // At this point, need to find a gap in the objects presented ahead of the car and
                // begin driving towards it
                double maxWidth = -1;
                double dist = 0;
                double prevDist = 0;
                sdcAngle targetAngle = this->GetOrientation();

                // If there isn't an object directly in front of us, we can safely drive forward
                if (!this->ObjectDirectlyAhead()) {
                    this->navWaypoint = math::Vector2d(this->x + cos(this->GetOrientation().angle) * 4, this->y + sin(this->GetOrientation().angle) * 4);
                    this->trackingNavWaypoint = true;
                    break;
                }

                // Loop through all objects in front of the car, find the space with the largest width
                // and store the point between them
                math::Vector2d prevPoint = math::Vector2d(this->frontObjects[0].right.GetLateralDist() + FRONT_OBJECT_COLLISION_WIDTH + 0.2, this->frontObjects[0].right.GetLongitudinalDist());
                // Angle closest to 0 that it's safe to drive through
                double bestMargin = 2 * PI;
                math::Vector2d curPoint;
                for (int i = 0; i < this->frontObjects.size(); i++) {
                    curPoint = this->frontObjects[i].right.GetAsPoint();
                    if (curPoint.Distance(prevPoint) > FRONT_OBJECT_COLLISION_WIDTH) {
                        // Point is on our left
                        if (curPoint.x < 0) {
                            math::Vector2d newPoint = math::Vector2d(prevPoint.x - FRONT_OBJECT_COLLISION_WIDTH/2., prevPoint.y);
                            sdcAngle newAngle = atan2(newPoint.x, newPoint.y);
                            if (newAngle.FindMargin(sdcAngle(0)) < bestMargin) {
                                bestMargin = newAngle.FindMargin(sdcAngle(0)).angle;
                                this->navWaypoint = math::Vector2d(this->x + cos((newAngle + this->GetOrientation()).angle)*newPoint.Distance(math::Vector2d(0,0)), this->y + sin((newAngle + this->GetOrientation()).angle)*newPoint.Distance(math::Vector2d(0,0)));
                            }
                        }
                        // Point is on our right
                        else {
                            math::Vector2d newPoint = math::Vector2d(curPoint.x + FRONT_OBJECT_COLLISION_WIDTH/2., curPoint.y);
                            sdcAngle newAngle = atan2(newPoint.x, newPoint.y);
                            if (newAngle.FindMargin(sdcAngle(0)) < bestMargin) {
                                bestMargin = newAngle.FindMargin(sdcAngle(0)).angle;
                                this->navWaypoint = math::Vector2d(this->x + cos((newAngle + this->GetOrientation()).angle)*newPoint.Distance(math::Vector2d(0,0)), this->y + sin((newAngle + this->GetOrientation()).angle)*newPoint.Distance(math::Vector2d(0,0)));
                            }
                        }
                    }
                    prevPoint = this->frontObjects[i].left.GetAsPoint();
                }
                curPoint = math::Vector2d(prevPoint.x, 0);
                if (curPoint.Distance(prevPoint) > FRONT_OBJECT_COLLISION_WIDTH + 0.2){
                    math::Vector2d newPoint = math::Vector2d(prevPoint.x - FRONT_OBJECT_COLLISION_WIDTH/2., prevPoint.y);
                    sdcAngle newAngle = atan2(newPoint.x, newPoint.y);
                    if (newAngle.FindMargin(sdcAngle(0)) < bestMargin) {
                        this->navWaypoint = math::Vector2d(this->x + cos((newAngle + this->GetOrientation()).angle)*newPoint.Distance(math::Vector2d(0,0)), this->y + sin((newAngle + this->GetOrientation()).angle)*newPoint.Distance(math::Vector2d(0,0)));
                    }
                }

                this->trackingNavWaypoint = true;
            }
            break;
        }

        case notAvoiding:
        default:
        this->currentState = DEFAULT_STATE;
        break;
    }

}

/*
 * Executes a turn at an intersection
 */
void sdcCar::GridTurning(int turn){
    int progress = this->waypointProgress;
    if(turn == 3){
        this->waypointProgress++;
        this->currentState = stop;
        return;
    } else if (turn == 0){
        this->waypointProgress++;
        this->turning = false;
        return;
    }
    math::Vector2d nextTarget = {WAYPOINT_VEC[progress+1].pos.first,WAYPOINT_VEC[progress+1].pos.second};
    sdcAngle targetAngle = AngleToTarget(nextTarget);
    this->SetTargetDirection(targetAngle);
    sdcAngle margin = this->GetOrientation().FindMargin(targetAngle);
    if(margin < .1 && margin > -.1){
        this->turning = false;
        this->waypointProgress++;
    }
}


/*
 * Perpendicular back parking algorithm
 */
void sdcCar::PerpendicularPark(){
    // Get back and side lidar rays that detect whether the back left and right bumpers of the car
    // will collide with other objects
    std::vector<double> backLidar = sdcSensorData::GetLidarRays(BACK);
    std::vector<double> rightBackSideLidar = sdcSensorData::GetLidarRays(SIDE_RIGHT_BACK);
    std::vector<double> leftBackSideLidar = sdcSensorData::GetLidarRays(SIDE_LEFT_BACK);
    std::vector<double> rightFrontSideLidar = sdcSensorData::GetLidarRays(SIDE_RIGHT_FRONT);
    std::vector<double> leftFrontSideLidar = sdcSensorData::GetLidarRays(SIDE_LEFT_FRONT);
    std::vector<double> backRightBound;
    std::vector<double> backMidBound;
    std::vector<double> backLeftBound;
    std::vector<double> backRightSideBound;
    std::vector<double> backLeftSideBound;
    bool isSafe = true;
    std::vector<sdcWaypoint> fix;
    math::Vector2d pos = sdcSensorData::GetPosition();
    this->SetTurningLimit(30.0);

    // Store vector of rays from the back lidar that detect objects behind the car, specifically the middle
    // and far right and left rays
    int numBackRightRays = sdcSensorData::GetLidarNumRays(SIDE_RIGHT_BACK);
    int rightSideBoundRange = numBackRightRays / 16;
    int midBackRightSideRay = numBackRightRays / 2;
    if(rightBackSideLidar.size() != 0){
        for(int i = 0; i < rightSideBoundRange; i++){
            backRightSideBound.push_back(rightBackSideLidar[i]);
        }
        for(int j = midBackRightSideRay - rightSideBoundRange / 2; j < midBackRightSideRay + rightSideBoundRange / 2; j++){
            backRightSideBound.push_back(rightBackSideLidar[j]);
        }
        for(int k = numBackRightRays - rightSideBoundRange; k < numBackRightRays; k++){
            backRightSideBound.push_back(rightBackSideLidar[k]);
        }
    }

    // Store vector of rays from the back left side lidar that detects objects on the back left side, specifically
    // the middle rays and far right and left rays
    int numBackLeftRays = sdcSensorData::GetLidarNumRays(SIDE_LEFT_BACK);
    int leftSideBoundRange = numBackLeftRays / 16;
    int midBackLeftSideRay = numBackLeftRays / 2;
    if(leftBackSideLidar.size() != 0){
        for(int i = 0; i < leftSideBoundRange; i++){
            backLeftSideBound.push_back(leftBackSideLidar[i]);
        }
        for(int j = midBackLeftSideRay - leftSideBoundRange / 2; j < midBackLeftSideRay + leftSideBoundRange /2; j++){
            backLeftSideBound.push_back(leftBackSideLidar[j]);
        }
        for(int k = numBackLeftRays - leftSideBoundRange; k < numBackLeftRays; k++){
            backLeftSideBound.push_back(leftBackSideLidar[k]);
        }
    }

    // Store vector of rays from the back right side lidar that detects objects on the back right side, specifically
    // the middle rays and far right and left rays
    int numBackRays = sdcSensorData::GetLidarNumRays(BACK);
    int backBoundRange = numBackRays / 20;
    int midBackRay = numBackRays / 2;
    if(backLidar.size() != 0){
        for(int i = 0; i < backBoundRange; i++){
            backRightBound.push_back(backLidar[i]);
        }
        for(int j = midBackRay - backBoundRange / 2; j < midBackRay + backBoundRange / 2; j++){
            backMidBound.push_back(backLidar[j]);
        }
        for(int k = numBackRays - backBoundRange; k < numBackRays; k++){
            backLeftBound.push_back(backLidar[k]);
        }
    }

    // Determines what perpendicular parking state the car should be in based on lidar data
    switch(this->currentPerpendicularState)
    {
        // Done with perpendicular parking and sets car to stop state
        case donePark:
        this->StopReverse();
        this->Stop();
        this->SetTurningLimit(10.0);
        this->parkingSpotSet = false;
        this->currentState = stop;
        break;

        // Pulls forward if it gets too close to anything in the back
        case frontPark:
        if(backLidar.size() != 0){
            for(int i = 0; i < backRightBound.size(); i++) {
                if(backRightBound[i] < 1.5){
                    isSafe = false;
                }
            }
            for(int j = 0; j < backMidBound.size(); j++) {
                if(backMidBound[j] < 2.0){
                    isSafe = false;
                }
            }
            for(int k = 0; k < backLeftBound.size(); k++) {
                if(backLeftBound[k] < 1.5){
                    isSafe = false;
                }
            }
        }

        if(isSafe){
            this->currentPerpendicularState = backPark;
            break;
        } else {
            this->StopReverse();
            this->SetTargetSpeed(0.5);
            sdcAngle margin = this->GetOrientation().FindMargin(this->targetParkingAngle);
            if(rightFrontSideLidar.size() > 0 && leftFrontSideLidar.size() > 0 && rightBackSideLidar.size() && leftBackSideLidar.size()){
                double rightSideMargins = std::abs(rightFrontSideLidar[sdcSensorData::GetLidarNumRays(SIDE_RIGHT_FRONT)/2] - rightBackSideLidar[sdcSensorData::GetLidarNumRays(SIDE_RIGHT_BACK)/2]);
                double leftSideMargins = std::abs(leftFrontSideLidar[sdcSensorData::GetLidarNumRays(SIDE_LEFT_FRONT)/2] - leftBackSideLidar[sdcSensorData::GetLidarNumRays(SIDE_LEFT_BACK)/2]);
                // Begin backing up into the parking space if the car is aligned with the space
                if(margin < 0.05 && rightSideMargins < 0.05 && leftSideMargins < 0.05){
                    this->parkingAngleSet = false;
                    this->currentPerpendicularState = straightPark;
                }
            }
            this->SetTargetDirection(targetParkingAngle);
            break;
        }
        break;

        // Backs up into parking spot until the space behind the car is good; only runs if the car
        // is aligned with the parking spot
        case straightPark:
        this->Reverse();
        this->SetTargetDirection(targetParkingAngle);
        this->SetTargetSpeed(0.5);
        if(backLidar[numBackRays / 2] < 0.5){
            this->currentPerpendicularState = donePark;
        }
        break;

        // Temporary stop state for the car while parking to help avoid hitting anything
        case stopPark:
        this->Stop();
        this->currentPerpendicularState = frontPark;
        break;

        // Backs into the parking spot
        case backPark:
        // If the car is too close to anything on the back or sides, stop and fix it
        if(backLidar.size() != 0){
            for(int i = 0; i < backRightBound.size(); i++) {
                if(backRightBound[i] < 0.7){
                    this->currentPerpendicularState = stopPark;
                }
            }
            for(int j = 0; j < backMidBound.size(); j++) {
                if(backMidBound[j] < 0.5){
                    this->currentPerpendicularState = stopPark;
                }
            }
            for(int k = 0; k < backLeftBound.size(); k++) {
                if(backLeftBound[k] < 0.7){
                    this->currentPerpendicularState = stopPark;
                }
            }
        }
        if(leftBackSideLidar.size() != 0){
            for(int l = 0; l < backLeftSideBound.size(); l++){
                if(leftBackSideLidar[l] < 0.25){
                    this->currentPerpendicularState = stopPark;
                }
            }
        }
        if(rightBackSideLidar.size() != 0){
            for(int m = 0; m < backRightSideBound.size(); m++){
                if(rightBackSideLidar[m] < 0.25){
                    this->currentPerpendicularState = stopPark;
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
        sdcAngle margin = this->GetOrientation().FindMargin(this->targetParkingAngle);
        if(rightFrontSideLidar.size() > 0 && leftFrontSideLidar.size() > 0 && rightBackSideLidar.size() && leftBackSideLidar.size()){
            double rightSideMargins = std::abs(rightFrontSideLidar[sdcSensorData::GetLidarNumRays(SIDE_RIGHT_FRONT)/2] - rightBackSideLidar[sdcSensorData::GetLidarNumRays(SIDE_RIGHT_BACK)/2]);
            double leftSideMargins = std::abs(leftFrontSideLidar[sdcSensorData::GetLidarNumRays(SIDE_LEFT_FRONT)/2] - leftBackSideLidar[sdcSensorData::GetLidarNumRays(SIDE_LEFT_BACK)/2]);
            if(margin < 0.05 && rightSideMargins < 0.05 && leftSideMargins < 0.05){
                this->parkingAngleSet = false;
                this->currentPerpendicularState = straightPark;
            }
        }
        break;
    }
}

/*
 * Parallel park algorithm
 */
void sdcCar::ParallelPark(){
    // Get back and front lidar rays for object detection in front and in back while parking
    std::vector<double> backLidar = sdcSensorData::GetLidarRays(BACK);
    std::vector<double> frontLidar = sdcSensorData::GetLidarRays(FRONT);
    std::vector<double> backRightBound;
    std::vector<double> backMidBound;
    std::vector<double> backLeftBound;
    std::vector<double> frontRightBound;
    std::vector<double> frontMidBound;
    std::vector<double> frontLeftBound;
    this->SetTurningLimit(30.0);

    // Store vector of rays from the back lidar that detect objects behind the car, specifically the middle
    // and far right and left rays
    int numBackRays = sdcSensorData::GetLidarNumRays(BACK);
    int backBoundRange = numBackRays / 20;
    int midBackRay = numBackRays / 2;
    if(backLidar.size() != 0){
        for(int i = 0; i < backBoundRange; i++){
            backRightBound.push_back(backLidar[i]);
        }
        for(int j = midBackRay / 2 - backBoundRange; j < midBackRay / 2 + backBoundRange; j++){
            backMidBound.push_back(backLidar[j]);
        }
        for(int k = numBackRays - backBoundRange; k < numBackRays; k++){
            backLeftBound.push_back(backLidar[k]);
        }
    }

    // Store vector of rays from the front lidar that detect objects in front of the car, specifically the middle
    // and far right and left rays
    int numFrontRays = sdcSensorData::GetLidarNumRays(FRONT);
    int frontBoundRange = numFrontRays / 64;
    int midFrontRay = numFrontRays / 2;
    if(frontLidar.size() != 0){
        for(int i = 0; i < frontBoundRange; i++){
            frontRightBound.push_back(frontLidar[i]);
        }
        for(int j = midFrontRay - frontBoundRange / 2; j < midFrontRay + frontBoundRange / 2; j++){
            frontMidBound.push_back(frontLidar[j]);
        }
        for(int k = numFrontRays - frontBoundRange; k < numFrontRays; k++){
            frontLeftBound.push_back(frontLidar[k]);
        }
    }

    // Determines what parallel parking state the car should be in while performing a parallel park
    switch(this->currentParallelState)
    {
        // Drive back while turning right into parking spot
        case rightBack:
        if(!parkingAngleSet){
            this->targetParkingAngle = this->GetOrientation();
            this->parkingAngleSet = true;
        } else {
            // Turn wheels left after turning 45 degrees
            if(this->GetOrientation() > targetParkingAngle + PI/4){
                this->currentParallelState = leftBack;
                break;
            }
            this->Reverse();
            this->SetTargetDirection(targetParkingAngle - PI/2);
            this->SetTargetSpeed(0.35);
            break;
        }

        // Drive back while turning left
        case leftBack:
        if(backLidar.size() != 0 && frontLidar.size() != 0){
            sdcAngle margin = this->GetOrientation().FindMargin(this->targetParkingAngle);
            double spaceMargin = std::abs(backLidar[numBackRays/2] - frontLidar[numFrontRays/2]);
            // If the car is aligned in the parking spot, begin driving forward
            if(margin < 0.01 &&  spaceMargin < 0.05){
                this->currentParallelState = straightForward;
                break;
            }
        }

        // If the car gets too close to anything behind it, pull forward while turning right
        if(backLidar.size() != 0){
            for(int i = 0; i < backRightBound.size(); i++) {
                if(backRightBound[i] < 0.5){
                    this->currentParallelState = rightForward;
                }
            }
            for(int j = 0; j < backMidBound.size(); j++) {
                if(backMidBound[j] < 0.3){
                    this->currentParallelState = rightForward;
                }
            }
            for(int k = 0; k < backLeftBound.size(); k++) {
                if(backLeftBound[k] < 0.5){
                    this->currentParallelState = rightForward;
                }
            }
        }
        this->SetTargetDirection(targetParkingAngle + PI/2);
        this->Reverse();
        this->SetTargetSpeed(0.35);
        break;

        // Drive forward while turning right
        case rightForward:
        if(backLidar.size() != 0 && frontLidar.size() != 0){
            sdcAngle margin = this->GetOrientation().FindMargin(this->targetParkingAngle);
            double spaceMargin = std::abs(backLidar[numBackRays/2] - frontLidar[numFrontRays/2]);
            if(margin < 0.01 &&  spaceMargin < 0.05){
                this->currentParallelState = straightForward;
                break;
            }
        }

        // Back up if the car gets too close to anything in front
        if(frontLidar.size() != 0){
            for(int i = 0; i < frontRightBound.size(); i++) {
                if(frontRightBound[i] < 0.9){
                    this->currentParallelState = leftBack;
                }
            }
            for(int j = 0; j < frontMidBound.size(); j++) {
                if(frontMidBound[j] < 0.5){
                    this->currentParallelState = leftBack;
                }
            }
            for(int k = 0; k < frontLeftBound.size(); k++) {
                if(frontLeftBound[k] < 0.9){
                    this->currentParallelState = leftBack;
                }
            }
        }
        this->StopReverse();
        this->SetTargetDirection(this->targetParkingAngle - PI/2);
        this->SetTargetSpeed(0.35);
        break;

        // Pull forward while in parking spot, finishes parallel parking if the car is aligned in the spot
        // and the space in front and back are roughly the same
        case straightForward:
        {
            double frontSpace = frontLidar[numFrontRays/2];
            double backSpace = backLidar[numBackRays/2];
            if(frontSpace == backSpace){
                this->currentParallelState = doneParallel;
                break;
            } else if(frontSpace > backSpace){
                double spaceMargin = std::abs(frontSpace - backSpace);
                if(spaceMargin < 0.01){
                    this->currentParallelState = doneParallel;
                    break;
                } else {
                    this->SetTargetDirection(targetParkingAngle);
                    this->StopReverse();
                    this->SetTargetSpeed(0.35);
                }
            } else {
                double spaceMargin = std::abs(frontSpace - backSpace);
                if(spaceMargin < 0.01){
                    this->currentParallelState = doneParallel;
                    break;
                } else {
                    this->SetTargetDirection(targetParkingAngle);
                    this->Reverse();
                    this->SetTargetSpeed(0.35);
                }
            }
            break;
        }

        // Finished parallel parking and sets current state to stop state
        case doneParallel:
        this->Stop();
        this->StopReverse();
        this->SetTurningLimit(10.0);
        this->currentState = stop;
        break;
    }
}

//////////////////////
// DIJKSTRA METHODS //
//////////////////////

//Generates a series of waypoints to get to the desired destination
void sdcCar::GenerateWaypoints(){
    GetNSEW();
    initializeGraph();
    const int start = getFirstIntersection();
    int dest;
    for(int i = 0; i < intersections.size(); ++i){
        if(intersections[i].waypoint.pos.first == destination.first && intersections[i].waypoint.pos.second == destination.second)
            dest = i;
    }
    std::vector<int> path;
    removeStartingEdge(start);
    path = dijkstras(start, dest);
    insertWaypointTypes(path, this->currentDir);
    for (int i = path.size()-1; i >=0; --i)
        WAYPOINT_VEC.push_back(intersections[path[i]].waypoint);
}

std::vector<int> sdcCar::dijkstras(int start, int dest) {
    std::vector<int> path;
    int current;
    intersections[start].dist = 0;
    intersections[start].previous = -1;
    double distance;

    // initializes the unvisited list by placing all of start's neighbors in it
    for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
    // push back each neighbor of the start into unvisited
    unvisited.push_back(intersections[start].neighbors_pairs[n].first);
    // set the distance of each neighbor to the distance of the edge
    // from start to neighbor and make neighbor previous = start
    intersections[intersections[start].neighbors_pairs[n].first].dist =
        intersections[start].neighbors_pairs[n].second;
    intersections[intersections[start].neighbors_pairs[n].first].previous =
        intersections[start].place;
    }

    // BFS using the unvisted FI FO vector, if unvisited is 0 then we have
    // visited all intersections
    while (unvisited.size() != 0) {
    current = unvisited[0];
    for (int n = 0; n < intersections[current].neighbors_pairs.size(); ++n) {
      // distance to the neighbor from current intersection
      distance = intersections[current].neighbors_pairs[n].second;
      // if the distance of the current intersection + the distance from
      // the current intersection to neighbor is smaller than the distance
      // to neighbor, update distance and previous
      if (intersections[intersections[current].neighbors_pairs[n].first].dist >
          intersections[current].dist + distance) {
        // update distance
        intersections[intersections[current].neighbors_pairs[n].first].dist =
            intersections[current].dist + distance;
        // update previous
        intersections[intersections[current].neighbors_pairs[n].first]
            .previous = intersections[current].place;
      }
      // if the neighbor has not been visited then push back into unvisited
      if (intersections[intersections[current].neighbors_pairs[n].first]
              .visited == 0) {
        // push back neighbor into unvisited
        unvisited.push_back(intersections[current].neighbors_pairs[n].first);
      }
      // mark the current intersection as visited
      intersections[current].visited = 1;
    }
    //pop front
    unvisited.erase(unvisited.begin());
    }
    //crawl backwards from dest to start to get the path
    for (int i = intersections[dest].place; i != -1;) {
    path.push_back(i);
    i = intersections[i].previous;
    }
    return path;
}
void sdcCar::initializeGraph() {
    //make the sdcIntersections
    sdcIntersection aa;
    aa.place = 0;
    sdcIntersection ab;
    ab.place = 1;
    sdcIntersection ac;
    ac.place = 2;
    sdcIntersection ad;
    ad.place = 3;
    sdcIntersection ae;
    ae.place = 4;
    sdcIntersection ba;
    ba.place = 5;
    sdcIntersection bb;
    bb.place = 6;
    sdcIntersection bc;
    bc.place = 7;
    sdcIntersection bd;
    bd.place = 8;
    sdcIntersection be;
    be.place = 9;
    sdcIntersection ca;
    ca.place = 10;
    sdcIntersection cb;
    cb.place = 11;
    sdcIntersection cc;
    cc.place = 12;
    sdcIntersection cd;
    cd.place = 13;
    sdcIntersection ce;
    cd.place = 14;
    sdcIntersection da;
    cd.place = 15;
    sdcIntersection db;
    db.place = 16;
    sdcIntersection dc;
    dc.place = 17;
    sdcIntersection dd;
    dd.place = 18;
    sdcIntersection de;
    de.place = 19;
    sdcIntersection ea;
    ea.place = 20;
    sdcIntersection eb;
    eb.place = 21;
    sdcIntersection ec;
    ec.place = 22;
    sdcIntersection ed;
    ed.place = 23;
    sdcIntersection ee;
    ee.place = 24;

    //make the edges
    aa.neighbors_pairs.push_back(std::pair<int, double>(1, 1));
    aa.neighbors_pairs.push_back(std::pair<int, double>(5, 1));
    aa.waypoint = sdcWaypoint(0,std::pair<double,double>(0,0));

    ab.neighbors_pairs.push_back(std::pair<int, double>(0, 1));
    ab.neighbors_pairs.push_back(std::pair<int, double>(2, 1));
    ab.neighbors_pairs.push_back(std::pair<int, double>(6, 1));
    ab.waypoint = sdcWaypoint(0,std::pair<double,double>(0,50));


    ac.neighbors_pairs.push_back(std::pair<int, double>(1, 1));
    ac.neighbors_pairs.push_back(std::pair<int, double>(3, 1));
    ac.neighbors_pairs.push_back(std::pair<int, double>(7, 1));
    ac.waypoint = sdcWaypoint(0,std::pair<double,double>(0,100));

    ad.neighbors_pairs.push_back(std::pair<int, double>(2, 1));
    ad.neighbors_pairs.push_back(std::pair<int, double>(4, 1));
    ad.neighbors_pairs.push_back(std::pair<int, double>(8, 1));
    ad.waypoint = sdcWaypoint(0,std::pair<double,double>(0,150));

    ae.neighbors_pairs.push_back(std::pair<int, double>(3, 1));
    ae.neighbors_pairs.push_back(std::pair<int, double>(9, 1));
    ae.waypoint = sdcWaypoint(0,std::pair<double,double>(0,200));

    ba.neighbors_pairs.push_back(std::pair<int, double>(0, 1));
    ba.neighbors_pairs.push_back(std::pair<int, double>(6, 1));
    ba.neighbors_pairs.push_back(std::pair<int, double>(10, 1));
    ba.waypoint = sdcWaypoint(0,std::pair<double,double>(50,0));

    bb.neighbors_pairs.push_back(std::pair<int, double>(1, 1));
    bb.neighbors_pairs.push_back(std::pair<int, double>(5, 1));
    bb.neighbors_pairs.push_back(std::pair<int, double>(7, 1));
    bb.neighbors_pairs.push_back(std::pair<int, double>(11, 1));
    bb.waypoint = sdcWaypoint(0,std::pair<double,double>(50,50));

    bc.neighbors_pairs.push_back(std::pair<int, double>(2, 1));
    bc.neighbors_pairs.push_back(std::pair<int, double>(6, 1));
    bc.neighbors_pairs.push_back(std::pair<int, double>(8, 1));
    bc.neighbors_pairs.push_back(std::pair<int, double>(12, 1));
    bc.waypoint = sdcWaypoint(0,std::pair<double,double>(50,100));

    bd.neighbors_pairs.push_back(std::pair<int, double>(3, 1));
    bd.neighbors_pairs.push_back(std::pair<int, double>(7, 1));
    bd.neighbors_pairs.push_back(std::pair<int, double>(9, 1));
    bd.neighbors_pairs.push_back(std::pair<int, double>(13, 1));
    bd.waypoint = sdcWaypoint(0,std::pair<double,double>(50,150));

    be.neighbors_pairs.push_back(std::pair<int, double>(4, 1));
    be.neighbors_pairs.push_back(std::pair<int, double>(8, 1));
    be.neighbors_pairs.push_back(std::pair<int, double>(14, 1));
    be.waypoint = sdcWaypoint(0,std::pair<double,double>(50,200));

    ca.neighbors_pairs.push_back(std::pair<int, double>(5, 1));
    ca.neighbors_pairs.push_back(std::pair<int, double>(11, 1));
    ca.neighbors_pairs.push_back(std::pair<int, double>(15, 1));
    ca.waypoint = sdcWaypoint(0,std::pair<double,double>(100,0));

    cb.neighbors_pairs.push_back(std::pair<int, double>(6, 1));
    cb.neighbors_pairs.push_back(std::pair<int, double>(10, 1));
    cb.neighbors_pairs.push_back(std::pair<int, double>(12, 1));
    cb.neighbors_pairs.push_back(std::pair<int, double>(16, 1));
    cb.waypoint = sdcWaypoint(0,std::pair<double,double>(100,50));

    cc.neighbors_pairs.push_back(std::pair<int, double>(7, 1));
    cc.neighbors_pairs.push_back(std::pair<int, double>(11, 1));
    cc.neighbors_pairs.push_back(std::pair<int, double>(13, 1));
    cc.neighbors_pairs.push_back(std::pair<int, double>(17, 1));
    cc.waypoint = sdcWaypoint(0,std::pair<double,double>(100,100));

    cd.neighbors_pairs.push_back(std::pair<int, double>(8, 1));
    cd.neighbors_pairs.push_back(std::pair<int, double>(12, 1));
    cd.neighbors_pairs.push_back(std::pair<int, double>(14, 1));
    cd.neighbors_pairs.push_back(std::pair<int, double>(18, 1));
    cd.waypoint = sdcWaypoint(0,std::pair<double,double>(100,150));


    ce.neighbors_pairs.push_back(std::pair<int, double>(9, 1));
    ce.neighbors_pairs.push_back(std::pair<int, double>(13, 1));
    ce.neighbors_pairs.push_back(std::pair<int, double>(19, 1));
    ce.waypoint = sdcWaypoint(0,std::pair<double,double>(100,200));


    da.neighbors_pairs.push_back(std::pair<int, double>(10, 1));
    da.neighbors_pairs.push_back(std::pair<int, double>(16, 1));
    da.neighbors_pairs.push_back(std::pair<int, double>(20, 1));
    da.waypoint = sdcWaypoint(0,std::pair<double,double>(150,0));


    db.neighbors_pairs.push_back(std::pair<int, double>(11, 1));
    db.neighbors_pairs.push_back(std::pair<int, double>(15, 1));
    db.neighbors_pairs.push_back(std::pair<int, double>(17, 1));
    db.neighbors_pairs.push_back(std::pair<int, double>(21, 1));
    db.waypoint = sdcWaypoint(0,std::pair<double,double>(150,50));

    dc.neighbors_pairs.push_back(std::pair<int, double>(12, 1));
    dc.neighbors_pairs.push_back(std::pair<int, double>(16, 1));
    dc.neighbors_pairs.push_back(std::pair<int, double>(18, 1));
    dc.neighbors_pairs.push_back(std::pair<int, double>(22, 1));
    dc.waypoint = sdcWaypoint(0,std::pair<double,double>(150,100));

    dd.neighbors_pairs.push_back(std::pair<int, double>(13, 1));
    dd.neighbors_pairs.push_back(std::pair<int, double>(17, 1));
    dd.neighbors_pairs.push_back(std::pair<int, double>(19, 1));
    dd.neighbors_pairs.push_back(std::pair<int, double>(23, 1));
    dd.waypoint = sdcWaypoint(0,std::pair<double,double>(150,150));

    de.neighbors_pairs.push_back(std::pair<int, double>(14, 1));
    de.neighbors_pairs.push_back(std::pair<int, double>(18, 1));
    de.neighbors_pairs.push_back(std::pair<int, double>(24, 1));
    de.waypoint = sdcWaypoint(0,std::pair<double,double>(150,200));

    ea.neighbors_pairs.push_back(std::pair<int, double>(15, 1));
    ea.neighbors_pairs.push_back(std::pair<int, double>(21, 1));
    ea.waypoint = sdcWaypoint(0,std::pair<double,double>(200,0));

    eb.neighbors_pairs.push_back(std::pair<int, double>(16, 1));
    eb.neighbors_pairs.push_back(std::pair<int, double>(20, 1));
    eb.neighbors_pairs.push_back(std::pair<int, double>(22, 1));
    eb.waypoint = sdcWaypoint(0,std::pair<double,double>(200,50));

    ec.neighbors_pairs.push_back(std::pair<int, double>(17, 1));
    ec.neighbors_pairs.push_back(std::pair<int, double>(21, 1));
    ec.neighbors_pairs.push_back(std::pair<int, double>(23, 1));
    ec.waypoint = sdcWaypoint(0,std::pair<double,double>(200,100));

    ed.neighbors_pairs.push_back(std::pair<int, double>(18, 1));
    ed.neighbors_pairs.push_back(std::pair<int, double>(22, 1));
    ed.neighbors_pairs.push_back(std::pair<int, double>(24, 1));
    ed.waypoint = sdcWaypoint(0,std::pair<double,double>(200,150));

    ee.neighbors_pairs.push_back(std::pair<int, double>(19, 1));
    ee.neighbors_pairs.push_back(std::pair<int, double>(23, 1));
    ee.waypoint = sdcWaypoint(0,std::pair<double,double>(200,200));

    //place the intersections into intersections
    intersections = {aa, ab, ac, ad, ae, ba, bb, bc, bd, be, ca, cb, cc,
                   cd, ce, da, db, dc, dd, de, ea, eb, ec, ed, ee};
    //make the distance to all intersections infinity
    for (int i = 0; i < intersections.size(); ++i) {
    intersections[i].dist = std::numeric_limits<double>::infinity();
    intersections[i].place = i;
    }
}

int sdcCar::getFirstIntersection(){
    std::pair<double,double> firstIntr;
    int firstIntersection;

    switch(this->currentDir){

        case west:
            firstIntr = {-1000,0};
            for(int i = 0; i < intersections.size();++i){
                if(this->y < intersections[i].waypoint.pos.second+5 && this->y > intersections[i].waypoint.pos.second-5 && intersections[i].waypoint.pos.first < this->x - 10 && intersections[i].waypoint.pos.first > firstIntr.first)
                    firstIntr = intersections[i].waypoint.pos;
            }
            break;

        case east:
            firstIntr = {1000,0};
            for(int i = 0; i < intersections.size();++i){
                if(this->y < intersections[i].waypoint.pos.second+5 && this->y > intersections[i].waypoint.pos.second-5 && intersections[i].waypoint.pos.first > this->x + 10 && intersections[i].waypoint.pos.first < firstIntr.first){
                    firstIntr = intersections[i].waypoint.pos;
                }
            }
            break;

        case north:
            firstIntr = {0,1000};
            for(int i = 0; i < intersections.size();++i){
                if(this->x < intersections[i].waypoint.pos.first+5 && this->x > intersections[i].waypoint.pos.first-5 && intersections[i].waypoint.pos.second > this->y + 10 && intersections[i].waypoint.pos.second < firstIntr.second)
                    firstIntr = intersections[i].waypoint.pos;
            }
            break;

        case south:
            firstIntr = {0,-1000};
            for(int i = 0; i < intersections.size();++i){
                if(this->x < intersections[i].waypoint.pos.first+5 && this->x > intersections[i].waypoint.pos.first-5 && intersections[i].waypoint.pos.second < this->y - 10 && intersections[i].waypoint.pos.second > firstIntr.second)
                    firstIntr = intersections[i].waypoint.pos;
            }
            break;
    }
    for(int i = 0; i < intersections.size();i++){
        if(firstIntr.first == intersections[i].waypoint.pos.first && firstIntr.second == intersections[i].waypoint.pos.second){
            firstIntersection = i;
            break;
        }
    }
    return firstIntersection;
}

void sdcCar::insertWaypointTypes(std::vector<int> path, Direction startDir) {
  Direction curDir = startDir;
  Direction nextDir;
  int current;
  int next;
  // get the direction the car heads in from the current intersection to
  // the next one
  for (int i = path.size() - 1; i > 0; i--) {
    current = path[i];
    next = path[i - 1];
    if (next - current == size) {
      nextDir = east;
    } else if (current - next == size) {
      nextDir = west;
    } else if (next - current == 1) {
      nextDir = north;
    } else if (current - next == 1) {
      nextDir = south;
    }
    switch (curDir) {
      case north:
        switch (nextDir) {
          case north:
            intersections[current].waypoint.waypointType = WaypointType_DriveStraight;
            break;
          case east:
            intersections[current].waypoint.waypointType = WaypointType_TurnRight;
            break;
          case west:
            intersections[current].waypoint.waypointType = WaypointType_TurnLeft;
          case south:
            break;
        }
        break;
      case south:
        switch (nextDir) {
          case south:
            intersections[current].waypoint.waypointType = WaypointType_DriveStraight;
            break;
          case east:
            intersections[current].waypoint.waypointType = WaypointType_TurnLeft;
            break;
          case west:
            intersections[current].waypoint.waypointType = WaypointType_TurnRight;
          case north:
            break;
        }
        break;
      case east:
        switch (nextDir) {
          case north:
            intersections[current].waypoint.waypointType = WaypointType_TurnLeft;
            break;
          case south:
            intersections[current].waypoint.waypointType = WaypointType_TurnRight;
            break;
          case east:
            intersections[current].waypoint.waypointType = WaypointType_DriveStraight;
          case west:
            break;
        }
        break;
      case west:
        switch (nextDir) {
          case north:
            intersections[current].waypoint.waypointType = WaypointType_TurnRight;
            break;
          case south:
            intersections[current].waypoint.waypointType = WaypointType_TurnLeft;
            break;
          case west:
            intersections[current].waypoint.waypointType = WaypointType_DriveStraight;
          case east:
            break;
        }
        break;
    }
    curDir = nextDir;
  }
  intersections[path[0]].waypoint.waypointType = WaypointType_Stop;
}

void sdcCar::removeStartingEdge(int start){
    Direction dir = east;
    switch (dir) {
      case north:
        for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
          if (intersections[start].neighbors_pairs[n].first == start - 1)
            intersections[start].neighbors_pairs[n].second =
                std::numeric_limits<double>::infinity();
        }
        break;
      case south:
        for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
          if (intersections[start].neighbors_pairs[n].first == start + 1)
            intersections[start].neighbors_pairs[n].second =
                std::numeric_limits<double>::infinity();
        }
        break;
      case east:
        for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
          if (intersections[start].neighbors_pairs[n].first == start - size)
            intersections[start].neighbors_pairs[n].second =
                std::numeric_limits<double>::infinity();
        }
        break;
      case west:
        for (int n = 0; n < intersections[start].neighbors_pairs.size(); ++n) {
          if (intersections[start].neighbors_pairs[n].first == start + size)
            intersections[start].neighbors_pairs[n].second =
                std::numeric_limits<double>::infinity();
        }
        break;
    }
}


////////////////////
// HELPER METHODS //
////////////////////

/*
 * Updates the list of objects in front of the car with the given list of new objects
 */
void sdcCar::UpdateFrontObjects(std::vector<sdcVisibleObject> newObjects){
    if(this->frontObjects.size() == 0){
        // The car wasn't tracking any objects, so just set the list equal to the new list
        this->frontObjects = newObjects;
        return;
    }

    std::vector<bool> isOldObjectMissing;
    std::vector<bool> isBrandNewObject;
    for(int i = 0; i < newObjects.size(); i++){
        isBrandNewObject.push_back(true);
    }

    // Compare each old object to the new objects, and determine
    // which of them are getting updated, which are missing, as well
    // as if any of the passed in objects are brand new
    for (int i = 0; i < this->frontObjects.size(); i++) {
        sdcVisibleObject oldObj = this->frontObjects[i];
        isOldObjectMissing.push_back(true);

        for (int j = 0; j < newObjects.size(); j++) {
            // Only match each new object to one old object
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

    // Delete objects that are missing
    for(int i = isOldObjectMissing.size() - 1; i >= 0; i--){
        if(isOldObjectMissing[i]){
            this->frontObjects.erase(this->frontObjects.begin() + i);
        }
    }

    // Add brand new objects
    for(int i = 0; i < newObjects.size(); i++){
        if(isBrandNewObject[i]){
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
    sdcAngle carAngle = this->GetOrientation();
    return (carAngle - velAngle).IsFrontFacing();
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
 * Gets the current direction the car is travelling in NSEW
 */
void sdcCar::GetNSEW(){
    if((this->yaw - WEST).WithinMargin(PI/4)){
        this->currentDir = west;
    } else if((this->yaw - SOUTH).WithinMargin(PI/4)){
        this->currentDir = south;
    } else if((this->yaw - EAST).WithinMargin(PI/4)){
        this->currentDir = east;
    } else {
        this->currentDir = north;
    }
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
    if(this->frontObjects.size() == 0) return false;

    for (int i = 0; i < this->frontObjects.size(); i++) {
        if(this->IsObjectDirectlyAhead(this->frontObjects[i])){
            return true;
        }
    }
    return false;
}

/*
 * Returns true if the given object is directly ahead of us, else false
 */
bool sdcCar::IsObjectDirectlyAhead(sdcVisibleObject obj){
    double leftDist = obj.left.GetLateralDist();
    double rightDist = obj.right.GetLateralDist();
    if(leftDist < 0 && rightDist > 0) return true;
    return fmin(fabs(leftDist), fabs(rightDist)) < FRONT_OBJECT_COLLISION_WIDTH / 2.;
}

/*
 * Returns true if there is an object on a potential collision course with our car
 */
bool sdcCar::ObjectOnCollisionCourse(){
    if(this->frontObjects.size() == 0) return false;

    for (int i = 0; i < this->frontObjects.size(); i++) {
        if(this->IsObjectOnCollisionCourse(this->frontObjects[i])){
            return true;
        }
    }
    return false;
}

/*
 * Returns true if the given object is on a potential collision course with our car
 */
bool sdcCar::IsObjectOnCollisionCourse(sdcVisibleObject obj){
    bool isTooFast = this->IsObjectTooFast(obj);
    bool isTooFurious = this->IsObjectTooFurious(obj);
    return isTooFast || isTooFurious;
}

/*
 * Returns true if the given object is projected to run into the car within a short time period from now
 */
bool sdcCar::IsObjectTooFast(sdcVisibleObject obj){
    math::Vector2d centerpoint = obj.GetCenterPoint();
    bool inLineToCollide = (fabs(obj.lineIntercept) < 1.5 || (fabs(centerpoint.x) < 1.5 && fabs(obj.GetEstimatedXSpeed()) < fabs(0.1 * obj.GetEstimatedYSpeed())));
    bool willHitSoon = obj.dist / obj.GetEstimatedSpeed() < 20;
    return inLineToCollide && willHitSoon;
}

/*
 * Returns true if the given object is very close to the car
 */
bool sdcCar::IsObjectTooFurious(sdcVisibleObject obj){
    math::Vector2d centerpoint = obj.GetCenterPoint();
    return (fabs(centerpoint.x) < FRONT_OBJECT_COLLISION_WIDTH / 2. && fabs(centerpoint.y) < 1.5);
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
    this->stopping = (this->targetSpeed == 0);
    this->SetAccelRate();
    this->SetBrakeRate();
}

/*
 * Sets the amount by which the car turns. A larger number makes the car turn
 * harder.
 */
void sdcCar::SetTurningLimit(double limit){
    this->turningLimit = limit;
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

    // During init, sensors aren't available so pull position and rotation information
    // straight from the car
    math::Pose pose = this->chassis->GetWorldPose();
    this->yaw = sdcAngle(pose.rot.GetYaw());
    this->x = pose.pos.x;
    this->y = pose.pos.y;
    GenerateWaypoints();
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

    // Check if the front lidars have been updated, and if they have update
    // the car's list
    if(this->frontLidarLastUpdate != sdcSensorData::GetLidarLastUpdate(FRONT)){
        std::vector<sdcVisibleObject> v = sdcSensorData::GetObjectsInFront();
        this->UpdateFrontObjects(v);
        this->frontLidarLastUpdate = sdcSensorData::GetLidarLastUpdate(FRONT);
    }

    // Call our Drive function, which is the brain for the car
    this->Drive();


    ////////////////////////////
    // GAZEBO PHYSICS METHODS //
    ////////////////////////////

    // Compute the angle of the front wheels.
    double wheelAngle = this->steeringAmount / this->steeringRatio;

    // Compute the rotational velocity of the wheels
    double jointVel = (this->gas-this->brake * this->maxSpeed) / this->wheelRadius;

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

    // Physics variables
    this->aeroLoad = 0.1;
    this->swayForce = 10;

    this->maxSpeed = 10;
    this->frontPower = 50;
    this->rearPower = 50;
    this->wheelRadius = 0.3;
    this->steeringRatio = 1.0;
    this->tireAngleRange = 1.0;

    // Movement parameters
    this->gas = 0.0;
    this->brake = 0.0;
    this->accelRate = 1.0;
    this->brakeRate = 1.0;

    // Limits on the car's speed
    this->maxCarSpeed = 10;
    this->maxCarReverseSpeed = -10;

    // Initialize state enums
    this->DEFAULT_STATE = waypoint;
    this->currentState = DEFAULT_STATE;

    this->currentPerpendicularState = backPark;
    this->currentParallelState = rightBack;
    this->currentAvoidanceState = notAvoiding;

    // Set starting speed parameters
    this->targetSpeed = 6;

    // Set starting turning parameters
    this->steeringAmount = 0.0;
    this->targetSteeringAmount = 0.0;
    this->targetDirection = sdcAngle(0);
    this->turningLimit = 20.0;

    // Booleans for the car's actions
    this->turning = false;
    this->reversing = false;
    this->stopping = false;

    // Variables for parking
    this->targetParkingAngle = sdcAngle(0.0);
    this->parkingAngleSet = false;
    this->isFixingParking = false;
    this->parkingSpotSet = false;

    // Variables for waypoint driving
    this->waypointProgress = 0;

    // Variables for intersections
    this->stoppedAtSign = false;
    this->ignoreStopSignsCounter = 0;
    this->atIntersection = 0;

    // Variables for following
    this->isTrackingObject = false;
    this->stationaryCount = 0;

    // Variables for avoidance
    this->trackingNavWaypoint = false;
}
