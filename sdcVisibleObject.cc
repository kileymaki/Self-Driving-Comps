#include "sdcVisibleObject.hh"

using namespace gazebo;

const double sdcVisibleObject::UNCERTAINTY_RATIO = 5.;

sdcVisibleObject::sdcVisibleObject(sdcLidarRay left, sdcLidarRay right, double dist){
    this->left = left;
    this->right = right;
    this->dist = dist;

    this->centerpoint = this->GetCenterPoint();

    this->estimatedSpeed = 0;
    this->estimatedDirection = (left.angle + right.angle) / 2.;
    this->confidence = 0.01;

    this->tracking = false;
}

bool sdcVisibleObject::IsSameObject(sdcVisibleObject other){
    math::Vector2d estPos = this->EstimateUpdate();
    double uncertainty = sqrt(pow(estPos.x - other.centerpoint.x, 2) + pow(estPos.y - other.centerpoint.y, 2));

    return uncertainty * confidence < UNCERTAINTY_RATIO;
}

math::Vector2d sdcVisibleObject::EstimateUpdate(){
    double newX = this->centerpoint.x + sin(this->estimatedDirection.angle) * this->estimatedSpeed;
    double newY = this->centerpoint.y + cos(this->estimatedDirection.angle) * this->estimatedSpeed;
    return math::Vector2d(newX, newY);
}

void sdcVisibleObject::Update(sdcLidarRay newLeft, sdcLidarRay newRight, double newDist){
    this->confidence = fmin(1.0, this->confidence + 0.01);

    math::Vector2d newCenterpoint = this->GetCenterPoint(newLeft, newRight);

    double newEstimatedSpeed = sqrt(pow(this->centerpoint.x - newCenterpoint.x, 2) + pow(this->centerpoint.y - newCenterpoint.y, 2));

    double alpha = fmax((newDist * .005), (.1 - newDist * -.005));
    newEstimatedSpeed = fmin(6, (alpha * newEstimatedSpeed) + ((1 - alpha) * this->estimatedSpeed));

    this->estimatedSpeed = newEstimatedSpeed;
    this->estimatedDirection = sdcAngle(atan2(this->centerpoint.x - newCenterpoint.x, this->centerpoint.y - newCenterpoint.y));

    this->left = newLeft;
    this->right = newRight;
    this->dist = newDist;
}

void sdcVisibleObject::SetTracking(bool isTracking){
    this->tracking = isTracking;
}

math::Vector2d sdcVisibleObject::GetCenterPoint(){
    double x = (this->left.GetLateralDist() + this->right.GetLateralDist()) / 2.;
    double y = (this->left.GetLongitudinalDist() + this->right.GetLongitudinalDist()) / 2.;
    return math::Vector2d(x, y);
}

math::Vector2d sdcVisibleObject::GetCenterPoint(sdcLidarRay left, sdcLidarRay right){
    double x = (left.GetLateralDist() + right.GetLateralDist()) / 2.;
    double y = (left.GetLongitudinalDist() + right.GetLongitudinalDist()) / 2.;
    return math::Vector2d(x, y);
}
