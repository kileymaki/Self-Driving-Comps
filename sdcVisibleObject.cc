#include "sdcVisibleObject.hh"

using namespace gazebo;

// How far off our estimates can be before we assume we're seeing a differnt object
const double sdcVisibleObject::UNCERTAINTY_RATIO = 5.;

/*
 * Visible objects are obstructions detected by Lidar rays. They have estimated
 * movement parameters in order to help us predict their motion and track them
 * across multiple sensor readings.
 */
sdcVisibleObject::sdcVisibleObject(sdcLidarRay right, sdcLidarRay left, double dist){
    this->left = left;
    this->right = right;
    this->dist = dist;

    this->centerpoint = this->GetCenterPoint();

    this->estimatedSpeed = 0;
    this->estimatedDirection = (left.angle + right.angle) / 2.;
    this->confidence = 0.01;

    this->tracking = false;
}

/*
 * Returns true if the given object is a possible new position of this object
 */
bool sdcVisibleObject::IsSameObject(sdcVisibleObject other){
    math::Vector2d estPos = this->EstimateUpdate();
    double uncertainty = sqrt(pow(estPos.x - other.centerpoint.x, 2) + pow(estPos.y - other.centerpoint.y, 2));

    // std::cout << "CUR \t" << this->centerpoint.x << "\t" << this->centerpoint.y << std::endl;
    // std::cout << "EST \t" << estPos.x << "\t" << estPos.y << std::endl;
    // std::cout << "OTH \t" << other.centerpoint.x << "\t" << other.centerpoint.y << std::endl;
    // std::cout << "Uncertainty\t" << uncertainty << "\t" << confidence << std::endl;
    // std::cout << (uncertainty * confidence < UNCERTAINTY_RATIO) << std::endl;

    return uncertainty * confidence < UNCERTAINTY_RATIO;
}

/*
 * Calculates an estimated new position this object would be at with it's given estimated
 * speed and direction
 */
math::Vector2d sdcVisibleObject::EstimateUpdate(){
    double newX = this->centerpoint.x + sin(this->estimatedDirection.angle) * this->estimatedSpeed;
    double newY = this->centerpoint.y + cos(this->estimatedDirection.angle) * this->estimatedSpeed;
    return math::Vector2d(newX, newY);
}

/*
 * Given new readings for the location of this object, update the stored parameters
 * to learn it's projected speed and direction
 */
void sdcVisibleObject::Update(sdcLidarRay newLeft, sdcLidarRay newRight, double newDist){
    this->confidence = fmin(1.0, this->confidence + 0.01);

    math::Vector2d newCenterpoint = this->GetCenterPoint(newLeft, newRight);

    double newEstimatedSpeed = sqrt(pow(this->centerpoint.x - newCenterpoint.x, 2) + pow(this->centerpoint.y - newCenterpoint.y, 2));

    double alpha = fmax((newDist * .005), (.1 - newDist * -.005));
    newEstimatedSpeed = fmin(6, (alpha * newEstimatedSpeed) + ((1 - alpha) * this->estimatedSpeed));

    this->estimatedSpeed = newEstimatedSpeed;
    this->estimatedDirection = sdcAngle(atan2(this->centerpoint.x - newCenterpoint.x, this->centerpoint.y - newCenterpoint.y));

    this->centerpoint = newCenterpoint;

    this->left = newLeft;
    this->right = newRight;
    this->dist = newDist;
}

void sdcVisibleObject::Update(sdcVisibleObject newObject){
    this->Update(newObject.left, newObject.right, newObject.dist);
}

/*
 * Set whether we are tracking this object
 */
void sdcVisibleObject::SetTracking(bool isTracking){
    this->tracking = isTracking;
}

/*
 * Gets the centerpoint of this object based on the left and right rays
 */
math::Vector2d sdcVisibleObject::GetCenterPoint(){
    return this->GetCenterPoint(this->left, this->right);
}

/*
 * Gets the centerpoint of the two given rays in (x,y) coordinates
 */
math::Vector2d sdcVisibleObject::GetCenterPoint(sdcLidarRay left, sdcLidarRay right){
    // std::cout << "LEFT\t" << left.angle << "\t" << left.dist << std::endl;
    // std::cout << "RIGHT\t" << right.angle << "\t" << right.dist << std::endl;
    double x = (left.GetLateralDist() + right.GetLateralDist()) / 2.;
    double y = (left.GetLongitudinalDist() + right.GetLongitudinalDist()) / 2.;
    // std::cout << x << "\t" << y << std::endl;
    return math::Vector2d(x, y);
}
