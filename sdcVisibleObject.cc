/*
 * This class provides a wrapper for objects we see, created by two lidar rays and 
 * a distance. This class provides methods that return information about these
 * objects, including estimates about future position based upon past data.
 */

#include "sdcVisibleObject.hh"

using namespace gazebo;

// How far off our estimates can be before we assume we're seeing a differnt object
const double sdcVisibleObject::UNCERTAINTY_RATIO = 0.3;
const double PI = 3.14159265359;

/*
 * Default constructor that leaves all parameters at default values
 */
sdcVisibleObject::sdcVisibleObject(){}

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
    this->prevPoints.push_back(this->centerpoint);

    this->estimatedXSpeed = 0;
    this->estimatedYSpeed = 0;
    this->confidence = 0.01;

    this->tracking = false;
    this->brandSpankinNew = true;
}

/*
 * Returns true if the given object is a possible new position of this object
 */
bool sdcVisibleObject::IsSameObject(sdcVisibleObject other){
    math::Vector2d estPos = this->EstimateUpdate();
    double uncertainty = sqrt(pow(estPos.x - other.centerpoint.x, 2) + pow(estPos.y - other.centerpoint.y, 2));

    return uncertainty * confidence < UNCERTAINTY_RATIO;
}

/*
 * Returns the estimated total speed of this object relative to the car's speed
 */
double sdcVisibleObject::GetEstimatedSpeed(){
    return sqrt(pow(this->estimatedXSpeed, 2) + pow(this->estimatedYSpeed, 2));
}

/*
 * Returns the current estimated speed parallel to the car's orientation (towards or away
 * from the car if the object is ahead of the car), and relative to the car's speed
 *
 * Negative speeds are moving towards the car, positive away
 */
double sdcVisibleObject::GetEstimatedYSpeed(){
    return this->estimatedYSpeed;
}

/*
 * Returns the current estimated speed perpendicular to the car's orientation (across the
 * car's field of view), and realtive to the car's speed
 *
 * Negative speeds are moving left, positive right
 */
double sdcVisibleObject::GetEstimatedXSpeed(){
    return this->estimatedXSpeed;
}

/*
 * Calculates an estimated new position this object would be at with it's given estimated
 * speed and direction
 */
math::Vector2d sdcVisibleObject::EstimateUpdate(){
    double newX = this->centerpoint.x + this->estimatedXSpeed;
    double newY = this->centerpoint.y + this->estimatedYSpeed;
    return math::Vector2d(newX, newY);
}

/*
 * Method to calculate the projected position some numSteps into the future.
 * 
 * Currently unused.
 */
math::Vector2d sdcVisibleObject::GetProjectedPosition(int numSteps){
    double newX = this->centerpoint.x + this->estimatedXSpeed * numSteps;
    double newY = this->centerpoint.y + this->estimatedYSpeed * numSteps;
    return math::Vector2d(newX, newY);
}

/*
 * Given new readings for the location of this object, update the stored parameters
 * to learn it's projected speed and direction
 */
void sdcVisibleObject::Update(sdcLidarRay newLeft, sdcLidarRay newRight, double newDist){
    this->confidence = fmin(1.0, this->confidence + 0.01);

    // Get the centerpoint of the new information
    math::Vector2d newCenterpoint = this->GetCenterPoint(newLeft, newRight, newDist);

    // Calculate the speed moving from the current point to the new point
    double newEstimatedXSpeed = (newCenterpoint.x - this->centerpoint.x);
    double newEstimatedYSpeed = (newCenterpoint.y - this->centerpoint.y);

    // If this object has already been updated at least once, try to learn the speed
    // over time
    if(!this->brandSpankinNew){
        double alpha = fmax((newDist * .005), (.1 - newDist * .005));
        newEstimatedXSpeed = (alpha * newEstimatedXSpeed) + ((1 - alpha) * this->estimatedXSpeed);
        newEstimatedYSpeed = (alpha * newEstimatedYSpeed) + ((1 - alpha) * this->estimatedYSpeed);
    }

    // Update the estimates for this object's speed
    this->estimatedXSpeed = newEstimatedXSpeed;
    this->estimatedYSpeed = newEstimatedYSpeed;

    // Fit a line to the points this object has been at, and store that line's information
    if(this->prevPoints.size() > 0){
        math::Vector2d newLineCoefficients = this->FitLineToPoints(this->prevPoints, newCenterpoint);
        this->lineSlope = newLineCoefficients.x;
        this->lineIntercept = newLineCoefficients.y;
    }

    // Maintain prevPoints to never be larger than 16 to help ensure accurate information
    if(this->prevPoints.size() > 15){
        this->prevPoints.erase(this->prevPoints.begin());
    }
    this->prevPoints.push_back(newCenterpoint);

    // Update the object's information
    this->centerpoint = newCenterpoint;

    this->left = newLeft;
    this->right = newRight;
    this->dist = newDist;

    // This object has now been updated, so set this flag accordingly
    this->brandSpankinNew = false;
}

/*
 * This method takes in a vector of multiple points and attemps to fit a line to these points.
 * This allows us to project its path and determine whether or not there is a chance for
 * the object to hit us. Method returns the predicted slope and Y-intercept based upon the 
 * vector of points.
 */
math::Vector2d sdcVisibleObject::FitLineToPoints(std::vector<math::Vector2d> points, math::Vector2d newPoint){
    int numPoints = points.size();

    // Calculate several necessary sums over all points
    double sumX=0, sumY=0, sumXY=0, sumX2=0;
    for(int i=0; i<numPoints; i++) {
        sumX += points[i].x;
        sumY += points[i].y;
        sumXY += points[i].x * points[i].y;
        sumX2 += points[i].x * points[i].x;
    }

    sumX += newPoint.x;
    sumY += newPoint.y;
    sumXY += newPoint.x * newPoint.y;
    sumX2 += newPoint.x * newPoint.x;

    // Get the averages for x and y
    double xMean = sumX / (numPoints + 1);
    double yMean = sumY / (numPoints + 1);

    // Calculate the denominator for the slope calculation
    double denom = sumX2 - sumX * xMean;

    // Calculate the slope and intercept of the line
    double slope = (sumXY - sumX * yMean) / denom;
    double yInt = yMean - slope * xMean;

    return math::Vector2d(slope, yInt);
}

/*
 * Update this object with the given object's parameters
 */
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
 * Get whetehr this object is being tracked
 */
bool sdcVisibleObject::IsTracking(){
    return this->tracking;
}

/*
 * Gets the centerpoint of this object based on the left and right rays
 */
math::Vector2d sdcVisibleObject::GetCenterPoint(){
    return this->GetCenterPoint(this->left, this->right, this->dist);
}

/*
 * Gets the centerpoint of the two given rays in (x,y) coordinates
 */
math::Vector2d sdcVisibleObject::GetCenterPoint(sdcLidarRay left, sdcLidarRay right, double dist){
    sdcLidarRay midRay = sdcLidarRay(left.angle.GetMidAngle(right.angle), dist);
    double x = midRay.GetLateralDist();
    double y = midRay.GetLongitudinalDist();

    // double x = (left.GetLateralDist() + right.GetLateralDist()) / 2.;
    // double y = (left.GetLongitudinalDist() + right.GetLongitudinalDist()) / 2.;
    return math::Vector2d(x, y);
}
