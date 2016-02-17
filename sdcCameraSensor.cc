#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <numeric>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>


#include "sdcCameraSensor.hh"

using namespace gazebo;
using namespace cv;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(sdcCameraSensor)

// Pointer to the update event connection
event::ConnectionPtr updateConnection;
sensors::MultiCameraSensorPtr parentSensor;

// Cascade Classifier information using CPU
CascadeClassifier cpu_stop_sign;
String cascade_file_path = "/Users/selfcar/Desktop/Self-Driving-Comps/OpenCV/haarcascade_stop.xml";

void sdcCameraSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    // Get the parent sensor.
    this->parentSensor =
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "Couldn't find a camera\n";
        return;
    }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&sdcCameraSensor::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);
    //std::cout << this->parentSensor->GetNoise() << std::endl;
}

// Called by the world update start event
void sdcCameraSensor::OnUpdate() {
  // pull raw data from camera sensor object
  const unsigned char* img = this->parentSensor->GetImageData(0);
  Mat image = Mat(this->parentSensor->GetImageHeight(0), this->parentSensor->GetImageWidth(0), CV_8UC3, const_cast<unsigned char*>(img));

  //Select Region of Interest (ROI) for lane detection - currently this is the bottom half of the image.
  //set area for ROI as a rectangle
  //Rect ROI = image(cv::Rect(0, (4*image.rows)/5, image.cols, image.rows/5));
  Rect ROI = cv::Rect(0, image.rows/2, image.cols, image.rows/2);
  Mat imageROI = image(ROI);
  //rectangle(image,ROI,Scalar(0,255,0),2);

  // Canny algorithm for edge dectection
  Mat contours;
  Canny(imageROI,contours,50,150);
  //Mat contoursInv;
  //threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
  // Hough Transform detects lines within the edge map, stores result in lines.
  float PI = 3.14159;
  std::vector<Vec2f> lines;
  HoughLines(contours,lines,1,PI/180, 100);
  //HoughLinesP(contoursInv,lines,1,PI/180,80, 30, 10 );

  //print out line angles
  // for (std::vector<Vec2f>::const_iterator i = lines.begin(); i != lines.end(); ++i)
  // std::cout << *i << ' ' << std::endl;
  // std::cout << "=====================================\n";


  std::vector<Vec2f>::const_iterator it = lines.begin();

  // white line grid overlay for reference points on displayed image
  //line(imageROI, Point((imageROI.cols/2)-10,0), Point((imageROI.cols/2)+10,0), Scalar(255,255,255), 2);
  //line(imageROI, Point(imageROI.cols/2,10), Point(imageROI.cols/2,-10), Scalar(255,255,255), 2);

  Vec2f left_lane_marker = Vec2f(0.0, PI);
  Vec2f right_lane_marker = Vec2f(0.0, 0.0);
  //std::cout << left_lane_marker[0];

  while (it!=lines.end()) {
      float rho= (*it)[0];   // first element is distance rho
      float theta= (*it)[1]; // second element is angle theta

      if ( 0 < theta < PI/2 && theta < left_lane_marker[1]) {
        left_lane_marker = Vec2f(rho,theta);
      }

      if (PI/2 < theta < PI && theta > right_lane_marker[1]) {
        right_lane_marker = Vec2f(rho,theta);
      }

      // Point pt1(rho/cos(theta),0);
      // Point pt2((rho-imageROI.rows*sin(theta))/cos(theta),imageROI.rows);
      // line(imageROI, pt1, pt2, Scalar(0,0,255), 3);

      ++it;
  }
  // ATTN: need to have ROI.rows in numerator because of trig stuff. It isnt an oversight!
  // Changed things so the lines are drawn on the image, not the region of interest. hopefully 
  // this will make some of the issues easier to debug

  //draw left lane marker
  Point leftp1 = Point(left_lane_marker[0]/cos(left_lane_marker[1]),240);
  Point leftp2 = Point((left_lane_marker[0] - (imageROI.rows) * sin(left_lane_marker[1])) / cos(left_lane_marker[1]), (image.rows));
  line(image, leftp1, leftp2, Scalar(255), 3);

  //draw right lane marker
  Point rightp1 = Point(right_lane_marker[0]/cos(right_lane_marker[1]),240);
  Point rightp2 = Point((right_lane_marker[0] - (imageROI.rows) * sin(right_lane_marker[1])) / cos(right_lane_marker[1]), (image.rows));
  line(image, rightp1, rightp2, Scalar(255), 3);

  std::cout << leftp1 << "\t" << leftp2 << "\t" << rightp1 << "\t" << rightp2 << "\t" << std::endl;

  //BEGIN HAAR CASCADE OBJECT DETECTION
/*
  if(!cpu_stop_sign.load(cascade_file_path)){ printf("--(!)Error loading face cascade\n");};
  std::vector<Rect> stopSigns_left, stopSigns;
  cpu_stop_sign.detectMultiScale( image_left, stopSigns_left, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
  cpu_stop_sign.detectMultiScale( image, stopSigns, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
  sdcSensorData::stopSignInLeftCamera = false;
  for( int i = 0; i < stopSigns_left.size(); i++ )
  {
     cv::rectangle(image_left, stopSigns_left[i], Scalar(0,0,255),3,LINE_8,0);
     std::cout << "stop sign found in left image" <<std::endl;
     sdcSensorData::stopSignInLeftCamera = true;
  }

  sdcSensorData::stopSignInRightCamera = false;
  for( int i = 0; i < stopSigns.size(); i++ )
  {
     cv::rectangle(image, stopSigns[i], Scalar(0,0,255),3,LINE_8,0);
     std::cout << "stop sign found in right image" <<std::endl;
     sdcSensorData::stopSignInRightCamera = true;
  }
*/

// BEGIN LCF LANE DETECTION
double leftNearLaneSlope, rightNearLaneSlope, leftLaneIntercept, rightLaneIntercept;
double a, b, c, d, e, u, v, n, k, lane_midpoint, eps = 25.0;
float FOCAL_LENGTH = 554.382; //lambda in Park et. al.
float Tz = 0.85; // in meters
// float xf = leftp1.x;
// A = eps * i / lambda^2 * Tz
// A = 25 * i / 471.2247

//TBH THIS STUFF SHOULD NOT BE SET EVERY UPDATE NEEDS TO BE MOVED ~~~~~~~
std::vector<double> vec_of_i_vals(79);// = {-48., -39., -38., -37., -36., -34., -32., -31., -30., -29., -28., -27., -26., -25.}
std::iota(std::begin(vec_of_i_vals), std::end(vec_of_i_vals), -39.);
vec_of_i_vals.push_back(48.);
vec_of_i_vals.push_back(-48.);
// for (std::vector<double>::const_iterator i = vec_of_i_vals.begin(); i != vec_of_i_vals.end(); ++i)
// std::cout << *i << ' ' << std::endl;
// std::cout << "=====================================\n";


// Using the two lane markers
Point vanishingPoint;

// using the left lane
if (leftp1.x - leftp2.x != 0) {
    leftNearLaneSlope = (1.*leftp2.y - leftp1.y)/(leftp2.x - leftp1.x);
}

// using the right lane
if (rightp2.x - rightp1.x != 0) {
    rightNearLaneSlope = (1.*rightp2.y - rightp1.y)/(rightp2.x - rightp1.x);
}
std::cout << "left slope: " << leftNearLaneSlope << "\t|\t" << "right slope: "<< rightNearLaneSlope << std::endl;

// Calculate y-intercepts of the lanes
//b = y - mx
leftLaneIntercept = leftp1.y - leftNearLaneSlope*leftp1.x; //((0-leftp1.y)/leftNearLaneSlope)+leftp1.x;
rightLaneIntercept = rightp1.y - rightNearLaneSlope*rightp1.x;// ((0-rightp1.y)/rightNearLaneSlope)+rightp1.x;
std::cout << "left intercept: " << leftLaneIntercept << "\t|\t" << "right intecept: "<< rightLaneIntercept << std::endl;

// SOLVE VANISHING POINT (u,v), which is intersection of two lines
//v is the height component of the vanishing point, described as vp = (u,v)
//v = image.rows/2;
//au + c = bu + d
//u = (d - c) / (a - b)
u = abs((leftLaneIntercept - rightLaneIntercept) / (leftNearLaneSlope - rightNearLaneSlope));
v = abs((leftNearLaneSlope * u)) + rightLaneIntercept;
Point vp = Point(u,v);
//std::cout << "Vanishing Point: " << vp.x << "|" << vp.y << std::endl;
circle(image,vp, 4, Scalar(0,255,0), 3);

lane_midpoint = (leftp2.x + rightp2.x)/2;

// n is our position relative to the center of the lane directly in front of us.
n = (image.cols/2) - lane_midpoint;

// Naive lane detection is better than no lane detection
sdcSensorData::UpdateCameraData(n);

line(image, Point(lane_midpoint, 480), Point(lane_midpoint, 0), Scalar(0, 255, 0), 1);
// std::cout << "n: " << n << "\tlane midpoint: " << lane_midpoint << std::endl;
a = leftNearLaneSlope/2;
b = (v+n)/2;
c = pow(leftNearLaneSlope,2)/4;
d = a * (n-v);

//DRAW LEFT NEAR FIELD AND FAR FIELD ASYMPTOTES
Point nfa_p1, nfa_p2, ffa_p1, ffa_p2;
//Yf = (a+ sqrt(c)x_f + (b + c(d/(2*sqrt(c)))))
nfa_p1.x = 0.;
nfa_p2.x = 320.;
nfa_p1.y = (a + sqrt(c))*(nfa_p1.x) + (b + (d/(2*sqrt(c))));
nfa_p2.y = (a + sqrt(c))*(nfa_p2.x) + (b + (d/(2*sqrt(c))));

ffa_p1.x = 0.;
ffa_p2.x = 320.;
ffa_p1.y = (a - sqrt(c))*(ffa_p1.x) + (b - (d/(2*sqrt(c)))) +480;
ffa_p2.y = (a - sqrt(c))*(ffa_p2.x) + (b - (d/(2*sqrt(c)))) +480;

line(image, nfa_p1, nfa_p2, Scalar(255,255,0), 1, CV_AA);
line(image, ffa_p1, ffa_p2, Scalar(255,255,0), 1, CV_AA);
std::cout << "ASYMPTOTES: " << nfa_p1 << "\t" << nfa_p2 << "\t" << ffa_p1 << "\t" << ffa_p2 << "\t" << std::endl;
std::cout << "VARIABLES: " << a << "\t" << b << "\t" << c << "\t" << d << "\t" << std::endl;

//e = (b-v)^2 + ka
//e =  pow((b-v),2) + (k*a);
//Mat curves = image.clone();
for (std::vector<double>::const_iterator i = vec_of_i_vals.begin(); i != vec_of_i_vals.end(); i++) {
  // double A_i = 25*i/471.2247;
  e = pow((b-v),2) + (eps * a * *i);
  //std::cout << e << std::endl;

  std::vector<Point> curve_points_top, curve_points_bot;
  for (float x = 0.; x < 320. ; x++ ) {
    float y_top = (a * x) + b + sqrt( c*pow(x,2) + (d * x) + e);
    float y_bot = (a * x) + b - sqrt( c*pow(x,2) + (d * x) + e);
    Point curve_point_top = Point(x,y_top);
    curve_points_top.push_back(curve_point_top);
    Point curve_point_bot = Point(x,y_bot);
    curve_points_bot.push_back(curve_point_bot);

  }
  for (int i = 0; i < curve_points_bot.size() - 1; i++){
    line(image, curve_points_top[i], curve_points_top[i + 1], Scalar(255,0,255), 1, CV_AA);
    line(image, curve_points_bot[i], curve_points_bot[i + 1], Scalar(255,0,255), 1, CV_AA);
  }
}
//std::cout << "=====================================\n";
// Uncomment to see the wonders of opencv in action
//line(image, rightp1, rightp2, Scalar(0,255,0),2);
//std::cout << rightLaneIntercept << "   " << rightNearLaneSlope << std::endl;
///////// END LCF LANE DETECTION

  //namedWindow("Lane Detection", WINDOW_AUTOSIZE);
  //imshow("Lane Detection", contours);
  
  //draw roi boundary last so it is on top!
  //rectangle(image,ROI,Scalar(0,255,0),2);
  namedWindow("Camera View", WINDOW_AUTOSIZE);
  imshow("Camera View", image);
  waitKey(4);
}
