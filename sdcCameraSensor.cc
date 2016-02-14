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
// #include "opencv2/cudaobjdetect.hpp"
#include <opencv2/opencv.hpp>
// #include <opencv2/cudaimgproc.hpp>
// #include <opencv2/core/cuda.hpp>

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
}

// Called by the world update start event
void sdcCameraSensor::OnUpdate() {
  // pull raw data from camera sensor object
  const unsigned char* img = this->parentSensor->GetImageData(1);
  Mat image = Mat(this->parentSensor->GetImageHeight(1), this->parentSensor->GetImageWidth(1), CV_8UC3, const_cast<unsigned char*>(img));

  //Select Region of Interest (ROI) for lane detection - this is the bottom half of the image.
  //Mat imageROI = image(cv::Rect(0, (4*image.rows)/5, image.cols, image.rows/5));
  Mat imageROI = image(cv::Rect(0, image.rows/2, image.cols, image.rows/2));
  // Canny algorithm for edge dectection
  Mat contours;
  Canny(imageROI,contours,50,150);
  //Mat contoursInv;
  //threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
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
  line(image, Point((imageROI.cols/2)-10,0), Point((imageROI.cols/2)+10,0), Scalar(255,255,255), 2);
  line(image, Point(imageROI.cols/2,10), Point(imageROI.cols/2,-10), Scalar(255,255,255), 2);

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
      ++it;
  }

  //draw left lane marker
  Point leftp1 = Point(left_lane_marker[0]/cos(left_lane_marker[1]),0);
  Point leftp2 = Point((left_lane_marker[0] - imageROI.rows * sin(left_lane_marker[1])) / cos(left_lane_marker[1]), imageROI.rows);
  line(imageROI, leftp1, leftp2, Scalar(255), 3);

  //draw right lane marker
  Point rightp1 = Point(right_lane_marker[0]/cos(right_lane_marker[1]),0);
  Point rightp2 = Point((right_lane_marker[0] - imageROI.rows * sin(right_lane_marker[1])) / cos(right_lane_marker[1]), imageROI.rows);
  line(imageROI, rightp1, rightp2, Scalar(255), 3);


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
double leftNearLaneSlope = 1., rightNearLaneSlope = 1., leftLaneIntercept, rightLaneIntercept;
double a, b, c, d;
// Using the two lane markers
Point vanashingPoint;

// using the right lane
if (rightp2.x - rightp1.x != 0) {
    rightNearLaneSlope = (1.*rightp2.y - rightp1.y)/(rightp2.x - rightp1.x);
}

// The intercept is appearing on the midline, also the top of the ROI, for some reason
rightLaneIntercept = ((0-rightp1.y)/rightNearLaneSlope)+rightp1.x;

a = rightNearLaneSlope/2;
// b = v+n/2;
c = pow(rightNearLaneSlope,2)/4;
// d = a * (n-v);

// using the left lane
if (leftp1.x - leftp2.x != 0) {
    leftNearLaneSlope = (1.*leftp1.y - leftp2.y)/(leftp1.x - leftp2.x);
}

// Uncomment to see the wonders of opencv in action
//line(image, rightp1, rightp2, Scalar(0,255,0),2);

//std::cout << rightLaneIntercept << "   " << rightNearLaneSlope << std::endl;
///////// END LCF LANE DETECTION

  namedWindow("Lane Detection", WINDOW_AUTOSIZE);
  imshow("Lane Detection", contours);
  namedWindow("Camera View", WINDOW_AUTOSIZE);
  imshow("Camera View", image);
  waitKey(4);
}
