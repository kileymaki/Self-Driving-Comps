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
#include "opencv2/cudaobjdetect.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core/cuda.hpp>

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

  // Draw the lines

  std::vector<Vec2f>::const_iterator it = lines.begin();

  // white line grid overlay for reference points on displayed image
  line(image, Point((image.cols/2)-10,0), Point((image.cols/2)+10,0), Scalar(255,255,255), 2);
  line(image, Point(image.cols/2,10), Point(image.cols/2,-10), Scalar(255,255,255), 2);


  //this isnt working that well - should classify lines on left and lines on right
  //based on their theta value and then take averages of left and right lines.
  //take the area between the single left and single right line as our area we can
  //go to

  //iter over right
  //std::vector<float> s;
  //std::vector<float> c;
  //std::vector<float> p;

  while (it!=lines.end()) {
      float rho= (*it)[0];   // first element is distance rho
      float theta= (*it)[1]; // second element is angle theta
      //s.push_back(sin(theta));
      //c.push_back(cos(theta));
      //p.push_back(rho);
      // point of intersection of the line with first row
      //if ( (theta > 0.09 && theta < 1.48) || (theta < 3.14 && theta > 1.66) ){
      //if ( (theta > 0.8 && theta < 1.2) || (theta > 2.2 && theta < 2.4) ) {
          Point pt1(rho/cos(theta),0);
          // point of intersection of the line with last row
          Point pt2((rho-imageROI.rows*sin(theta))/cos(theta),imageROI.rows);
          // draw line
          line(imageROI, pt1, pt2, Scalar(255), 3);
    //}
      ++it;
  }
  /*
  Mat Mat_s = Mat(s);
  Mat Mat_c = Mat(c);
  Mat Mat_p = Mat(p);

  std::vector<float>::const_iterator i;
  for (i = s.begin(); i != s.end(); ++i)
  std::cout << *i << ' ';
  std::cout << "\n";
  for (i = c.begin(); i != c.end(); ++i)
  std::cout << *i << ' ';
  std::cout << "\n";
  for (i = p.begin(); i != p.end(); ++i)
  std::cout << *i << ' ';
  std::cout << "\n=====================================\n";
  */

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

  namedWindow("Lane Detection", CV_WINDOW_AUTOSIZE);
  imshow("Lane Detection", contours);
  namedWindow("Camera View", CV_WINDOW_AUTOSIZE);
  imshow("Camera View", image);
  waitKey(4);
}
