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
#include "opencv2/objdetect/objdetect.hpp"

#include "sdcCameraSensor.hh"

using namespace gazebo;
using namespace cv;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(sdcCameraSensor)

// Pointer to the update event connection
event::ConnectionPtr updateConnection;
sensors::MultiCameraSensorPtr parentSensor;

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
void sdcCameraSensor::OnUpdate(){
  // printf("\n\n");

  // Kappa
  const unsigned char* img_left = this->parentSensor->GetImageData(0);
  const unsigned char* img_right = this->parentSensor->GetImageData(1);

  Mat image_left = Mat(this->parentSensor->GetImageHeight(0), this->parentSensor->GetImageWidth(0), CV_8UC3, const_cast<unsigned char*>(img_left));
  Mat image_right = Mat(this->parentSensor->GetImageHeight(1), this->parentSensor->GetImageWidth(1), CV_8UC3, const_cast<unsigned char*>(img_right));

  Mat imageROI_left = image_left(cv::Rect(0, image_left.rows/2, image_left.cols, image_left.rows/2));
  Mat imageROI_right = image_right(cv::Rect(0, image_right.rows/2, image_right.cols, image_right.rows/2));

  // Canny algorithm
  Mat contours_left, contours_right;
  Canny(imageROI_left,contours_left,50,350);
  Canny(imageROI_right,contours_right,50,350);
  Mat contoursInv_left, contoursInv_right;
  threshold(contours_left,contoursInv_left,128,255,THRESH_BINARY_INV);
  threshold(contours_right,contoursInv_right,128,255,THRESH_BINARY_INV);

  float PI = 3.14159;
  std::vector<Vec2f> lines_left;
  std::vector<Vec2f> lines_right;

  HoughLines(contours_left,lines_left,1,PI/180, 100);
  HoughLines(contours_right,lines_right,1,PI/180, 100);
  Mat result_left(contours_left.rows,contours_left.cols,CV_8U,Scalar(255));
  Mat result_right(contours_right.rows,contours_right.cols,CV_8U,Scalar(255));
  image_left.copyTo(result_left);
  image_right.copyTo(result_right);
  //print out line angles
  //for (std::vector<Vec2f>::const_iterator i = lines_left.begin(); i != lines_left.end(); ++i)
  //  std::cout << *i << ' ' << std::endl;
  //std::cout << "=====================================\n";
  // Draw the lines
  std::vector<Vec2f>::const_iterator it_left = lines_left.begin();
  std::vector<Vec2f>::const_iterator it_right = lines_right.begin();

  line(imageROI_left, Point(0,0), Point(320,0), Scalar(255,255,255), 1);
  line(imageROI_left, Point(160,0), Point(160,120), Scalar(255,255,255), 1);
  line(imageROI_right, Point(0,0), Point(320,0), Scalar(255,255,255), 1);
  line(imageROI_right, Point(160,0), Point(160,120), Scalar(255,255,255), 1);


//this isnt working that well - should classify lines on left and lines on right
//based on their theta value and then take averages of left and right lines.
//take the area between the single left and single right line as our area we can
//go to
  //iter over left
  std::vector<float> pt1_x;
  std::vector<float> pt1_y;
  std::vector<float> pt2_x;
  std::vector<float> pt2_y;
  while (it_left!=lines_left.end()) {
      float rho= (*it_left)[0];   // first element is distance rho
      float theta= (*it_left)[1]; // second element is angle theta

      if ( (theta > 0.09 && theta < 1.48) || (theta < 3.14 && theta > 1.66) ){
          Point pt1(rho/cos(theta),0);
          pt1_x.push_back(rho/cos(theta));
          pt1_y.push_back(0);
          Point pt2((rho-result_left.rows*sin(theta))/cos(theta),result_left.rows);
          pt2_x.push_back((rho-result_left.rows*sin(theta))/cos(theta));
          pt2_y.push_back(result_left.rows);
    }
      ++it_left;
  }
  if (pt1_x.size() > 0 && pt1_y.size() > 0 && pt2_x.size() > 0 && pt2_y.size() > 0) {
      float pt1_x_average = std::accumulate(pt1_x.begin(), pt1_x.end(), 0)/pt1_x.size();
      float pt1_y_average = std::accumulate(pt1_y.begin(), pt1_y.end(), 0)/pt1_y.size();
      float pt2_x_average = std::accumulate(pt2_x.begin(), pt2_x.end(), 0)/pt2_x.size();
      float pt2_y_average = std::accumulate(pt2_y.begin(), pt2_y.end(), 0)/pt2_y.size();
      line(imageROI_left, Point(pt1_x_average,pt1_y_average), Point(pt2_x_average,pt2_y_average), Scalar(255), 3);
  }

//iter over right
  while (it_right!=lines_right.end()) {
      float rho= (*it_right)[0];   // first element is distance rho
      float theta= (*it_right)[1]; // second element is angle theta
      // point of intersection of the line with first row
      //if (theta < 1.5 || theta > 1.7)
      Point pt1(rho/cos(theta),0);
      // point of intersection of the line with last row
      Point pt2((rho-result_right.rows*sin(theta))/cos(theta),result_right.rows);
      // draw a white line
      line(imageROI_right, pt1, pt2, Scalar(255), 3);
      ++it_right;
  }

  //BEGIN HAAR CASCADE OBJECT DETECTION
  CascadeClassifier stopSign;
  //Load Stop Sign Schema for Haar - this should NOT be in the OnUpdate Loop!!
  if (!stopSign.load("/Users/selfcar/Desktop/haarcascade_stop.xml")) {
    printf("Error: Unable to load stop sign cascade xml file!\n");
  }

  std::vector<Rect> stopSigns_left;
  std::vector<Rect> stopSigns_right;
  stopSign.detectMultiScale( image_left, stopSigns_left, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
  stopSign.detectMultiScale( image_right, stopSigns_right, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

  for( int i = 0; i < stopSigns_left.size(); i++ )
  {
     Point center( stopSigns_left[i].x + stopSigns_left[i].width*0.5, stopSigns_left[i].y + stopSigns_left[i].height*0.5 );
     ellipse( image_left, center, Size( stopSigns_left[i].width*0.5, stopSigns_left[i].height*0.5), 0, 0, 360, Scalar( 0, 0, 255 ), 4, 8, 0 );
  }
  
  for( int i = 0; i < stopSigns_right.size(); i++ )
  {
     Point center( stopSigns_right[i].x + stopSigns_right[i].width*0.5, stopSigns_right[i].y + stopSigns_right[i].height*0.5 );
     ellipse( image_right, center, Size( stopSigns_right[i].width*0.5, stopSigns_right[i].height*0.5), 0, 0, 360, Scalar( 0, 0, 255 ), 4, 8, 0 );
  }

  namedWindow("Lane Detection Left", CV_WINDOW_AUTOSIZE);
  namedWindow("Lane Detection Right", CV_WINDOW_AUTOSIZE);
  imshow("Lane Detection Left", image_left);
  imshow("Lane Detection Right", image_right);

  waitKey(4);
}
