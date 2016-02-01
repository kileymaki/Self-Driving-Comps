#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

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
  const unsigned char* img = this->parentSensor->GetImageData(0);

  Mat image = Mat(this->parentSensor->GetImageHeight(0), this->parentSensor->GetImageWidth(0), CV_8UC3, const_cast<unsigned char*>(img));
  Mat imageROI = image(cv::Rect(0, image.rows/2, image.cols, image.rows/2));

  // Canny algorithm
  Mat contours;
  Canny(imageROI,contours,50,350);
  Mat contoursInv;
  threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);

  float PI = 3.14159;
  std::vector<Vec2f> lines;
  HoughLines(contours,lines,1,PI/180, 100);
  Mat result(contours.rows,contours.cols,CV_8U,Scalar(255));
  image.copyTo(result);

  // Draw the limes
  std::vector<Vec2f>::const_iterator it= lines.begin();
  Mat hough(imageROI.size(),CV_8U,Scalar(0));

  while (it!=lines.end()) {
      float rho= (*it)[0];   // first element is distance rho
      float theta= (*it)[1]; // second element is angle theta
      // point of intersection of the line with first row
      //if (theta < 1.5 || theta > 1.7) {
      Point pt1(rho/cos(theta),0);
      // point of intersection of the line with last row
      Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
      // draw a white line
      line(result, pt1, pt2, Scalar(255), 3);
      line(imageROI, pt1, pt2, Scalar(255), 3);
    //   std::cout << "line: (" << rho << "," << theta << ")\n";
      //}
      ++it;
  }

  namedWindow("Lane Detection", CV_WINDOW_AUTOSIZE);
  imshow("Lane Detection", image);
  waitKey(4);

  //const unsigned char* img2 = this->parentSensor->GetImageData(1);
  //Mat frame2 = Mat(this->parentSensor->GetImageHeight(1), this->parentSensor->GetImageWidth(1), CV_8UC3, const_cast<unsigned char*>(img2));
  //frame.setTo(_InputArray(img));
  //namedWindow("MyWindow2", CV_WINDOW_AUTOSIZE);
  //imshow("MyWindow2", frame2);

}
