/*
 * This was an older version of the camera sensor that offloaded the
 * image processing to the dedicated GPU. We determined that this caused
 * more trouble than it was worth since the data transfer overhead between 
 * the CPU and GPU resulted in an overall decrease in performace. Had our 
 * camera been higher resolution this most likely would not have been the case
 */

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

//Pointer to Cascade Classifier file for GPU
//Ptr<cuda::CascadeClassifier> stopSign = cuda::CascadeClassifier::create("/Users/selfcar/Desktop/Self-Driving-Comps/OpenCV/haarcascade_stop.xml");

// Cascade Classifier information using CPU
CascadeClassifier cpu_stop_sign;
String cascade_file_path = "/Users/selfcar/Desktop/Self-Driving-Comps/OpenCV/haarcascade_stop.xml";

void sdcCameraSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    /*
    //Ensure Gazebo is making use of GPU Acceleration
    cuda::DeviceInfo::DeviceInfo gpuInfo;
    const char* gpuName = gpuInfo.name();
    int isGPULoaded = cuda::getCudaEnabledDeviceCount();
    if (isGPULoaded == 1){
      std::cout << "OpenCV using CUDA-enabled Graphics Card: " << gpuName << std::endl;
    }
    else {
      std::cout << "Unable to Find CUDA-enabled Graphics Card" << std::endl;
      return;
    }
    */

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
  const unsigned char* img_left = this->parentSensor->GetImageData(0);
  const unsigned char* img_right = this->parentSensor->GetImageData(1);
  /*
  //Allocate page-locked memory in the GPU for the incoming frames
  cuda::HostMem left_src_plm(this->parentSensor->GetImageHeight(0), this->parentSensor->GetImageWidth(0),CV_8UC3, cuda::HostMem::PAGE_LOCKED);
  cuda::HostMem right_src_plm(this->parentSensor->GetImageHeight(1), this->parentSensor->GetImageWidth(1),CV_8UC3, cuda::HostMem::PAGE_LOCKED);
  cuda::HostMem left_dst_plm, right_dst_plm;

  Mat image_left = left_src_plm.createMatHeader();
  Mat image_right = right_src_plm.createMatHeader();

  Mat image_left_add(this->parentSensor->GetImageHeight(0), this->parentSensor->GetImageWidth(0),CV_8UC3,const_cast<unsigned char*>(img_left));
  Mat image_right_add(this->parentSensor->GetImageHeight(1), this->parentSensor->GetImageWidth(1),CV_8UC3,const_cast<unsigned char*>(img_right));
  image_left.push_back(image_left_add);
  image_right.push_back(image_right_add);

  //Pass the frames to the GPU
  cuda::GpuMat gpu_image_left, gpu_image_right;
  cuda::Stream::Stream stream;
  gpu_image_left.upload(left_src_plm, stream);
  gpu_image_right.upload(right_src_plm, stream);
  */

  //Load frames onto CPU
  //Mat image_left = Mat(this->parentSensor->GetImageHeight(0), this->parentSensor->GetImageWidth(0), CV_8UC3, const_cast<unsigned char*>(img_left));
  Mat image_right = Mat(this->parentSensor->GetImageHeight(1), this->parentSensor->GetImageWidth(1), CV_8UC3, const_cast<unsigned char*>(img_right));

  //Select Region of Interest (ROI) for lane detection - this is the bottom half of the image.
  //Mat imageROI_left = image_left(cv::Rect(0, image_left.rows/2, image_left.cols, image_left.rows/2));
  Mat imageROI_right = image_right(cv::Rect(0, image_right.rows/2, image_right.cols, image_right.rows/2));
  // Canny algorithm for edge dectection
  Mat contours_left, contours_right;
  //Canny(imageROI_left,contours_left,50,350);
  Canny(image_right,contours_right,50,350);
  Mat contoursInv_left, contoursInv_right;
  //threshold(contours_left,contoursInv_left,128,255,THRESH_BINARY_INV);
  threshold(contours_right,contoursInv_right,128,255,THRESH_BINARY_INV);

  float PI = 3.14159;
  //std::vector<Vec2f> lines_left;
  std::vector<Vec2f> lines_right;

  //HoughLines(contours_left,lines_left,1,PI/180, 100);
  HoughLines(contours_right,lines_right,1,PI/180, 100);

  //print out line angles
  //for (std::vector<Vec2f>::const_iterator i = lines_right.begin(); i != lines_right.end(); ++i)
  //std::cout << *i << ' ' << std::endl;
  //std::cout << "=====================================\n";

  // Draw the lines
 // std::vector<Vec2f>::const_iterator it_left = lines_left.begin();
  std::vector<Vec2f>::const_iterator it_right = lines_right.begin();

  // white line grid overlay for reference points on displayed image
  //line(imageROI_left, Point(0,0), Point(imageROI_left.cols,0), Scalar(255,255,255), 2);
  //line(imageROI_left, Point(imageROI_left.cols/2,0), Point(imageROI_left.cols/2,imageROI_left.cols), Scalar(255,255,255), 2);
  line(imageROI_right, Point(0,0), Point(imageROI_right.cols,0), Scalar(255,255,255), 2);
  line(imageROI_right, Point(imageROI_right.cols/2,0), Point(imageROI_right.cols/2,imageROI_right.cols), Scalar(255,255,255), 2);


  //this isnt working that well - should classify lines on left and lines on right
  //based on their theta value and then take averages of left and right lines.
  //take the area between the single left and single right line as our area we can
  //go to
  //iter over left
  /*
  std::vector<float> pt1_x;
  std::vector<float> pt1_y;
  std::vector<float> pt2_x;
  std::vector<float> pt2_y;
  while (it_left!=lines_left.end()) {
      float rho= (*it_left)[0];   // first element is distance rho
      float theta= (*it_left)[1]; // second element is angle theta

      //if ( (theta > 0.09 && theta < 1.48) || (theta < 3.14 && theta > 1.66) ){
      if ((theta > 0 && theta < 3.14)) {
          Point pt1(rho/cos(theta),0);
          pt1_x.push_back(rho/cos(theta));
          pt1_y.push_back(0);
          Point pt2((rho-image_left.rows*sin(theta))/cos(theta),image_left.rows);
          pt2_x.push_back((rho-image_left.rows*sin(theta))/cos(theta));
          pt2_y.push_back(image_left.rows);
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
  */
  //iter over right
  while (it_right!=lines_right.end()) {
      float rho= (*it_right)[0];   // first element is distance rho
      float theta= (*it_right)[1]; // second element is angle theta
      // point of intersection of the line with first row
      //if ( (theta > 0.09 && theta < 1.48) || (theta < 3.14 && theta > 1.66) ){
      //if ( (theta > 0.8 && theta < 1.2) || (theta > 2.2 && theta < 2.4) ) {
          Point pt1(rho/cos(theta),0);
          // point of intersection of the line with last row
          Point pt2((rho-image_right.rows*sin(theta))/cos(theta),image_right.rows);
          // draw line
          line(image_right, pt1, pt2, Scalar(255), 3);
    //}
      ++it_right;
  }


  //BEGIN HAAR CASCADE OBJECT DETECTION
/*
  if(!cpu_stop_sign.load(cascade_file_path)){ printf("--(!)Error loading face cascade\n");};
  std::vector<Rect> stopSigns_left, stopSigns_right;
  cpu_stop_sign.detectMultiScale( image_left, stopSigns_left, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
  cpu_stop_sign.detectMultiScale( image_right, stopSigns_right, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
  sdcSensorData::stopSignInLeftCamera = false;
  for( int i = 0; i < stopSigns_left.size(); i++ )
  {
     cv::rectangle(image_left, stopSigns_left[i], Scalar(0,0,255),3,LINE_8,0);
     std::cout << "stop sign found in left image" <<std::endl;
     sdcSensorData::stopSignInLeftCamera = true;
  }

  sdcSensorData::stopSignInRightCamera = false;
  for( int i = 0; i < stopSigns_right.size(); i++ )
  {
     cv::rectangle(image_right, stopSigns_right[i], Scalar(0,0,255),3,LINE_8,0);
     std::cout << "stop sign found in right image" <<std::endl;
     sdcSensorData::stopSignInRightCamera = true;
  }
*/

  //comment out gpu implementation because it sucks
  /*
  std::vector<Rect> stopSigns_left, stopSigns_right;
  cuda::GpuMat left_gpu_buffer, right_gpu_buffer;

  stopSign->setFindLargestObject(false);
  stopSign->setScaleFactor(1.1);
  stopSign->setMinNeighbors(4);
  stopSign->setMinObjectSize(cv::Size(40,40));
  stopSign->setMaxObjectSize(image_left.size());


  stopSign->detectMultiScale(gpu_image_left, left_gpu_buffer,stream);
  stopSign->detectMultiScale(gpu_image_right, right_gpu_buffer,stream);
  stopSign->convert(left_gpu_buffer, stopSigns_left);
  stopSign->convert(right_gpu_buffer, stopSigns_right);

  for( int i = 0; i < stopSigns_left.size(); i++ )
  {
     //cv::rectangle(gpu_image_left, stopSigns_left[i], Scalar(255),1,LINE_8,0);
     Point center( stopSigns_left[i].x + stopSigns_left[i].width*0.5, stopSigns_left[i].y + stopSigns_left[i].height*0.5 );
     ellipse( gpu_image_left, center, Size( stopSigns_left[i].width*0.5, stopSigns_left[i].height*0.5), 0, 0, 360, Scalar( 0, 0, 255 ), 4, 8, 0 );
     std::cout << "stop sign found in left image" <<std::endl;
  }

  for( int i = 0; i < stopSigns_right.size(); i++ )
  {
     //cv::rectangle(gpu_image_right, stopSigns_right[i], Scalar(255),1,LINE_8,0);
     Point center( stopSigns_right[i].x + stopSigns_right[i].width*0.5, stopSigns_right[i].y + stopSigns_right[i].height*0.5 );
     ellipse( gpu_image_right, center, Size( stopSigns_right[i].width*0.5, stopSigns_right[i].height*0.5), 0, 0, 360, Scalar( 0, 0, 255 ), 4, 8, 0 );
     std::cout << "stop sign found in right image" <<std::endl;
  }

  Mat image_left_final;
  Mat image_right_final;
  gpu_image_left.download(image_left_final,stream);
  gpu_image_right.download(image_right_final,stream);


  //stream.waitForCompletion();
  */

  //namedWindow("Lane Detection Left", CV_WINDOW_AUTOSIZE);
  namedWindow("Lane Detection Right", CV_WINDOW_AUTOSIZE);
  //imshow("Lane Detection Left", image_left);
  imshow("Lane Detection Right", contours_right);

  waitKey(4);
}
