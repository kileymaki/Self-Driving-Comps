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
  printf("\n\n");
  // Kappa
  std::cout << this->parentSensor->GetImageHeight(0)*this->parentSensor->GetImageWidth(0) << std::endl;
  const unsigned char * img = this->parentSensor->GetImageData(0);
  // ResidentSleeper
  std::string image = std::string(reinterpret_cast<const char*>(this->parentSensor->GetImageData(0)));

  Mat frame;
  frame = Mat(this->parentSensor->GetImageHeight(0), this->parentSensor->GetImageWidth(0), CV_16UC1, this);

  namedWindow("MyWindow", CV_WINDOW_AUTOSIZE);
  imshow("MyWindow", frame);
  waitKey(0);
  destroyWindow("MyWindow");

  // PogChamp
  printf("%lu\n",image.size());

}
