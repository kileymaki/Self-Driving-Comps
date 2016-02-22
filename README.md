# Self Driving Comps

The files in this directory rely on an underlying installation of Gazebo found in /Library/Caches/Homebrew/gazebo-6.1.0

Logic that controls the car can be found in the following files:
    * sdcAngle.cc - provides a custom angle class that wraps from 0 to 2Pi
    * sdcCameraSensor.cc - provides the backbone for our image recognition algorithms
    * sdcCar.cc - the main class for the car, controls logical decisions made based on all available data
    * sdcLidarRay.cc - wraps a single ray of a Lidar sensor into an angle and distance relative to the car and provides some convenience methods
    * sdcSensorData.cc - holds available data from sensors, as well as convenience methods; provides the bridge between sdcCar and all sensor plugins
    * sdcVisibleObject.cc - wraps obstructions detected in the Lidar sensors into trackable objects with several convenience and logical methods

Other files contain data gathered by sensors or transfer sensor data from Gazebo's core functionality to our plugins.

This project contains several world files which Gazebo takes in as a command line argument. The world files created for the purpose of exhibiting  some specific functionality of the car are labelled in the fashion #_world_description.world. 
