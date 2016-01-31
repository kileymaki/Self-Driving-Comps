cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
#link_directories(${OPENCV_LIBRARY_DIRS})

MESSAGE(WARNING ${GAZEBO_LIBRARIES})
MESSAGE(WARNING ${OpenCV_LIBS})
MESSAGE(WARNING ${OPENCV_INCLUDE_DIRS})
MESSAGE(WARNING ${OPENCV_LIBRARY_DIRS})
MESSAGE(WARNING ${OPENCV_LIBS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_library(sdcBox SHARED sdcBox.cpp)
target_link_libraries(sdcBox ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcFlyingBox SHARED sdcFlyingBox.cc)
target_link_libraries(sdcFlyingBox ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcSensorData SHARED sdcSensorData.cc)
target_link_libraries(sdcSensorData ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcLaserSensor SHARED sdcLaserSensor.cc)
target_link_libraries(sdcLaserSensor sdcSensorData ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcTopLaserSensor SHARED sdcTopLaserSensor.cc)
target_link_libraries(sdcTopLaserSensor sdcSensorData ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcGpsSensor SHARED sdcGpsSensor.cc)
target_link_libraries(sdcGpsSensor sdcSensorData ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcCameraSensor SHARED sdcCameraSensor.cc)
target_link_libraries(sdcCameraSensor sdcSensorData ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_library(Angle SHARED Angle.cc)
target_link_libraries(Angle ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcCar SHARED sdcCar.cc)
target_link_libraries(sdcCar Angle sdcSensorData sdcGpsSensor sdcLaserSensor ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
