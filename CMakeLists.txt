cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(model_push SHARED model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(model_pull SHARED model_pull.cpp)
target_link_libraries(model_pull ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(model_car SHARED model_car.cpp)
target_link_libraries(model_car ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(laser_car SHARED laser_car.cc)
target_link_libraries(laser_car ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(VehiclePlugin SHARED VehiclePlugin.cc)
target_link_libraries(VehiclePlugin laser_car ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(car_gps SHARED car_gps.cc)
target_link_libraries(car_gps ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(camera_listener camera_listener.cc)
#target_link_libraries(camera_listener ${GAZEBO_LIBRARIES} pthread)
