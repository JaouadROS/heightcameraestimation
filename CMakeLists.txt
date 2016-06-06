cmake_minimum_required(VERSION 2.8.3)
project(heightcameraestimation)

set(catkin_RUN_PACKAGES
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  pcl_msgs
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
   LIBRARIES
   CATKIN_DEPENDS roscpp std_msgs pcl_ros pcl_conversions pcl_msgs#actionlib_msgs actionlib
   DEPENDS PCL

)

add_executable(subKinect
  src/subKinect.cpp
)
target_link_libraries(subKinect
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

