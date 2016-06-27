cmake_minimum_required(VERSION 2.8.3)
project(height_camera_estimation)

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
	include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS roscpp std_msgs pcl_ros pcl_conversions pcl_msgs
   DEPENDS PCL
)


add_executable(camera_height
  src/Camera_Height.cpp
  src/Camera_Height_main.cpp
)
target_link_libraries(camera_height
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)


