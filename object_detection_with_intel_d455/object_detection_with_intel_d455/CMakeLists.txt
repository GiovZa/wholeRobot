cmake_minimum_required(VERSION 3.0.2)
project(object_detection_with_intel_d455)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)


include_directories(

  ${catkin_INCLUDE_DIRS}
)

