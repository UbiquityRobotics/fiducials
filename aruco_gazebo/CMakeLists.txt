cmake_minimum_required(VERSION 3.0.2)
project(aruco_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  vision_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
