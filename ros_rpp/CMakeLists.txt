cmake_minimum_required(VERSION 2.8)

project(ros_rpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf)

find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(. ${OpenCV_INCLUDE_DIRS})


add_executable(rpp_pose Rpoly.cpp RPP.cpp calcPose.cpp)
target_link_libraries(rpp_pose m ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(rpp_pose fiducials_ros_gencpp)

install(PROGRAMS
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
