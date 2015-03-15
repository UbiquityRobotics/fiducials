cmake_minimum_required(VERSION 2.8)

project(fiducial_pose)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
)

find_package(OpenCV REQUIRED)

add_message_files(
   FILES
   FiducialTransform.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs 
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(include ${OpenCV_INCLUDE_DIRS})


add_executable(rpp_pose src/Rpoly.cpp src/RPP.cpp src/calcPose.cpp)
target_link_libraries(rpp_pose m ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_dependencies(rpp_pose fiducials_ros_gencpp ros_rpp_gencpp)

install(PROGRAMS
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
