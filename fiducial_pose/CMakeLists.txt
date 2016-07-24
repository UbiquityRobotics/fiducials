cmake_minimum_required(VERSION 2.8)

project(fiducial_pose)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   message_generation
   OpenCV
   geometry_msgs)

#find_package(OpenCV REQUIRED)

add_message_files(
   FILES
   Fiducial.msg
   FiducialTransform.msg
   FiducialTransformArray.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs 
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rpp_pose
)


include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


add_library(rpp_pose src/Rpoly.cpp src/RPP.cpp src/calcPose.cpp)
target_link_libraries(rpp_pose m ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_dependencies(rpp_pose fiducial_detect_gencpp fiducial_pose_gencpp)

install(PROGRAMS
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
