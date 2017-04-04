cmake_minimum_required(VERSION 2.8)

project(fiducial_msgs)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
)

find_package(OpenCV REQUIRED)

add_message_files(
   FILES
   Fiducial.msg
   FiducialArray.msg
   FiducialTransform.msg
   FiducialTransformArray.msg
   FiducialMapEntry.msg
   FiducialMapEntryArray.msg
)

add_service_files(
  FILES
  InitializeMap.srv
)

generate_messages(
  DEPENDENCIES 
  std_msgs 
  geometry_msgs
)

catkin_package(
)

install(TARGETS
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
