cmake_minimum_required(VERSION 3.5)

project(fiducial_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Fiducial.msg"
  "msg/FiducialArray.msg"
  "msg/FiducialTransform.msg"
  "msg/FiducialTransformArray.msg"
  "msg/FiducialMapEntry.msg"
  "msg/FiducialMapEntryArray.msg"
)

set(srv_files
  "srv/InitializeMap.srv"
  "srv/AddFiducial.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
