cmake_minimum_required(VERSION 2.8.3)
project(fiducials_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  fiducials
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
  image_transport
  cv_bridge
  sensor_msgs
)
find_package(orocos_kdl REQUIRED)


catkin_package()

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(fiducials_localization src/fiducials_localization.cpp)
target_link_libraries(fiducials_localization ${catkin_LIBRARIES}
${orocos_kdl_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS fiducials_localization
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

