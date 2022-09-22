
cmake_minimum_required(VERSION 2.8.3)
project(aruco_detect)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  tf
  visualization_msgs
  vision_msgs
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  fiducial_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

# Python Setup
catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/DetectorParams.cfg)

catkin_package(INCLUDE_DIRS DEPENDS OpenCV)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(aruco_detect src/aruco_detect.cpp)

add_dependencies(aruco_detect ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(aruco_detect ${catkin_LIBRARIES} ${OpenCV_LIBS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS aruco_detect
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/create_markers.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
) 

###########
## Tests ##
###########

if(CATKIN_ENABLE_TESTING)
        find_package(rostest REQUIRED)

        add_rostest_gtest(aruco_images_test 
          test/aruco_images.test 
          test/aruco_images_test.cpp)
        target_link_libraries(aruco_images_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()
