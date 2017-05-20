
cmake_minimum_required(VERSION 2.8.3)
project(aruco_detect)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  fiducial_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/DetectorParams.cfg)

catkin_package(INCLUDE_DIRS include
  DEPENDS OpenCV)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(aruco_detect src/aruco_detect.cpp)
add_executable(test_board src/test_board.cpp)
add_executable(create_marker src/create_marker.cpp)

add_dependencies(aruco_detect ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(aruco_detect ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(test_board ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(create_marker ${OpenCV_LIBS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS aruco_detect create_marker
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

        # Tests need c++11
        add_definitions(-std=c++11)
        
        add_rostest_gtest(aruco_images_test 
          test/aruco_images.test 
          test/aruco_images_test.cpp)
        target_link_libraries(aruco_images_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()
