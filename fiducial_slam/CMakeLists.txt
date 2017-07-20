
cmake_minimum_required(VERSION 2.8.3)
project(fiducial_slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
  cv_bridge
  sensor_msgs
  std_msgs
  fiducial_msgs
  genmsg
)

find_package(OpenCV REQUIRED)

catkin_package(INCLUDE_DIRS include
  DEPENDS OpenCV)

###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(fiducial_slam src/fiducial_slam.cpp src/estimator.cpp src/map.cpp)
add_dependencies(fiducial_slam ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(fiducial_slam ${catkin_LIBRARIES} ${OpenCV_LIBS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS fiducial_slam
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES fiducials.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

###########
## Tests ##
###########

if(CATKIN_ENABLE_TESTING)
        find_package(catkin REQUIRED COMPONENTS
          rostest roscpp sensor_msgs tf2_ros image_transport cv_bridge)

        # Tests need c++11
        add_definitions(-std=c++11)

        add_rostest(test/create_map_aruco.xml)
        add_rostest(test/init_map_aruco.xml)

        add_rostest_gtest(auto_init_403_test
           test/auto_init_403.test
           test/auto_init_403_test.cpp)
        target_link_libraries(auto_init_403_test
            ${catkin_LIBRARIES}
            ${OpenCV_LIBS})

endif()
