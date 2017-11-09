cmake_minimum_required(VERSION 2.8.3)
project(fiducial_detect)

find_package(catkin REQUIRED COMPONENTS
  fiducial_lib
  fiducial_pose
  fiducial_msgs
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
  image_transport
  compressed_image_transport
  cv_bridge
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()


###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(fiducial_detect src/fiducial_detect.cpp)
target_link_libraries(fiducial_detect ${catkin_LIBRARIES}
                      ${OpenCV_INCLUDE_LIBS})
add_dependencies(fiducial_detect ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS fiducial_detect
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
        
        add_rostest_gtest(fiducial_vertices_test 
          test/fiducial_vertices.test 
          test/fiducial_vertices_test.cpp)
        target_link_libraries(fiducial_vertices_test ${catkin_LIBRARIES})

        add_rostest_gtest(fiducial_transform_test 
          test/fiducial_transform.test 
          test/fiducial_transform_test.cpp)
        target_link_libraries(fiducial_transform_test ${catkin_LIBRARIES})
endif()
