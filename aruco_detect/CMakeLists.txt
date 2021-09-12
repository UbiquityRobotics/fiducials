cmake_minimum_required(VERSION 3.5)
project(aruco_detect)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED) 
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(fiducial_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(dynamic_reconfigure REQUIRED)

find_package(OpenCV REQUIRED)

#generate_dynamic_reconfigure_options(cfg/DetectorParams.cfg)

#catkin_package(INCLUDE_DIRS DEPENDS OpenCV)


set(DEPENDENCIES
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  vision_msgs
  fiducial_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
  image_transport
)

###########
## Build ##
###########

include_directories(include)
add_definitions(-std=c++11)

#include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(aruco_detect src/aruco_detect.cpp)

#add_dependencies(aruco_detect ${${PROJECT_NAME}_EXPORTED_TARGETS})

ament_target_dependencies(aruco_detect ${DEPENDENCIES} OpenCV)

#############
## Install ##
#############

install(TARGETS aruco_detect
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

## Mark executables and/or libraries for installation
# install(TARGETS aruco_detect
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(PROGRAMS scripts/create_markers.py
        DESTINATION lib/${PROJECT_NAME}
)
# install(DIRECTORY launch/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# ) 

###########
## Tests ##
###########

if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros_testing REQUIRED)

  # Not sure if this is correct
  install(DIRECTORY test/test_images
    DESTINATION share/${PROJECT_NAME}
  )

  ament_add_gmock(
    helpers_test
    test/helpers_test.cpp
  )

  ament_add_gtest_executable(
    aruco_images_test
    test/aruco_images_test.cpp
  )
  target_link_libraries(aruco_images_test)
  ament_target_dependencies(aruco_images_test ${DEPENDENCIES})
  add_ros_test(test/aruco_images.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()

# if(CATKIN_ENABLE_TESTING)
#         find_package(rostest REQUIRED)

#         # Tests need c++11
#         add_definitions(-std=c++11)
        
#         add_rostest_gtest(aruco_images_test 
#           test/aruco_images.test 
#           test/aruco_images_test.cpp)
#         target_link_libraries(aruco_images_test ${catkin_LIBRARIES} ${OpenCV_LIBS})
# endif()

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_package()
