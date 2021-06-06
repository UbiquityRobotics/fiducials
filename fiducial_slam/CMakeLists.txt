cmake_minimum_required(VERSION 3.5)
project(fiducial_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fiducial_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  filesystem
)


set(DEPENDENCIES
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  fiducial_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
  image_transport
  tf2_geometry_msgs
  std_srvs
)

###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(include)
# catkin_package(INCLUDE_DIRS include
#   DEPENDS OpenCV)

# include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(fiducial_slam_node #TODO ROS2PORT should be "fiducial_slam" but the clashes with rosidl_generate_interfaces
               src/fiducial_slam.cpp
               src/map.cpp
               src/transform_with_variance.cpp
               src/observation.cpp)

target_link_libraries(fiducial_slam_node ${Boost_LIBRARIES})

ament_target_dependencies(fiducial_slam_node ${DEPENDENCIES} OpenCV)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS fiducial_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(FILES fiducials.rviz
  DESTINATION share/${PROJECT_NAME}
)

###########
## Tests ##
###########

if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  install(DIRECTORY test/test_images
    DESTINATION share/${PROJECT_NAME}/test
  )

  # transform_var_test
  ament_add_gtest(transform_var_test
                  test/transform_var_test.cpp
                  src/transform_with_variance.cpp)
  ament_target_dependencies(transform_var_test
                            "geometry_msgs")

  # auto_init_403_test
  ament_add_gtest_executable(
    auto_init_403_test
    test/auto_init_403_test.cpp
  )
  target_link_libraries(auto_init_403_test)
  ament_target_dependencies(auto_init_403_test ${DEPENDENCIES})
  add_ros_test(test/auto_init_403.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
endif()
# if(CATKIN_ENABLE_TESTING)
#         find_package(catkin REQUIRED COMPONENTS
#           rostest roscpp sensor_msgs tf2_ros image_transport cv_bridge)

#         # Tests need c++11
#         add_definitions(-std=c++11)
	
# 	catkin_add_gtest(transform_var_test test/transform_var_test.cpp src/transform_with_variance)
# 	target_link_libraries(transform_var_test ${catkin_LIBRARIES})

#         add_rostest(test/create_map_aruco.xml)
#         add_rostest(test/init_map_aruco.xml)

#         add_rostest_gtest(auto_init_403_test
#            test/auto_init_403.test
#            test/auto_init_403_test.cpp)
#         target_link_libraries(auto_init_403_test
#             ${catkin_LIBRARIES}
#             ${OpenCV_LIBS})

# endif()

ament_export_include_directories(include)
ament_package()
