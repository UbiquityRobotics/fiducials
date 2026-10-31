cmake_minimum_required(VERSION 3.20)
project(fiducial_detect LANGUAGES CXX)

# Enforce ISO C++20 Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Position Independent Code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Strict warning flags for safety
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wnon-virtual-dtor
    -Woverloaded-virtual
  )
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(fiducial_msgs REQUIRED)
find_package(fiducial_core REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d)

# ------------------------------------------------------------------------------
# 1. ArUco Composable Component & Standalone Node
# ------------------------------------------------------------------------------
add_library(aruco_detect_component SHARED
  src/aruco_detect_component.cpp
)
target_compile_features(aruco_detect_component PUBLIC cxx_std_20)

target_include_directories(aruco_detect_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(aruco_detect_component
  ${OpenCV_LIBS}
)

ament_target_dependencies(aruco_detect_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  fiducial_msgs
  fiducial_core
)

rclcpp_components_register_node(aruco_detect_component
  PLUGIN "fiducial_detect::ArucoDetectComponent"
  EXECUTABLE aruco_detector_node
)

# ------------------------------------------------------------------------------
# 2. STag Composable Component & Standalone Node
# ------------------------------------------------------------------------------
add_library(stag_detect_component SHARED
  src/stag_detect_component.cpp
)
target_compile_features(stag_detect_component PUBLIC cxx_std_20)

target_include_directories(stag_detect_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(stag_detect_component
  ${OpenCV_LIBS}
)

ament_target_dependencies(stag_detect_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  fiducial_msgs
  fiducial_core
)

rclcpp_components_register_node(stag_detect_component
  PLUGIN "fiducial_detect::StagDetectComponent"
  EXECUTABLE stag_detector_node
)

# ------------------------------------------------------------------------------
# Installation & Packaging
# ------------------------------------------------------------------------------
install(TARGETS
  aruco_detect_component
  stag_detect_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(aruco_detect_component stag_detect_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  fiducial_msgs
  fiducial_core
  OpenCV
)

ament_package()
