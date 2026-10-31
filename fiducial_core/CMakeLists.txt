cmake_minimum_required(VERSION 3.20)
project(fiducial_core LANGUAGES CXX)

# Enforce ISO C++20 Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Position Independent Code for dynamic shared plugin compatibility
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Strict warning flags for code quality and safety
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wnon-virtual-dtor
    -Woverloaded-virtual
    -Wformat=2
  )
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)
find_package(stag_detect REQUIRED)

# Declare Core Vision Engine library (dynamically respects BUILD_SHARED_LIBS)
add_library(${PROJECT_NAME}
  src/ippe_solver.cpp
  src/aruco_detector.cpp
  src/stag_detector.cpp
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${stag_detect_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${OpenCV_LIBS}
  ${stag_detect_LIBRARIES}
)

# Installation targets and exports
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Ament modern CMake packaging
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(OpenCV stag_detect)

# Tier 1 Automated Google Test Suite
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_pinhole_ground_truth test/test_pinhole_ground_truth.cpp)
  target_link_libraries(test_pinhole_ground_truth ${PROJECT_NAME})
  
  ament_add_gtest(test_ippe_anti_flip test/test_ippe_anti_flip.cpp)
  target_link_libraries(test_ippe_anti_flip ${PROJECT_NAME})
  
  ament_add_gtest(test_adversarial_noise test/test_adversarial_noise.cpp)
  target_link_libraries(test_adversarial_noise ${PROJECT_NAME})
endif()

ament_package()
