cmake_minimum_required(VERSION 2.8)

project(fiducials)

find_package(catkin REQUIRED COMPONENTS angles)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fiducials
  DEPENDS OpenCV
)

add_definitions(-g)
#add_definitions(-DMEMORY_LEAK_CHECK=1)

include_directories(include/fiducials)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(fiducials_base
  Bounding_Box.cpp CRC.cpp FEC.cpp File.cpp
  Memory.cpp String.cpp SVG.cpp)

add_library(fiducials_cv CV.cpp)
target_link_libraries(fiducials_cv fiducials_base ${OpenCV_LIBS})

add_library(fiducials Fiducials.cpp Arc.cpp Camera_Tag.cpp Map.cpp Tag.cpp)
target_link_libraries(fiducials fiducials_base fiducials_cv)

add_executable(Demo Demo.cpp)
target_link_libraries(Demo fiducials)
target_link_libraries(Demo m)

add_executable(Tags Tags.cpp)
target_link_libraries(Tags fiducials_base)
target_link_libraries(Tags m)

add_executable(Map_Test Map_Test.cpp)
target_link_libraries(Map_Test fiducials)
target_link_libraries(Map_Test m)

add_executable(Video_Capture Video_Capture.cpp)
target_link_libraries(Video_Capture fiducials_cv)
target_link_libraries(Video_Capture m)

install(TARGETS
  Demo Tags Map_Test Video_Capture
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
