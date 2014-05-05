cmake_minimum_required(VERSION 2.8)

project(fiducials)

find_package(catkin REQUIRED COMPONENTS)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fiducials
  DEPENDS OpenCV
)

add_definitions(-g)
#add_definitions(-DMEMORY_LEAK_CHECK=1)

# Really simple, poor-man's find_package for the flycapture library
if(EXISTS /usr/include/flycapture)
  set(flycapture_FOUND true)
  include_directories(/usr/include/flycapture)
endif()

include_directories(include/fiducials)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(fiducials_base
  Bounding_Box.cpp Character.cpp CRC.cpp Double.cpp FEC.cpp File.cpp Float.cpp 
  List.cpp Memory.cpp String.cpp SVG.cpp Table.cpp Unsigned.cpp)

add_library(fiducials_cv CV.cpp High_GUI2.cpp)
target_link_libraries(fiducials_cv fiducials_base ${OpenCV_LIBS})

add_library(fiducials Fiducials.cpp Location.cpp Arc.cpp Camera_Tag.cpp Map.cpp Tag.cpp)
target_link_libraries(fiducials fiducials_base fiducials_cv)

add_executable(Demo Demo.cpp)
target_link_libraries(Demo fiducials)
target_link_libraries(Demo m)

#add_executable(Rviz_Demo Rviz_Demo.cpp)
#target_link_libraries(Rviz_Demo fiducials)
#target_link_libraries(Rviz_Demo m ${catkin_LIBRARIES})

add_executable(Tags Tags.cpp)
target_link_libraries(Tags fiducials_base)
target_link_libraries(Tags m)

add_executable(Map_Test Map_Test.cpp)
target_link_libraries(Map_Test fiducials)
target_link_libraries(Map_Test m)

add_executable(Video_Capture Video_Capture.cpp)
target_link_libraries(Video_Capture fiducials_cv)
target_link_libraries(Video_Capture m)

if(${flycapture_FOUND})
  add_executable(FlyCapture2Test FlyCapture2Test.cpp)
  target_link_libraries(FlyCapture2Test flycapture-c flycapture)

  add_executable(Fly_Capture Fly_Capture.cpp FC2.cpp)
  target_link_libraries(Fly_Capture flycapture-c flycapture m fiducials
   ${OpenCV_LIBS})
endif()

#add_executable(undistort undistort.cpp)
#target_link_libraries(undistort ${OpenCV_LIBS})

#add_executable(Rviz_Test Rviz_Test.cpp)
#target_link_libraries(Rviz_Test ${catkin_LIBRARIES})

install(TARGETS
  Demo Tags Map_Test Video_Capture
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
