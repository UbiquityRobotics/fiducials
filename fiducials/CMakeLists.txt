cmake_minimum_required(VERSION 2.8)

project(fiducials)

find_package(catkin REQUIRED COMPONENTS fiducials_rviz)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fiducials
  DEPENDS OpenCV
  CATKIN_DEPENDS fiducials_rviz
)

add_definitions(-std=c11)
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
  Bounding_Box.c Character.c CRC.c Double.c FEC.c File.c Float.c Integer.c
  List.c Logical.c Memory.c String.c SVG.c Table.c Unsigned.c)

add_library(fiducials_cv CV.c High_GUI2.c)
target_link_libraries(fiducials_cv fiducials_base ${OpenCV_LIBS})

add_library(fiducials Fiducials.c Location.c Arc.c Camera_Tag.c Map.c Tag.c)
target_link_libraries(fiducials fiducials_base fiducials_cv)

add_executable(Demo Demo.c)
target_link_libraries(Demo fiducials)
target_link_libraries(Demo m)

add_executable(Rviz_Demo Rviz_Demo.c)
target_link_libraries(Rviz_Demo fiducials)
target_link_libraries(Rviz_Demo m ${catkin_LIBRARIES})

add_executable(Tags Tags.c)
target_link_libraries(Tags fiducials_base)
target_link_libraries(Tags m)

add_executable(Map_Test Map_Test.c)
target_link_libraries(Map_Test fiducials)
target_link_libraries(Map_Test m)

add_executable(Video_Capture Video_Capture.c)
target_link_libraries(Video_Capture fiducials_cv)
target_link_libraries(Video_Capture m)

if(${flycapture_FOUND})
  add_executable(FlyCapture2Test FlyCapture2Test.c)
  target_link_libraries(FlyCapture2Test flycapture-c flycapture)

  add_executable(Fly_Capture Fly_Capture.c FC2.c)
  target_link_libraries(Fly_Capture flycapture-c flycapture m fiducials
   ${OpenCV_LIBS})
endif()

add_executable(undistort undistort.c)
target_link_libraries(undistort ${OpenCV_LIBS})

add_executable(Rviz_Test Rviz_Test.c)
target_link_libraries(Rviz_Test ${catkin_LIBRARIES})

install(PROGRAMS
	Demo Tags Map_Test Video_Capture
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
