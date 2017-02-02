cmake_minimum_required(VERSION 2.8)

project(fiducial_lib)

find_package(catkin REQUIRED COMPONENTS angles)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fiducials
  DEPENDS OpenCV
)

add_definitions(-g)
#add_definitions(-DMEMORY_LEAK_CHECK=1)

include_directories(include/fiducial_lib)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(fiducials_base
  Bounding_Box.cpp CRC.cpp FEC.cpp File.cpp
  Memory.cpp String.cpp SVG.cpp)

add_library(fiducials_cv CV.cpp)
target_link_libraries(fiducials_cv fiducials_base ${OpenCV_LIBS})

add_library(fiducials Fiducials.cpp)
target_link_libraries(fiducials fiducials_base fiducials_cv)

add_executable(Tags Tags.cpp)
target_link_libraries(Tags fiducials_base)
target_link_libraries(Tags m)

install(TARGETS Tags
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS fiducials_base fiducials_cv fiducials
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

catkin_install_python(PROGRAMS scripts/create_markers.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
