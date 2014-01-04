cmake_minimum_required(VERSION 2.8)

project(fiducials)

find_package(catkin REQUIRED COMPONENTS
  OpenCV roscpp visualization_msgs fiducials_rviz)

add_definitions(-std=c11)
add_definitions(-g)

add_executable(Demo 
    Bounding_Box.c Character.c CRC.c Double.c FEC.c Fiducials.c File.c Float.c
    Integer.c List.c Location.c Logical.c Memory.c String.c SVG.c Table.c
    Unsigned.c Arc.c Camera_Tag.c CV.c Demo.c High_GUI2.c Map.c Tag.c 
)
add_executable(Rviz_Demo 
    Bounding_Box.c Character.c CRC.c Double.c FEC.c Fiducials.c File.c Float.c
    Integer.c List.c Location.c Logical.c Memory.c String.c SVG.c Table.c
    Unsigned.c Arc.c Camera_Tag.c CV.c Rviz_Demo.c High_GUI2.c Map.c Tag.c 
)
add_executable(Tags 
    Bounding_Box.c Character.c CRC.c Double.c FEC.c File.c Float.c Integer.c
    List.c Logical.c Memory.c String.c SVG.c Table.c Unsigned.c
    Tags.c
)
add_executable(Map_Test
    Bounding_Box.c Character.c CRC.c Double.c FEC.c File.c Float.c Integer.c
    List.c Location.c Logical.c Memory.c String.c SVG.c Table.c Unsigned.c 
    Arc.c CV.c Camera_Tag.c Map.c Map_Test.c Tag.c
)
add_executable(Video_Capture
    Bounding_Box.c Character.c CRC.c Double.c FEC.c File.c Float.c Integer.c
    List.c Logical.c Memory.c String.c SVG.c Table.c Unsigned.c 
    CV.c High_GUI2.c Video_Capture.c
)

add_executable(FlyCapture2Test
    FlyCapture2Test.c
)

add_executable(Fly_Capture
    Bounding_Box.c Character.c CRC.c Double.c FEC.c Fiducials.c
    Fly_Capture.c FC2.c File.c Float.c Integer.c List.c Location.c Logical.c
    Memory.c String.c SVG.c Table.c Unsigned.c 
    Arc.c Camera_Tag.c CV.c High_GUI2.c Map.c Tag.c
)

add_executable(undistort undistort.c)

add_executable(Rviz_Test Rviz_Test.c)

include_directories(/usr/include/flycapture ../fiducials_rviz/include)

target_link_libraries(FlyCapture2Test flycapture-c flycapture)
target_link_libraries(Fly_Capture flycapture-c flycapture)

target_link_libraries(Demo m ${OpenCV_LIBS})
target_link_libraries(Rviz_Demo m 
  ${OpenCV_LIBS} ${fiducials_rviz_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(Tags m ${OpenCV_LIBS})
target_link_libraries(Map_Test m ${OpenCV_LIBS})
target_link_libraries(Video_Capture m ${OpenCV_LIBS})
target_link_libraries(Fly_Capture flycapture-c flycapture m ${OpenCV_LIBS})

target_link_libraries(undistort ${OpenCV_LIBS})

target_link_libraries(Rviz_Test ${fiducials_rviz_LIBRARIES} ${catkin_LIBRARIES})

catkin_package(
    CATKIN_DEPENDS geometry_msgs tf
)

install(PROGRAMS
	Demo Tags Map_Test Video_Capture
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
