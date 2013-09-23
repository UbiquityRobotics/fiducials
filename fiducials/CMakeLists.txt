project(fiducials)

find_package(OpenCV)

add_definitions(-std=c11)

add_executable(Demo 
    Character.c CRC.c Double.c FEC.c File.c Float.c Integer.c List.c 
    Logical.c Memory.c String.c SVG.c Table.c Unsigned.c 
    Arc.c Camera_Tag.c CV.c Demo.c Map.c Tag.c High_GUI2.c
)
add_executable(Tags 
    Character.c CRC.c Double.c FEC.c File.c Float.c Integer.c List.c 
    Logical.c Memory.c String.c SVG.c Table.c Unsigned.c 
    Tags.c
)
add_executable(Map_Test
    Character.c CRC.c Double.c FEC.c File.c Float.c Integer.c List.c 
    Logical.c Memory.c String.c SVG.c Table.c Unsigned.c 
    Arc.c CV.c Camera_Tag.c Map.c Map_Test.c Tag.c
)
add_executable(Video_Capture
    Character.c CRC.c Double.c FEC.c File.c Float.c Integer.c List.c 
    Logical.c Memory.c String.c SVG.c Table.c Unsigned.c 
    CV.c High_GUI2.c Video_Capture.c
)

add_executable(FlyCapture2Test
    FlyCapture2Test.c
)

add_executable(Fly_Capture
    Character.c CRC.c Double.c Fly_Capture.c FC2.c File.c Float.c Integer.c List.c 
    Logical.c Memory.c String.c SVG.c Table.c Unsigned.c 
    CV.c High_GUI2.c
)

include_directories(/usr/include/flycapture)

target_link_libraries(FlyCapture2Test flycapture-c flycapture)
target_link_libraries(Fly_Capture flycapture-c flycapture)

target_link_libraries(Demo m ${OpenCV_LIBS})
target_link_libraries(Tags m ${OpenCV_LIBS})
target_link_libraries(Map_Test m ${OpenCV_LIBS})
target_link_libraries(Video_Capture m ${OpenCV_LIBS})
target_link_libraries(Fly_Capture flycapture-c flycapture m ${OpenCV_LIBS})

catkin_package(
    CATKIN_DEPENDS geometry_msgs tf
)

install(PROGRAMS
	Demo Tags Map_Test Video_Capture
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
