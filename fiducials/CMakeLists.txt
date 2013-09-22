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

target_link_libraries(Demo m ${OpenCV_LIBS})
target_link_libraries(Tags m ${OpenCV_LIBS})
target_link_libraries(Map_Test m ${OpenCV_LIBS})
target_link_libraries(Video_Capture m ${OpenCV_LIBS})

catkin_package(
    CATKIN_DEPENDS geometry_msgs tf
)

install(PROGRAMS
	Demo Tags Map_Test Video_Capture
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
