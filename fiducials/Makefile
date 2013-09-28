# Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

# ROS_ROOT needs to be a defined environment variable.
# Currently, mine is set to /opt/ros/groovy.
# See http://ros.org/ to see what it takes to get ROS installed.

ROS_INCLUDES := \
    -I$(ROS_ROOT)/include

ROS_LIBRARIES_DIRECTORY := \
    $(ROS_ROOT)/lib

POINT_GREY_INCLUDES := \
    -I/usr/include/flycapture \

# The following warning options tend to work for both C and C++:
MIXED_WARNING_OPTIONS := \
    -Wreturn-type \
    -Wno-missing-braces \
    -Wextra \
    -Wno-missing-field-initializers \
    -Wformat=2 \
    -Wswitch-default \
    -Wcast-align \
    -Wpointer-arith \
    -Winline \
    -Wundef \
    -Wshadow \
    -Wunreachable-code \
    -Wlogical-op \
    -Wfloat-equal \
    -Wstrict-aliasing=2 \
    -Wstrict-overflow=5 \

# The following warning options tend not to work for C++; they do work for C:
C_ONLY_WARNING_OPTIONS := \
     ${MIXED_WARNING_OPTIONS} \
    -Wbad-function-cast \
    -Wstrict-prototypes \
    -Wold-style-definition \
    -Werror \

# The following warning options should be turned on, but are causing problems:
C_NOT_WORKING_RIGHT := \
    -Wcast-qual \

# The following options are passed to the compiler.
# Warnings are done separately:
CC_OPTIONS := \
    -DPTGREY=1 \
    ${POINT_GREY_INCLUDES} \
    ${ROS_INCLUDES} \
    -L $(ROS_LIBRARIES_DIRECTORY) \
    -std=c11 \
    -g \
    -MMD \

# The following options are used sometimes:
OTHER_CC_OPTIONS := \
    -O3 \
    -pg \

# Use different warning options depending if C++ is encountered or not:
CC := gcc 
CC_MIXED :=  $(CC) ${CC_OPTIONS} ${MIXED_WARNING_OPTIONS}
CC_C_ONLY := $(CC) ${CC_OPTIONS} ${C_ONLY_WARNING_OPTIONS}

COMMON_O_FILES := \
    Bounding_Box.o \
    Character.o \
    CRC.o \
    Double.o \
    FEC.o \
    File.o \
    Float.o \
    Integer.o \
    List.o \
    Logical.o \
    Memory.o \
    String.o \
    SVG.o \
    Table.o \
    Unsigned.o \

DEMO_O_FILES := \
    Arc.o \
    Camera_Tag.o \
    CV.o \
    Demo.o \
    Fiducials.o \
    Map.o \
    Tag.o \
    High_GUI2.o \

FLY_CAPTURE_O_FILES := \
    CV.o \
    FC2.o \
    Fly_Capture.o \
    High_GUI2.o \

FLYCAPTURE2TEST_O_FILES := \
    FC2.o \
    FlyCapture2Test.o \

MAP_TEST_O_FILES := \
    Arc.o \
    CV.o \
    Camera_Tag.o \
    Map.o \
    Map_Test.o \
    Tag.o \

TAGS_O_FILES := \
    Tags.o \

VIDEO_CAPTURE_O_FILES := \
    CV.o \
    FC2.o \
    High_GUI2.o \
    Video_Capture.o \

ALL_O_FILES := \
    ${COMMON_O_FILES} \
    ${DEMO_O_FILES} \
    ${FLYCAPTURE2TEST_O_FILES} \
    ${MAP_TEST_O_FILES} \
    ${TAGS_O_FILES} \
    ${VIDEO_CAPTURE_O_FILES} \

ALL_C_BACKUPS := ${ALL_O_FILES:%.o=%.c~}
ALL_D_FILES := ${ALL_O_FILES:%.o=%.d}
ALL_H_BACKUPS := ${ALL_O_FILES:%.o=%.h~}

POINT_GREY_LIBRARIES := \
    -lflycapture-c \
    -lflycapture \

OPENCV_LIBRARIES := \
    -lopencv_core \
    -lopencv_imgproc \
    -lopencv_calib3d \
    -lopencv_highgui \
    -lm \

PROGRAMS := \
    Demo \
    Fly_Capture \
    FlyCapture2Test \
    Map_Test \
    Tags \
    Video_Capture \

all: ${PROGRAMS}

# Program linking:

Tags: ${COMMON_O_FILES} ${TAGS_O_FILES}
	${CC_C_ONLY} -o $@ ${TAGS_O_FILES} \
	  ${COMMON_O_FILES} -lm

Demo: ${COMMON_O_FILES} ${DEMO_O_FILES}
	${CC_C_ONLY} -o $@ ${DEMO_O_FILES} \
	  ${COMMON_O_FILES} ${OPENCV_LIBRARIES} -lm

Fly_Capture: ${COMMON_O_FILES} ${FLY_CAPTURE_O_FILES}
	${CC_MIXED} -o $@ ${FLY_CAPTURE_O_FILES} \
	  ${COMMON_O_FILES} ${OPENCV_LIBRARIES} ${POINT_GREY_LIBRARIES} -lm

FlyCapture2Test: ${FLYCAPTURE2TEST_O_FILES}
	${CC_MIXED} -o $@ ${FLYCAPTURE2TEST_O_FILES} \
	  ${COMMON_O_FILES} ${POINT_GREY_LIBRARIES}

Map_Test: ${COMMON_O_FILES} ${MAP_TEST_O_FILES}
	${CC_C_ONLY} -o $@ ${MAP_TEST_O_FILES} \
	  ${COMMON_O_FILES} ${OPENCV_LIBRARIES} -lm

Video_Capture: ${COMMON_O_FILES} ${VIDEO_CAPTURE_O_FILES}
	${CC_MIXED} -o $@ ${VIDEO_CAPTURE_O_FILES} \
	  ${COMMON_O_FILES} ${OPENCV_LIBRARIES} ${POINT_GREY_LIBRARIES} -lm


# Individual C file compilation:

FlyCapture2Test.o: FlyCapture2Test.c
	${CC_MIXED} -o $@ -c $<

FC2.o: FC2.c
	${CC_MIXED} -o $@ -c $<

Video_Capture.o: Video_Capture.c
	${CC_MIXED} -o $@ -c $<

Fly_Capture.o: Fly_Capture.c
	${CC_MIXED} -o $@ -c $<

clean:
	rm -f ${ALL_C_BACKUPS}
	rm -f ${ALL_D_FILES}
	rm -f ${ALL_H_BACKUPS}
	rm -f ${ALL_O_FILES}
	rm -f ${PROGRAMS}
	rm -f Makefile~ Tags.ezc~

%.o: %.c
	${CC_C_ONLY} -c -o $@ $<


# Used with gcc -MMD option to force automagic dependency checking:
-include ${ALL_D_FILES}


