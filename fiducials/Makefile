# Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

# ROS_ROOT needs to be a defined environment variable.
# Currently, mine is set to /opt/ros/groovy.
# See http://ros.org/ to see what it takes to get ROS installed.

$(warning This should now be built with catkin_make)

ROS_INCLUDE_DIR := $(ROS_ROOT)/include
ROS_LIB_DIR := $(ROS_ROOT)/lib

NOT_C_PLUS_PLUS := \
    -Wbad-function-cast \
    -Wstrict-prototypes \
    -Wold-style-definition \
    -Werror \

C_WARNING_OPTIONS := \
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
    ${NOT_C_PLUS_PLUS}

NO_WORKING := \
    -Wcast-qual \

C_OPTIONS := \
    -I$(ROS_INCLUDE_DIR) \
    -L $(ROS_LIB_DIR) \
    -std=c11 \
    -g \
    -MMD \
    ${C_WARNING_OPTIONS} \

OLD_C_OPTIONS := \
    -O3 \
    -pg \

CC := gcc ${C_OPTIONS}

COMMON_O_FILES := \
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
    Map.o \
    Tag.o \
    High_GUI2.o \

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
    High_GUI2.o \
    Video_Capture.o \

ALL_O_FILES := \
    ${COMMON_O_FILES} \
    ${DEMO_O_FILES} \
    ${MAP_TEST_O_FILES} \
    ${TAGS_O_FILES} \
    ${VIDEO_CAPTURE_O_FILES} \

ALL_C_BACKUPS := ${ALL_O_FILES:%.o=%.c~}
ALL_D_FILES := ${ALL_O_FILES:%.o=%.d}
ALL_H_BACKUPS := ${ALL_O_FILES:%.o=%.h~}

OPENCV_LIBRARIES := \
    -lopencv_core \
    -lopencv_imgproc \
    -lopencv_calib3d \
    -lopencv_highgui \
    -lm \

PROGRAMS := \
    Demo \
    Map_Test \
    Tags \
    Video_Capture \

all: ${PROGRAMS}

Tags: ${COMMON_O_FILES} ${TAGS_O_FILES}
	$(CC) -o $@ ${TAGS_O_FILES} ${COMMON_O_FILES} -lm

Demo: ${COMMON_O_FILES} ${DEMO_O_FILES}
	$(CC) -o $@ ${DEMO_O_FILES} ${COMMON_O_FILES} ${OPENCV_LIBRARIES} -lm

Map_Test: ${COMMON_O_FILES} ${MAP_TEST_O_FILES}
	$(CC) -o $@ ${MAP_TEST_O_FILES} \
	    ${COMMON_O_FILES} ${OPENCV_LIBRARIES} -lm

Video_Capture: ${COMMON_O_FILES} ${VIDEO_CAPTURE_O_FILES}
	$(CC) -o $@ ${VIDEO_CAPTURE_O_FILES} \
	    ${COMMON_O_FILES} ${OPENCV_LIBRARIES} -lm

clean:
	rm -f ${ALL_C_BACKUPS}
	rm -f ${ALL_D_FILES}
	rm -f ${ALL_H_BACKUPS}
	rm -f ${ALL_O_FILES}
	rm -f ${PROGRAMS}
	rm -f Makefile~ Tags.ezc~

%.o: %c
	$(CC) -c -o $@ $<


# Used with gcc -MMD option to force automagic dependency checking:
-include ${ALL_D_FILES}


