# Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

C_WARNING_OPTIONS := \
    -Wreturn-type \
    -Wno-missing-braces \
    -Wextra \
    -Wno-missing-field-initializers \
    -Wformat=2 \
    -Wswitch-default \
    -Wcast-align \
    -Wpointer-arith \
    -Wbad-function-cast \
    -Wstrict-overflow=5 \
    -Wstrict-prototypes \
    -Winline \
    -Wundef \
    -Wshadow \
    -Wunreachable-code \
    -Wlogical-op \
    -Wfloat-equal \
    -Wstrict-aliasing=2 \
    -Wold-style-definition \
    -Werror \

NO_WORKING := \
    -Wcast-qual \

C_OPTIONS := \
    -std=c11 \
    -g \
    -MMD \
    ${C_WARNING_OPTIONS} \

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
    CV.o \
    Demo.o \
    Map.o \
    Tag.o \
    High_GUI2.o \

MAP_TEST_O_FILES := \
    Arc.o \
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
    -lm \
    -lopencv_core \
    -lopencv_imgproc \
    -lopencv_calib3d \
    -lopencv_highgui \

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
	$(CC) -o $@ ${COMMON_O_FILES} ${MAP_TEST_O_FILES} -lm

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


