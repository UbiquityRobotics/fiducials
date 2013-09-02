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
    -Wcast-qual \
    -Wshadow \
    -Wunreachable-code \
    -Wlogical-op \
    -Wfloat-equal \
    -Wstrict-aliasing=2 \
    -Wold-style-definition \
    -Werror \

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
    Integer.o \
    Logical.o \
    Memory.o \
    String.o \
    SVG.o \
    Unsigned.o \

DEMO_O_FILES := \
    CV.o \
    Demo.o \

TAGS_O_FILES := \
    Tags.o \

ALL_O_FILES := \
    ${COMMON_O_FILES} \
    ${TAGS_O_FILES} \

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
    Tags \

all: ${PROGRAMS}

Tags: ${COMMON_O_FILES} ${TAGS_O_FILES}
	$(CC) -o $@ ${TAGS_O_FILES} ${COMMON_O_FILES}

Demo: ${COMMON_O_FILES} ${DEMO_O_FILES}
	$(CC) -o $@ ${DEMO_O_FILES} ${COMMON_O_FILES} ${OPENCV_LIBRARIES}

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


