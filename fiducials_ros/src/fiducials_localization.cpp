// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include "assert.h"
#include "sys/time.h"
#include <unistd.h>

#include "Character.h"
#include "CV.h"
#include "Double.h"
#include "File.h"
#include "Fiducials.h"
#include "Float.h"
#include "High_GUI2.h"
#include "Integer.h"
#include "List.h"
#include "Logical.h"
#include "String.h"
#include "Unsigned.h"

#include "fiducials_rviz.h"

/// @brief Print out tag update information.
/// @param anounce_object is an opaque object from *Map*->*announce_object*.
/// @param id is the tag id.
/// @param x is the tag X location.
/// @param y is the tag Y location.
/// @param z is the tag Z location.
/// @param twist is the tag twist in radians.
/// @param dx is the tag size along the X axis (before twist).
/// @param dy is the tag size along the Y axis (before twist).
/// @param dz is the tag height in the Z axis.
///
/// *Map__tag_announce*() is called each time the map algorithm
/// updates the location or twist for a *tag*.

void Rviz__tag_announce(void *rviz, Integer id,
  Double x, Double y, Double z, Double twist, Double dx, Double dy, Double dz) {
    File__format(stderr, "Rviz__tag_announce:id=%d x=%f y=%f twist=%f\n",
      id, x, y, twist);
    Double scale = 1000.0;
    sendMarker(rviz, "fiducial_frame", id, x / scale, y / scale, z / scale,
      0.0, dx / scale, dy / scale, dz / scale);
}

void Rviz__location_announce(void *rviz, Integer id,
  Double x, Double y, Double z, Double bearing) {
    File__format(stderr, "Rviz__location_announce:id=%d x=%f y=%f bearing=%f\n",
      id, x, y, bearing * 180. / 3.1415926);
    Double scale = 1000.0;
    sendArrow(rviz, "fiducial_frame", id, x / scale, y / scale, z / scale,
        0.0, .2, .05, .05, bearing);
}

int main(int arguments_size, char * arguments[]) {
    struct timeval start_time_value_struct;    
    struct timeval end_time_value_struct;    
    struct timeval difference_time_value_struct;    
    Time_Value start_time_value = &start_time_value_struct;
    Time_Value end_time_value = &end_time_value_struct;
    Time_Value difference_time_value = &difference_time_value_struct;

    assert (gettimeofday(start_time_value, (struct timezone *)0) == 0);

    List /* <String> */ image_file_names = List__new();
    String lens_calibrate_file_name = (String)0;
    //File__format(stdout, "Hello\n");
    if (arguments_size <= 1) {
        File__format(stderr, "Usage: Demo lens.txt *.pnm\n");
    } else {
        for (Unsigned index = 1; index < arguments_size; index++) {
            String argument = arguments[index];
            Unsigned size = String__size(argument);
            if (size > 4 && String__equal(argument + size - 4, ".txt")) {
                lens_calibrate_file_name = argument;
            } else if (size > 4 && String__equal(argument + size - 4, ".pnm")) {
                List__append(image_file_names, argument);
            } else {
                File__format(stderr, "Unrecoginized file '%s'\n", argument);
            }
        }
    }

    Unsigned size = List__size(image_file_names);
    if (size > 0) {
        String image_file_name0 = (String)List__fetch(image_file_names, 0);
        CV_Image image = (CV_Image)0;
        image = CV_Image__pnm_read(image_file_name0);
        assert (image != (CV_Image)0);
        void *rviz = initRviz(arguments_size, arguments, "Rviz_Demo");
        Fiducials fiducials =
          Fiducials__create(image, lens_calibrate_file_name,
          rviz, Rviz__location_announce, Rviz__tag_announce);
        Fiducials__tag_heights_xml_read(fiducials, "Tag_Heights.xml");

        for (Unsigned index = 0; index < size; index++) {
            String image_file_name = 
              (String)List__fetch(image_file_names, index);
            image = CV_Image__pnm_read(image_file_name);
            Fiducials__image_set(fiducials, image);
            Fiducials__process(fiducials);
            sleep(2);

            //float xpos = 0.0;
            //float ypos = 0.0;
            //float zpos = 0.0;
            //int id = 0;
            //File__format(stderr, "sent a frame\n");
            //sendMarker(rviz, "fiducial_frame", id, xpos, ypos, zpos);
        }

        assert (gettimeofday(end_time_value, (struct timezone *)0) == 0);

        Double start_time = (Double)start_time_value->tv_usec / 1000000.0;
        Double end_time =
          (Double)(end_time_value->tv_sec - start_time_value->tv_sec) +
          (Double)end_time_value->tv_usec / 1000000.0;
        Double time = end_time - start_time;
        Double frames_per_second = (Double)size / time;

        File__format(stderr, "%d frames / %f sec = %f Frame/sec\n",
          size, time, frames_per_second);

        if (size == 1) {
            Fiducials__image_show(fiducials, (Logical)1);
        } else {
            Map map = fiducials->map;
            Map__save(map, "Rviz_Demo.xml");
            List /*<Location>*/ locations = fiducials->locations;
            File__format(stderr,
              "Outputing %d locations\n", List__size(locations));
            Map__svg_write(map, "Demo", locations);
        }
    }


    return 0;
}

