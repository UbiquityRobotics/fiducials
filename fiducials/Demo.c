// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <stdlib.h>
#include "assert.h"
#include "sys/time.h"

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
#include "Map.h"
#include "String.h"
#include "Unsigned.h"

int main(int arguments_size, char * arguments[]) {
    struct timeval start_time_value_struct;    
    struct timeval end_time_value_struct;    
    struct timeval difference_time_value_struct;    
    Time_Value start_time_value = &start_time_value_struct;
    Time_Value end_time_value = &end_time_value_struct;
    Time_Value difference_time_value = &difference_time_value_struct;

    assert (gettimeofday(start_time_value, (struct timezone *)0) == 0);

    Logical image_log = (Logical)0;
    List /* <String> */ image_file_names =
      List__new("Demo:main:List__new:image_file_names");
    String lens_calibrate_file_name = (String)0;
    String log_file_name = (String)0;
    //File__format(stdout, "Hello\n");
    if (arguments_size <= 1) {
	File__format(stderr, "Usage: Demo lens.txt *.pnm\n");
    } else {
	for (Unsigned index = 1; index < arguments_size; index++) {
	    String argument = arguments[index];
	    Unsigned size = String__size(argument);
	    if (String__equal(argument, "--image_log")) {
		image_log = (Logical)1;
	    } else if (size > 4 && String__equal(argument + size - 4, ".txt")) {
		lens_calibrate_file_name = argument;
	    } else if (size > 4 && String__equal(argument + size - 4, ".log")) {
		log_file_name = argument;
	    } else if (size > 4 && String__equal(argument + size - 4, ".pnm")) {
		List__append(image_file_names,
		  argument, "Demo:main:List__append:image_file_names");
	    } else if (size > 4 && String__equal(argument + size - 4, ".chk")) {
		// Do nothing:
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
	Fiducials fiducials =
	  Fiducials__create(image, ".", lens_calibrate_file_name,
	  (void *)0, Fiducials__arc_announce,
	  Fiducials__location_announce, Fiducials__tag_announce,
	  log_file_name, "Demo_Map", "Tag_Heights.xml");
	fiducials->map->image_log = image_log;

	for (Unsigned index = 0; index < size; index++) {
	    String image_file_name = 
	      (String)List__fetch(image_file_names, index);
	    image = CV_Image__pnm_read(image_file_name);
	    Fiducials__image_set(fiducials, image);
	    Fiducials__process(fiducials);
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
	    Map__save(map);
	    List /*<Location>*/ locations = fiducials->locations;
	    File__format(stderr,
	      "Outputing %d locations\n", List__size(locations));
	    Map__svg_write(map, "Demo", locations);
	}

	// Release all the storage associated with *fiducials*:
	Fiducials__free(fiducials);

	// Some debugging code to make sure that we are reading/writing
	// map.xml files correctly:
	//fiducials =
	//  Fiducials__create(image, lens_calibrate_file_name, (void *)0,
	//  Fiducials__location_announce, Map__tag_announce, log_file_name,
	//  "Demo.xml");
	//fiducials->map->is_saved = (Logical)0;
	//fiducials->map->file_name = "Demo2.xml";
	//Fiducials__free(fiducials);
    }

    List__free(image_file_names);

    return 0;
}

