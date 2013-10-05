// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

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
#include "String.h"
#include "Unsigned.h"

Integer main(Unsigned argc, String argv[]) {
    struct timeval start_time_value_struct;    
    struct timeval end_time_value_struct;    
    struct timeval difference_time_value_struct;    
    Time_Value start_time_value = &start_time_value_struct;
    Time_Value end_time_value = &end_time_value_struct;
    Time_Value difference_time_value = &difference_time_value_struct;

    assert (gettimeofday(start_time_value, (struct timezone *)0) == 0);

    List /* <String> */ image_file_names = List__new();
    File__format(stdout, "Hello\n");
    if (argc <= 1) {
	File__format(stderr, "Usage: Demo *.pnm\n");
    } else {
	for (Unsigned index = 1; index < argc; index++) {
	    List__append(image_file_names, argv[index]);
	}
    }

    Unsigned size = List__size(image_file_names);
    if (size > 0) {
	String image_file_name0 = (String)List__fetch(image_file_names, 0);
	CV_Image image = (CV_Image)0;
	image = CV_Image__pnm_read(image_file_name0);
	assert (image != (CV_Image)0);
	Fiducials fiducials = Fiducials__create(image);

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
	}
    }


    return 0;
}

