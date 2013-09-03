// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

typedef struct Fiducials__Struct *Fiducials;

#include "Character.h"
#include "CV.h"
#include "Double.h"
#include "File.h"
#include "High_GUI2.h"
#include "Integer.h"
#include "Logical.h"
#include "String.h"
#include "Unsigned.h"

struct Fiducials__Struct {
    Logical blur;
    CV_Image debug_image;
    Unsigned debug_index;
    CV_Image edge_image;
    CV_Image gray_image;
    CV_Scalar green;
    CV_Point origin;
    CV_Image original_image;
    CV_Scalar red;
    CV_Memory_Storage storage;
};

extern Fiducials Fiducials__create(CV_Image original_image);
extern void Fiducials__image_show(Fiducials fiducials, Logical show);
extern Unsigned Fiducials__process(Fiducials fiducials);

Integer main(Unsigned arguments_size, String arguments[]) {
    File__format(stdout, "Hello\n");
    if (arguments_size <= 1) {
	File__format(stderr, "Usage: Demo *.tga\n");
    } else {
        String image_file_name = arguments[1];
	CV_Image original_image = (CV_Image)0;
	original_image = CV__tga_read(original_image, image_file_name);
	Fiducials fiducials = Fiducials__create(original_image);
	Fiducials__image_show(fiducials, (Logical)1);
    }
    return 0;
}


// This routinew will show {original_image} on the screen along
// with a primitive debugging interface to showing how the debugging
// is going.

void Fiducials__image_show(Fiducials fiducials, Logical show) {
    // Grab some values out of *fiduicals*:
    CV_Image debug_image = fiducials->debug_image;
    CV_Image gray_image = fiducials->gray_image;
    CV_Image original_image = fiducials->original_image;

    // Create the window we need:
    String window_name = "Example1";
    if (show) {
	CV__named_window(window_name, CV__window_auto_size);
    }

    // Processing *original_image* with different options
    // for each time through the loop:
    Unsigned debug_index = 0;
    Unsigned previous_debug_index = debug_index;
    Logical done = (Logical)0;
    while (!done) {
        // Process {gray_image}; a debug image lands in {debug_image}:
	Fiducials__process(fiducials);

	// Display either *original_image* or *debug_image*:
	if (show) {
	    CV__show_image(window_name, debug_image);
	}

	// Get a *control_character* from the user:
	Character control_character = '\0';
	if (show) {
	    control_character = (Character)(CV__wait_key(0) & 0xff);
	}

	// Dispatch on *control_character*:
	switch (control_character) {
	  case '\33':
	    //# Exit program:
	    done = (Logical)1;
	    File__format(stderr, "done\n");
	    break;
	  case '+':
	    //# Increment {debug_index}:
	    debug_index += 1;
	    break;
	  case '-':
	    // Decrement {debug_index}:
	    if (debug_index > 0) {
		debug_index -= 1;
	    }
	    break;
	  case '<':
	    // Set {debug_index} to beginning:
	    debug_index = 0;
	    break;
	  case '>':
	    // Set {debug_index} to end:
	    debug_index = 100;
	    break;
	  case 'b':
	    // Toggle image blur:
	    fiducials->blur = !fiducials->blur;
	    File__format(stderr, "blur = %d\n", fiducials->blur);
	    break;
	  default:
	    // Deal with unknown {control_character}:
	    if ((Unsigned)control_character <= 127) {
		File__format(stderr,
		  "Unknown control character %d\n", control_character);
	    }
	    break;
	}

	// Update *debug_index* in *fiducials*:
	fiducials->debug_index = debug_index;

	// Show user *debug_index* if it has changed:
	if (debug_index != previous_debug_index) {
	  File__format(stderr,
	    "****************************debug_index = %d\n", debug_index);
	  previous_debug_index = debug_index;
	}
    }

    // Release storage:
    CV__release_image(original_image);
    if (show) {
	CV__destroy_window(window_name);
    }
}

Fiducials Fiducials__create(CV_Image original_image) {
    // Create *image_size*:
    Unsigned width = CV_Image__width_get(original_image);
    Unsigned height = CV_Image__height_get(original_image);
    CV_Size image_size = CV_Size__create(width, height);
    CV_Memory_Storage storage = CV_Memory_Storage__create(0);

    // Create and load *fiducials*:
    Fiducials fiducials = Memory__new(Fiducials);
    fiducials->blur = (Logical)1;
    fiducials->debug_image = CV_Image__create(image_size, CV__depth_8u, 3);
    fiducials->debug_index = 0;
    fiducials->edge_image = CV_Image__create(image_size, CV__depth_8u, 1);
    fiducials->gray_image = CV_Image__create(image_size, CV__depth_8u, 1);
    fiducials->green = CV_Scalar__rgb(0.0, 255.0, 0.0);
    fiducials->origin = CV_Point__create(0, 0);
    fiducials->original_image = original_image;
    fiducials->red = CV_Scalar__rgb(255.0, 0.0, 0.0);
    fiducials->storage = storage;
    return fiducials;
}

Unsigned Fiducials__process(Fiducials fiducials) {
    // Clear *storage*:
    CV_Memory_Storage storage = fiducials->storage;
    CV_Memory_Storage__clear(storage);

    // Grab some values from *fiducials*:
    CV_Image debug_image = fiducials->debug_image;
    Unsigned debug_index = fiducials->debug_index;
    CV_Image edge_image = fiducials->edge_image;
    CV_Image gray_image = fiducials->gray_image;
    CV_Image original_image = fiducials->original_image;

    // For *debug_level* 0, we show the original image in color:
    if (debug_index == 0) {
	CV_Image__copy(original_image, debug_image, (CV_Image)0);
    }

    // Convert from color to gray scale:
    Integer channels = CV_Image__channels_get(original_image);

    // Deal with *debug_index* 0:
    if (debug_index == 0) {
	if (channels == 3) {
	    // Original image is color, so a simple copy will work:
	    CV_Image__copy(original_image, debug_image, (CV_Image)0);
	} else if (channels == 1) {
	    // Original image is gray, so we have to convert back to "color":
	    CV_Image__convert_color(original_image,
	      debug_image, CV__gray_to_rgb);
	}
    }

    // Convert *original_image* to gray scale:
    if (channels == 3) {
        // Original image is color, so we need to convert to gray scale:
	CV_Image__convert_color(original_image, gray_image, CV__rgb_to_gray);
    } else if (channels == 1) {
	// Original image is gray, so a simple copy will work:
	CV_Image__copy(original_image, gray_image, (CV_Image)0);
    } else {
        assert(0);
    }

    // Show results of gray scale converion for *debug_index* 1:
    if (debug_index == 1) {
        CV_Image__convert_color(gray_image, debug_image, CV__gray_to_rgb);
    }
    
    // Perform Gaussian blur if requested:
    if (fiducials->blur) {
	CV_Image__smooth(gray_image, gray_image, CV__gaussian, 3, 0, 0.0, 0.0);
    }

    // Show results of Gaussian blur for *debug_index* 2:
    if (debug_index == 2) {
        CV_Image__convert_color(gray_image, debug_image, CV__gray_to_rgb);
    }

    // Perform adpative threshold:
    CV_Image__adaptive_threshold(gray_image, edge_image, 255.0,
      CV__adaptive_thresh_gaussian_c, CV__thresh_binary, 45, 5.0);

    // Show results of adaptive threshold for *debug_index* 3:
    if (debug_index == 3) {
        CV_Image__convert_color(edge_image, debug_image, CV__gray_to_rgb);
    }

    // Find the *edge_image* *contours*:
    CV_Point origin = fiducials->origin;
    Integer header_size = 128;
    CV_Sequence contours = CV_Image__find_contours(edge_image, storage,
      header_size, CV__retr_list, CV__chain_approx_simple, origin);
    if (contours == (CV_Sequence)0) {
	File__format(stderr, "no contours found\n");
    }

    // For *debug_index* 4, show the *edge_image* *contours*:
    if (debug_index == 4) {
	//File__format(stderr, "Draw red contours\n");
	CV_Scalar red = fiducials->red;
	CV_Image__convert_color(gray_image, debug_image, CV__gray_to_rgb);
	CV_Image__draw_contours(debug_image,
	  contours, red, red, 2, 2, 8, origin);
    }

    // For the remaining debug steps, we use the original *gray_image*:
    if (debug_index >= 5) {
	CV_Image__convert_color(gray_image, debug_image, CV__gray_to_rgb);
    }

    // Iterate over all of the countours:
    //big :@= map.big
    Unsigned contours_count = 0;
    for (CV_Sequence contour = contours; contour != (CV_Sequence)0;
      contour = CV_Sequence__next_get(contour)) {
	// Keep a count of total countours:
        contours_count += 1;
	//File__format(stderr, "contours_count=%d\n", contours_count);

	static CvSlice whole_sequence;
	CV_Slice CV__whole_seq = &whole_sequence;
	whole_sequence = CV_WHOLE_SEQ;

	// Perform a polygon approximation of {contour}:
	Integer arc_length =
	  (Integer)(CV_Sequence__arc_length(contour, CV__whole_seq, 1) * 0.02);
	CV_Sequence polygon_contour =
	  CV_Sequence__approximate_polygon(contour,
	  header_size, storage, CV__poly_approx_dp, arc_length, 0.0);
	if (debug_index == 5) {
	    //File__format(stderr, "Draw green contours\n");
	    CV_Scalar green = fiducials->green;
	    CV_Image__draw_contours(debug_image,
	      polygon_contour, green, green, 2, 2, 1, origin);
	}

	// Compute the contour area:
	Double exact_area =
	  CV_Sequence__contour_area(polygon_contour, CV__whole_seq, 0);
	Integer area = (Integer)exact_area;
	if (area < 0) {
	    area = -area;
	}

	// If we have a 4-sided polygon with an area greater than 500 square
	// pixels, we can explore to see if we have a tag:
	if (CV_Sequence__total_get(polygon_contour) == 4 && area > 500 &&
	  CV_Sequence__check_contour_convexity(polygon_contour)) {
	    // For debugging, display the polygons in red:
	    //File__format(stderr, "Have 4 sides > 500i\n");

	    if (debug_index == 6) {
		CV_Scalar red = fiducials->red;
		CV_Image__draw_contours(debug_image,
		  polygon_contour, red, red, 2, 2, 1, origin);
	    }
	}
    }

    return 0;
}
