// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

typedef struct Fiducials__Struct *Fiducials;

#include "assert.h"
#include "sys/time.h"

#include "Camera_Tag.h"
#include "Character.h"
#include "CRC.h"
#include "CV.h"
#include "Double.h"
#include "File.h"
#include "FEC.h"
#include "Fiducials.h"
#include "Float.h"
#include "High_GUI2.h"
#include "Integer.h"
#include "List.h"
#include "Logical.h"
#include "Map.h"
#include "String.h"
#include "Tag.h"
#include "Unsigned.h"

void Fiducials__location_announce(void *object, Integer id,
  Double x, Double y, Double z, Double bearing) {
    File__format(stderr,
      "Location: id=%d x=%f y=%f bearing=%f\n", id, x, y, bearing);
}

void Fiducials__image_set(Fiducials fiducials, CV_Image image) {
    fiducials->original_image = image;
}

// This routine will show {original_image} on the screen along
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
	    CV_Image__show(debug_image, window_name);
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
	  case 'f':
	    // Toggle image blur:
	    fiducials->y_flip = !fiducials->y_flip;
	    File__format(stderr, "y_flip = %d\n", fiducials->y_flip);
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

Fiducials Fiducials__create(
  CV_Image original_image, String lens_calibrate_file_name,
  void *announce_object,
  Fiducials_Location_Announce_Routine location_announce_routine,
  Fiducials_Tag_Announce_Routine tag_announce_routine) {
    // Create *image_size*:
    Unsigned width = CV_Image__width_get(original_image);
    Unsigned height = CV_Image__height_get(original_image);
    CV_Size image_size = CV_Size__create(width, height);
    CV_Memory_Storage storage = CV_Memory_Storage__create(0);

    File__format(stderr, "CV width=%d CV height = %d\n", width, height);

    Integer term_criteria_type =
      CV__term_criteria_iterations | CV__term_criteria_eps;

    // The north/west/south/east mappings must reside in static
    // memory rather than on the stack:

    static Logical north_mapping[64] = {
        //corner1              corner0
	 0,  1,  2,  3,  4,  5,  6,  7,
	 8,  9, 10, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 50, 51, 52, 53, 54, 55,
	56, 57, 58, 59, 60, 61, 62, 63,
	//corner2              corner3
    };

    static Logical west_mapping[64] = {
	//corner1              corner0
	 7, 15, 23, 31, 39, 47, 55, 63,
	 6, 14, 22, 30, 38, 46, 54, 62,
	 5, 13, 21, 29, 37, 45, 53, 61,
	 4, 12, 20, 28, 36, 44, 52, 60,
	 3, 11, 19, 27, 35, 43, 51, 59,
	 2, 10, 18, 26, 34, 42, 50, 58,
	 1,  9, 17, 25, 33, 41, 49, 57,
	 0,  8, 16, 24, 32, 40, 48, 56,
	//corner2              corner3
    };

    static Logical south_mapping[64] = {
	//corner1              corner0
	63, 62, 61, 60, 59, 58, 57, 56,
	55, 54, 53, 52, 51, 50, 49, 48,
	47, 46, 45, 44, 43, 42, 41, 40,
	39, 38, 37, 36, 35, 34, 33, 32,
	31, 30, 29, 28, 27, 26, 25, 24,
	23, 22, 21, 20, 19, 18, 17, 16,
	15, 14, 13, 12, 11, 10,  9,  8,
	 7,  6,  5,  4,  3,  2,  1,  0,
	//corner2              corner3
    };

    static Logical east_mapping[64] = {
	//corner1              corner0
	56, 48, 40, 32, 24, 16,  8,  0,
	57, 49, 41, 33, 25, 17,  9,  1,
	58, 50, 42, 34, 26, 18, 10,  2,
	59, 51, 43, 35, 27, 19, 11,  3,
	60, 52, 44, 36, 28, 20, 12,  4,
	61, 53, 45, 37, 29, 21, 13,  5,
	62, 54, 46, 38, 30, 22, 14,  6,
	63, 55, 47, 39, 31, 23, 15,  7,
	//corner2              corner3
    };

    static Logical north_mapping_flipped[64] = {
	//corner1              corner0
	 7,  6,  5,  4,  3,  2,  1,  0,
	15, 14, 13, 12, 11, 10,  9,  8,
	23, 22, 21, 20, 19, 18, 17, 16,
	31, 30, 29, 28, 27, 26, 25, 24,
	39, 38, 37, 36, 35, 34, 33, 32,
	47, 46, 45, 44, 43, 42, 41, 40,
	55, 54, 53, 52, 51, 50, 49, 48,
	63, 62, 61, 60, 59, 58, 57, 56,
	 //corner2              corner3
    };

    static Logical west_mapping_flipped[64] = {
	//corner1              corner0
	63, 55, 47, 39, 31, 23, 15, 7,
	62, 54, 46, 38, 30, 22, 14, 6,
	61, 53, 45, 37, 29, 21, 13, 5,
	60, 52, 44, 36, 28, 20, 12, 4,
	59, 51, 43, 35, 27, 19, 11, 3,
	58, 50, 42, 34, 26, 18, 10, 2,
	57, 49, 41, 33, 25, 17,  9, 1,
	56, 48, 40, 32, 24, 16,  8, 0,
	//corner2              corner3
    };

    static Logical south_mapping_flipped[64] = {
	//corner1              corner0
	56, 57, 58, 59, 60, 61, 62, 63, 
	48, 49, 50, 51, 52, 53, 54, 55,
	40, 41, 42, 43, 44, 45, 46, 47,
	32, 33, 34, 35, 36, 37, 38, 39,
	24, 25, 26, 27, 28, 29, 30, 31,
	16, 17, 18, 19, 20, 21, 22, 23,
	 8,  9, 10, 11, 12, 13, 14, 15, 
	 0,  1,  2,  3,  4,  5,  6,  7,
	//corner2              corner3
    };

    static Logical east_mapping_flipped[64] = {
	//corner1              corner0
	 0,  8, 16, 24, 32, 40, 48, 56,
	 1,  9, 17, 25, 33, 41, 49, 57,
	 2, 10, 18, 26, 34, 42, 50, 58,
	 3, 11, 19, 27, 35, 43, 51, 59,
	 4, 13, 20, 28, 36, 44, 52, 60,
	 5, 13, 21, 29, 37, 45, 53, 61,
	 6, 14, 22, 30, 38, 46, 54, 62,
	 7, 15, 23, 31, 39, 47, 55, 63,
	 //corner2              corner3
    };

//    static Logical north_mapping_flipped[64] = {
//        //corner1              corner0
//	56, 57, 58, 59, 60, 61, 62, 63,
//	48, 49, 50, 51, 52, 53, 54, 55,
//	40, 41, 42, 43, 44, 45, 46, 47,
//	32, 33, 34, 35, 36, 37, 38, 39,
//	24, 25, 26, 27, 28, 29, 30, 31,
//	16, 17, 18, 19, 20, 21, 22, 23,
//	 8,  9, 10, 11, 12, 13, 14, 15,
//	 0,  1,  2,  3,  4,  5,  6,  7,
//	//corner2              corner3
//    };
//
//    static Logical west_mapping_flipped[64] = {
//	//corner1              corner0
//	 0,  8, 16, 24, 32, 40, 48, 56,
//	 1,  9, 17, 25, 33, 41, 49, 57,
//	 2, 10, 18, 26, 34, 42, 50, 58,
//	 3, 11, 19, 27, 35, 43, 51, 59,
//	 4, 12, 20, 28, 36, 44, 52, 60,
//	 5, 13, 21, 29, 37, 45, 53, 61,
//	 6, 14, 22, 30, 38, 46, 54, 62,
//	 7, 15, 23, 31, 39, 47, 55, 63,
//	//corner2              corner3
//    };
//
//    static Logical south_mapping_flipped[64] = {
//	//corner1              corner0
//	 7,  6,  5,  4,  3,  2,  1,  0,
//	15, 14, 13, 12, 11, 10,  9,  8,
//	23, 22, 21, 20, 19, 18, 17, 16,
//	31, 30, 29, 28, 27, 26, 25, 24,
//	39, 38, 37, 36, 35, 34, 33, 32,
//	47, 46, 45, 44, 43, 42, 41, 40,
//	55, 54, 53, 52, 51, 50, 49, 48,
//	63, 62, 61, 60, 59, 58, 57, 56,
//	//corner2              corner3
//    };
//
//    static Logical east_mapping_flipped[64] = {
//	//corner1              corner0
//	63, 55, 47, 39, 31, 23, 15,  7,
//	62, 54, 46, 38, 30, 22, 14,  6,
//	61, 53, 45, 37, 29, 21, 13,  5,
//	60, 52, 44, 36, 28, 20, 12,  4,
//	59, 51, 43, 35, 27, 19, 11,  3,
//	58, 50, 42, 34, 26, 18, 10,  2,
//	57, 49, 41, 33, 25, 17,  9,  1,
//	56, 48, 40, 32, 24, 16,  8,  0,
//	//corner2              corner3
//    };

    // The north/west/south/east mappings must reside in static
    // memory rather than on the stack:

    static Logical *mappings[4] = {
	&north_mapping_flipped[0],
	&west_mapping_flipped[0],
	&south_mapping_flipped[0],
	&east_mapping_flipped[0],
    };

    //for (Unsigned index = 0; index < 4; index++) {
    //	File__format(stderr, "mappings[%d]=0x%x\n", index, mappings[index]);
    //}

    CV_Image map_x = (CV_Image)0;
    CV_Image map_y = (CV_Image)0;
    if (lens_calibrate_file_name != (String)0) {
	assert (CV__undistortion_setup(
	  lens_calibrate_file_name, width, height, &map_x, &map_y) == 0);
    }

    // Create and load *fiducials*:
    Fiducials fiducials = Memory__new(Fiducials);
    fiducials->announce_object = announce_object;
    fiducials->blue = CV_Scalar__rgb(0.0, 0.0, 1.0);
    fiducials->blur = (Logical)1;
    fiducials->camera_tags = List__new(); // <Camera_Tag>
    fiducials->camera_tags_pool = List__new(); // <Camera_Tag>
    fiducials->corners = CV_Point2D32F_Vector__create(4);
    fiducials->cyan = CV_Scalar__rgb(0.0, 1.0, 1.0);
    fiducials->debug_image = CV_Image__create(image_size, CV__depth_8u, 3);
    fiducials->debug_index = 0;
    fiducials->edge_image = CV_Image__create(image_size, CV__depth_8u, 1);
    fiducials->fec = FEC__create(8, 4, 4);
    fiducials->gray_image = CV_Image__create(image_size, CV__depth_8u, 1);
    fiducials->green = CV_Scalar__rgb(0.0, 255.0, 0.0);
    fiducials->location_announce_routine = location_announce_routine;
    fiducials->locations = List__new();
    fiducials->map = Map__new(announce_object, tag_announce_routine);
    fiducials->map_x = map_x;
    fiducials->map_y = map_y;
    fiducials->mappings = &mappings[0];
    fiducials->origin = CV_Point__create(0, 0);
    fiducials->original_image = original_image;
    fiducials->purple = CV_Scalar__rgb(255.0, 0.0, 255.0);
    fiducials->red = CV_Scalar__rgb(255.0, 0.0, 0.0);
    fiducials->references = CV_Point2D32F_Vector__create(8);
    fiducials->sample_points = CV_Point2D32F_Vector__create(64);
    fiducials->size_5x5 = CV_Size__create(5, 5);
    fiducials->size_m1xm1 = CV_Size__create(-1, -1);
    fiducials->storage = storage;
    fiducials->temporary_gray_image =
      CV_Image__create(image_size, CV__depth_8u, 1);
    fiducials->weights_index = 0;
    fiducials->term_criteria = 
      CV_Term_Criteria__create(term_criteria_type, 5, 0.2);
    fiducials->y_flip = (Logical)0;

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
    CV_Image temporary_gray_image = fiducials->temporary_gray_image;
    CV_Image original_image = fiducials->original_image;
    List /*<Location>*/ locations = fiducials->locations;

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
    
    // Preform undistort if available:
    if (fiducials->map_x != (CV_Image)0) {
	Integer flags = CV_INTER_NN | CV_WARP_FILL_OUTLIERS;
	CV_Image__copy(gray_image, temporary_gray_image, (CV_Image)0);
	CV_Image__remap(temporary_gray_image, gray_image,
	  fiducials->map_x, fiducials->map_y, flags, fiducials->black);
    }

    // Show results of undistort:
    if (debug_index == 2) {
	CV_Image__convert_color(gray_image, debug_image, CV__gray_to_rgb);
    }

    // Perform Gaussian blur if requested:
    if (fiducials->blur) {
	CV_Image__smooth(gray_image, gray_image, CV__gaussian, 3, 0, 0.0, 0.0);
    }

    // Show results of Gaussian blur for *debug_index* 2:
    if (debug_index == 3) {
        CV_Image__convert_color(gray_image, debug_image, CV__gray_to_rgb);
    }

    // Perform adpative threshold:
    CV_Image__adaptive_threshold(gray_image, edge_image, 255.0,
      CV__adaptive_thresh_gaussian_c, CV__thresh_binary, 45, 5.0);

    // Show results of adaptive threshold for *debug_index* 3:
    if (debug_index == 4) {
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
    if (debug_index == 5) {
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

    // Iterate over all of the *contours*:
    List /* <Camera_Tag> */ camera_tags = fiducials->camera_tags;
    Map map = fiducials->map;
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
	if (debug_index == 6) {
	    //File__format(stderr, "Draw green contours\n");
	    CV_Scalar green = fiducials->green;
	    CV_Image__draw_contours(debug_image,
	      polygon_contour, green, green, 2, 2, 1, origin);
	}

	// If we have a 4-sided polygon with an area greater than 500 square
	// pixels, we can explore to see if we have a tag:
	if (CV_Sequence__total_get(polygon_contour) == 4 &&
	  fabs(CV_Sequence__contour_area(polygon_contour,
	  CV__whole_seq, 0)) > 500.0 &&
	  CV_Sequence__check_contour_convexity(polygon_contour)) {
	    // For debugging, display the polygons in red:
	    //File__format(stderr, "Have 4 sides > 500i\n");

	    // Just show the fiducial outlines for *debug_index* of 6:
	    if (debug_index == 7) {
		CV_Scalar red = fiducials->red;
		CV_Image__draw_contours(debug_image,
		  polygon_contour, red, red, 2, 2, 1, origin);
	    }

	    // Copy the 4 corners from {poly_contour} to {corners}:
	    CV_Point2D32F_Vector corners = fiducials->corners;
	    for (Unsigned index = 0; index < 4; index++) {
		CV_Point2D32F corner =
		  CV_Point2D32F_Vector__fetch1(corners, index);
		CV_Point point =
		  CV_Sequence__point_fetch1(polygon_contour, index);
		CV_Point2D32F__point_set(corner, point);

		if (debug_index == 7) {
		    //File__format(stderr,
		    //  "point[%d] x:%f y:%f\n", index, point->x, point->y);
		}
	    }

	    // Now find the sub pixel corners of {corners}:
	    CV_Image__find_corner_sub_pix(gray_image, corners, 4,
	      fiducials->size_5x5, fiducials->size_m1xm1,
	      fiducials->term_criteria);

	    // Ensure that the corners are in a counter_clockwise direction:
	    CV_Point2D32F_Vector__corners_normalize(corners);

	    // For debugging show the 4 corners of the possible tag where
	    //corner0=red, corner1=green, corner2=blue, corner3=purple:
	    if (debug_index == 8) {
		for (Unsigned index = 0; index < 4; index++) {
		    CV_Point point =
		      CV_Sequence__point_fetch1(polygon_contour, index);
		    Integer x = CV_Point__x_get(point);
		    Integer y = CV_Point__y_get(point);
		    CV_Scalar color = (CV_Scalar)0;
		    String text = (String)0;
		    switch (index) {
		      case 0:
			color = fiducials->red;
			text = "red";
			break;
		      case 1:
			color = fiducials->green;
			text = "green";
			break;
		      case 2:
			color = fiducials->blue;
			text = "blue";
			break;
		      case 3:
			color = fiducials->purple;
			text = "purple";
			break;
		      default:
			assert(0);
		    }
		    CV_Image__cross_draw(debug_image, x, y, color);
		    File__format(stderr,
		      "poly_point[%d]=(%d:%d) %s\n", index, x, y, text);
		}
	    }

	    // Compute the 8 reference points for deciding whether the
	    // polygon is "tag like" in its borders:
	    CV_Point2D32F_Vector references =
	      Fiducials__references_compute(fiducials, corners);

	    // Now sample the periphery of the tag and looking for the
	    // darkest white value (i.e. minimum) and the lightest black
	    // value (i.e. maximum):
	    //Integer white_darkest =
	    //  CV_Image__points_minimum(gray_image, references, 0, 3);
	    //Integer black_lightest =
	    //  CV_Image__points_maximum(gray_image, references, 4, 7);
	    Integer white_darkest =
	      Fiducials__points_minimum(fiducials, references, 0, 3);
	    Integer black_lightest =
	      Fiducials__points_maximum(fiducials, references, 4, 7);

	    // {threshold} should be smack between the two:
	    Integer threshold = (white_darkest + black_lightest) / 2;
	    
	    // For debugging, show the 8 points that are sampled around the
	    // the tag periphery to even decide whether to do further testing.
	    // Show "black" as green crosses, and "white" as green crosses:
	    if (debug_index == 9) {
		CV_Scalar red = fiducials->red;
		CV_Scalar green = fiducials->green;
		for (Unsigned index = 0; index < 8; index++) {
		    CV_Point2D32F reference =
		      CV_Point2D32F_Vector__fetch1(references, index);
		    Integer x = CV__round(CV_Point2D32F__x_get(reference));
		    Integer y = CV__round(CV_Point2D32F__y_get(reference));
		    //Integer value =
		    //  CV_Image__point_sample(gray_image, reference);
		    Integer value =
		      Fiducials__point_sample(fiducials, reference);
		    CV_Scalar color = red;
		    if (value < threshold) {
		        color = green;
		    }
		    CV_Image__cross_draw(debug_image, x, y, color);
		    File__format(stderr, "ref[%d:%d]:%d\n", x, y, value);
		}
	    }

	    // If we have enough contrast keep on trying for a tag match:
	    if (black_lightest < white_darkest) {
		// We have a tag to try:

		// Now it is time to read all the bits of the tag out:
		CV_Point2D32F_Vector sample_points = fiducials->sample_points;

		// Now compute the locations to sample for tag bits:
		Fiducials__sample_points_compute(corners, sample_points);

		// Extract all 64 tag bit values:
		Logical *tag_bits = &fiducials->tag_bits[0];
		for (Unsigned index = 0; index < 64; index++) {
		    // Grab the pixel value and convert into a {bit}:
		    CV_Point2D32F sample_point =
		      CV_Point2D32F_Vector__fetch1(sample_points, index);
		    //Integer value =
		    //  CV_Image__point_sample(gray_image, sample_point);
		    Integer value =
		      Fiducials__point_sample(fiducials, sample_point);
		    Logical bit = (value < threshold);
		    tag_bits[index] = bit;

		    // For debugging:
		    if (debug_index == 10) {
			CV_Scalar red = fiducials->red;
			CV_Scalar green = fiducials->green;
			CV_Scalar cyan = fiducials->cyan;
			CV_Scalar blue = fiducials->blue;

			// Show white bits as {red} and black bits as {green}:
			CV_Scalar color = red;
			if (bit) {
			    color = green;
			}

			// Show where bit 0 and 7 are:
			//if (index == 0) {
			//    // Bit 0 is {cyan}:
			//    color = cyan;
			//}
			//if (index == 7) {
			//    // Bit 7 is {blue}:
			//    color = blue;
			//}

			// Now splat a cross of {color} at ({x},{y}):
			Integer x =
			  CV__round(CV_Point2D32F__x_get(sample_point));
			Integer y =
			  CV__round(CV_Point2D32F__y_get(sample_point));
			CV_Image__cross_draw(debug_image, x, y, color);
		    }
		}

		//tag_bits :@= extractor.tag_bits
		//bit_field :@= extractor.bit_field
		//tag_bytes :@= extractor.tag_bytes

		// Now we iterate through the 4 different mapping
		// orientations to see if any one of the 4 mappings match:
		Logical **mappings = fiducials->mappings;
		Unsigned mappings_size = 4;
		for (Unsigned direction_index = 0;
		  direction_index < mappings_size; direction_index++) {
		    // Grab the mapping:
		    Logical *mapping = mappings[direction_index];
		    //File__format(stderr,
		    //  "mappings[%d]:0x%x\n", direction_index, mapping);


		    Logical mapped_bits[64];
		    for (Unsigned i = 0; i < 64; i++) {
			 mapped_bits[mapping[i]] = tag_bits[i];
		    }

		    // Fill in tag bytes;
		    Unsigned tag_bytes[8];
		    for (Unsigned i = 0; i < 8; i++) {
			Unsigned byte = 0;
			for (Unsigned j = 0; j < 8; j++) {
			    if (mapped_bits[(i<<3) + j]) {
				//byte |= 1 << j;
				byte |= 1 << (7 - j);
			    }
			}
			tag_bytes[i] = byte;
		    }
		    if (debug_index == 11) {
			File__format(stderr, "dir=%d Tag[0]=0x%x Tag[1]=0x%x\n",
			  direction_index, tag_bytes[0], tag_bytes[1]);
		    }

		    // Now we need to do some FEC (Forward Error Correction):
		    FEC fec = fiducials->fec;
		    if (FEC__correct(fec, tag_bytes, 8)) {
			// We passed FEC:
			if (debug_index == 11) {
			    File__format(stderr, "FEC correct\n");
			}

			// Now see if the two CRC's match:
			Unsigned computed_crc = CRC__compute(tag_bytes, 2);
			Unsigned tag_crc = (tag_bytes[3] << 8) | tag_bytes[2];
			if (computed_crc = tag_crc) {
			    // Yippee!!! We have a tag:
			    // Compute {tag_id} from the the first two bytes
			    // of {tag_bytes}:
			    Unsigned tag_id =
			      (tag_bytes[1] << 8) | tag_bytes[0];

			    if (debug_index == 11) {
				File__format(stderr,
				  "CRC correct, Tag=%d\n", tag_id);
			    }

			    // Allocate a *camera_tag*:
			    List /* <Camera_Tag> */ camera_tags_pool =
			      fiducials->camera_tags_pool;
			    Camera_Tag camera_tag = (Camera_Tag)0;
			    if (List__size(camera_tags_pool) == 0) {
			 	// *camera_tags_pool* is empty;
				// allocate a new one:
				camera_tag = Camera_Tag__new();
			    } else {
				camera_tag =
				  (Camera_Tag)List__pop(camera_tags_pool);
			    }

			    // Load up *camera_tag* to get center, twist, etc.:
			    Tag tag = Map__tag_lookup(map, tag_id);
			    if (debug_index == 11) {
			        Camera_Tag__initialize(camera_tag, tag,
				  direction_index, corners, debug_image);
			    } else {
			        Camera_Tag__initialize(camera_tag, tag,
				  direction_index, corners, (CV_Image)0);
			    }

			    // Record the maximum *camera_diagonal*:
			    Double camera_diagonal = camera_tag->diagonal;
			    Double diagonal =
			       camera_diagonal * tag->distance_per_pixel;
			    if (diagonal  > tag->diagonal) {
				tag->diagonal = diagonal;
			    }

			    // Append *camera_tag* to *camera_tags*:
			    List__append(camera_tags, (Memory)camera_tag);
			    //File__format(stderr,
			    //  "Found %d\n", camera_tag->tag->id);
			}
		    }
		}
	    }
	}
    }

    // Just for consistency sort *camera_tags*:
    List__sort(camera_tags, (List__Compare__Routine)Camera_Tag__compare);

    // Sweep through all *camera_tag* pairs to generat associated *Arc*'s:
    Unsigned camera_tags_size = List__size(camera_tags);
    if (camera_tags_size >= 2) {
	// Iterate through all pairs, using a "triangle" scan:
	for (Unsigned tag1_index = 0;
	  tag1_index < camera_tags_size - 1; tag1_index++) {
	    Camera_Tag camera_tag1 =
	      (Camera_Tag)List__fetch(camera_tags, tag1_index);
	
	    for (Unsigned tag2_index = tag1_index + 1;
	      tag2_index < camera_tags_size; tag2_index++) {
		Camera_Tag camera_tag2 =
		  (Camera_Tag)List__fetch(camera_tags, tag2_index);
		assert (camera_tag1->tag->id != camera_tag2->tag->id);
		Map__arc_update(map, camera_tag1, camera_tag2, gray_image);
	    }
	}
    }

    if (camera_tags_size > 0) {
	Double pi = 3.14159265358979323846264;
	Unsigned half_width = CV_Image__width_get(gray_image) >> 1;
	Unsigned half_height = CV_Image__height_get(gray_image) >> 1;
	File__format(stderr,
	  "half_width=d half_height=%d\n", half_width, half_height);
	Location closest_location = (Location)0;
	for (Unsigned index = 0; index < camera_tags_size; index++) {
	    Camera_Tag camera_tag = (Camera_Tag)List__fetch(camera_tags, index);
	    Tag tag = camera_tag->tag;
	    //File__format(stderr,
	    //  "[%d]:tag_id=%d tag_x=%f tag_y=%f tag_twist=%f\n",
	    //  index, tag->id, tag->x, tag->y, tag->twist * 180.0 / pi);
	    Double camera_dx = camera_tag->x - half_width;
	    Double camera_dy = camera_tag->y - half_height;
	    //File__format(stderr,
	    //  "[%d]:camera_dx=%f camera_dy=%f camera_twist=%f\n",
	    //  index, camera_dx, camera_dy, camera_tag->twist * 180.0 / pi);
	    Double polar_distance = Double__square_root(
	      camera_dx * camera_dx + camera_dy * camera_dy);
	    Double polar_angle = Double__arc_tangent2(camera_dy, camera_dx);
	    //File__format(stderr,
	    //  "[%d]:polar_distance=%f polar_angle=%f\n", index,
	    //  polar_distance, polar_angle * 180.0 / pi);
	    Double floor_distance = polar_distance * tag->distance_per_pixel;
	    Double angle =
	      Double__angle_normalize(polar_angle + pi - camera_tag->twist);
	    //File__format(stderr,
	    //  "[%d]:floor_distance=%f angle=%f\n",
	    //  index, floor_distance, angle * 180.0 / pi);
	    Double x = tag->x + floor_distance * Double__cosine(angle);
	    Double y = tag->y + floor_distance * Double__sine(angle);
	    Double bearing =
	      Double__angle_normalize(camera_tag->twist + tag->twist);
	    File__format(stderr,
	      "[%d]:x=%f:y=%f:bearing=%f\n", index, x, y, bearing * 180.0 / pi);
	    Unsigned location_index = List__size(locations);
	    Location location =
	      Location__create(x, y, bearing, floor_distance, location_index);
	    if (closest_location == (Location)0) {
		closest_location = location;
	    } else {
		if (location->goodness < closest_location->goodness) {
		    closest_location = location;
		}
	    }
	}
	if (closest_location != (Location)0) {
	    List__append(locations, (Memory)closest_location);
	    //File__format(stderr,
	    //  "Location: x=%f y=%f bearing=%f goodness=%f index=%d\n",
	    //  closest_location->x, closest_location->y,
	    //  closest_location->bearing * 180.0 / pi,
	    //  closest_location->goodness, closest_location->index);

	    // send rviz marker message here
	    fiducials->location_announce_routine(fiducials->announce_object, 0,
	      closest_location->x, closest_location->y, 0.0,
	      closest_location->bearing);
	}
	File__format(stderr, "\n");
    }

    // Clean out *camera_tags*:
    List__all_append(fiducials->camera_tags_pool, camera_tags);
    List__trim(camera_tags, 0);

    // Flip the debug image:
    if (fiducials->y_flip) {
	CV_Image__flip(debug_image, debug_image, 0);
    }

    // Update the map:
    Map__update(map);

    return 0;
}

Integer Fiducials__point_sample(Fiducials fiducials, CV_Point2D32F point) {
    // This routine will return a sample *fiducials* at *point*.

    // Get the (*x*, *y*) coordinates of *point*:
    Integer x = CV__round(CV_Point2D32F__x_get(point));
    Integer y = CV__round(CV_Point2D32F__y_get(point));
    CV_Image image = fiducials->gray_image;

    static Integer weights0[9] = {
      0,   0,  0,
      0, 100,  0,
      0,  0,   0};

    static Integer weights1[9] = {
       0,  15,  0,
      15,  40,  15,
       0,  15,  0};

    static Integer weights2[9] = {
       5,  10,  5,
      10,  40, 10,
       5,  10,  5};

    // Sample *image*:
    static Integer x_offsets[9] = {
      -1,  0,  1,
      -1,  0,  1,
      -1,  0,  1};
    static Integer y_offsets[9] = {
      -1, -1, -1,
       0,  0,  0,
       1,  1,  1};

    // Select sample *weights*:
    Integer *weights = (Integer *)0;
    switch (fiducials->weights_index) {
      case 1:
	weights = weights1;
	break;
      case 2:
	weights = weights2;
	break;
      default:
	weights = weights0;
	break;
    }

    // Interate across sample point;
    Integer numerator = 0;
    Integer denominator = 0;
    for (Integer index = 0; index < 9; index++) {
	Integer sample = CV_Image__gray_fetch(image,
	  x + x_offsets[index], y + y_offsets[index]);
	if (sample >= 0) {
	    Integer weight = weights[index];
	    numerator += sample * weight;
	    denominator += weight;
	}
    }

    // Compute *result* checking for divide by zero:
    Integer result = 0;
    if (denominator > 0) {
	result = numerator / denominator;
    }
    return result;
}

void CV_Point2D32F_Vector__corners_normalize(CV_Point2D32F_Vector corners) {
    // This routine will ensure that {corners} are ordered
    // in the counter-clockwise direction.

    if (CV_Point2D32F_Vector__is_clockwise(corners)) {
	// Extract two corners to be swapped:
	CV_Point2D32F corner1 = CV_Point2D32F_Vector__fetch1(corners, 1);
	CV_Point2D32F corner3 = CV_Point2D32F_Vector__fetch1(corners, 3);

	// Extract X and Y for both corners:
	Double x1 = CV_Point2D32F__x_get(corner1);
	Double y1 = CV_Point2D32F__y_get(corner1);
	Double x3 = CV_Point2D32F__x_get(corner3);
	Double y3 = CV_Point2D32F__y_get(corner3);

	// Swap contents of {corner1} and {corner3}:
	CV_Point2D32F__x_set(corner1, x3);
	CV_Point2D32F__y_set(corner1, y3);
	CV_Point2D32F__x_set(corner3, x1);
	CV_Point2D32F__y_set(corner3, y1);
    }
}

Logical CV_Point2D32F_Vector__is_clockwise(CV_Point2D32F_Vector corners) {

    // Extract the four corners:
    CV_Point2D32F corner0 = CV_Point2D32F_Vector__fetch1(corners, 0);
    CV_Point2D32F corner1 = CV_Point2D32F_Vector__fetch1(corners, 1);
    CV_Point2D32F corner2 = CV_Point2D32F_Vector__fetch1(corners, 2);

    // Extract X and Y for all four corners:
    Double x0 = CV_Point2D32F__x_get(corner0);
    Double y0 = CV_Point2D32F__y_get(corner0);
    Double x1 = CV_Point2D32F__x_get(corner1);
    Double y1 = CV_Point2D32F__y_get(corner1);
    Double x2 = CV_Point2D32F__x_get(corner2);
    Double y2 = CV_Point2D32F__y_get(corner2);

    // Create two vectors from the first two lines of the polygon:
    Double v1x = x1 - x0;
    Double v1y = y1 - y0;
    Double v2x = x2 - x1;
    Double v2y = y2 - y1;

    // Determine the sign of the Z coordinate of the cross product:
    Double z = v1x * v2y - v2x * v1y;

    // If the Z coordinate is negative, to reverse the sequence of the corners:
    return z < 0.0;
 }

CV_Point2D32F_Vector Fiducials__references_compute(
  Fiducials fiducials, CV_Point2D32F_Vector corners) {

    // This routine will use the 4 corner points in {corners} to
    // compute 8 reference points that returned.  The first 4 reference
    // points will be just outside of the quadrateral formed by {corners}
    // (i.e. the white bounding box) and the last 4 reference points are
    // on the inside (i.e. the black bounding box).

    // Extract the 8 references from {references}:
    CV_Point2D32F_Vector references = fiducials->references;
    CV_Point2D32F reference0 = CV_Point2D32F_Vector__fetch1(references, 0);
    CV_Point2D32F reference1 = CV_Point2D32F_Vector__fetch1(references, 1);
    CV_Point2D32F reference2 = CV_Point2D32F_Vector__fetch1(references, 2);
    CV_Point2D32F reference3 = CV_Point2D32F_Vector__fetch1(references, 3);
    CV_Point2D32F reference4 = CV_Point2D32F_Vector__fetch1(references, 4);
    CV_Point2D32F reference5 = CV_Point2D32F_Vector__fetch1(references, 5);
    CV_Point2D32F reference6 = CV_Point2D32F_Vector__fetch1(references, 6);
    CV_Point2D32F reference7 = CV_Point2D32F_Vector__fetch1(references, 7);

    // Extract the 4 corners from {corners}:
    CV_Point2D32F corner0 = CV_Point2D32F_Vector__fetch1(corners, 0);
    CV_Point2D32F corner1 = CV_Point2D32F_Vector__fetch1(corners, 1);
    CV_Point2D32F corner2 = CV_Point2D32F_Vector__fetch1(corners, 2);
    CV_Point2D32F corner3 = CV_Point2D32F_Vector__fetch1(corners, 3);

    // Extract the x and y references from {corner0} through {corner3}:
    Double x0 = CV_Point2D32F__x_get(corner0);
    Double y0 = CV_Point2D32F__y_get(corner0);
    Double x1 = CV_Point2D32F__x_get(corner1);
    Double y1 = CV_Point2D32F__y_get(corner1);
    Double x2 = CV_Point2D32F__x_get(corner2);
    Double y2 = CV_Point2D32F__y_get(corner2);
    Double x3 = CV_Point2D32F__x_get(corner3);
    Double y3 = CV_Point2D32F__y_get(corner3);

    Double dx21 = x2 - x1;
    Double dy21 = y2 - y1;
    Double dx30 = x3 - x0;
    Double dy30 = y3 - y0;

    // Determine the points ({xx0, yy0}) and ({xx1, yy1}) that determine
    // a line parrallel to one side of the quadralatal:
    Double xx0 = x1 + dx21 * 5.0 / 20.0;
    Double yy0 = y1 + dy21 * 5.0 / 20.0;
    Double xx1 = x0 + dx30 * 5.0 / 20.0;
    Double yy1 = y0 + dy30 * 5.0 / 20.0;

    // Set the outside and inside reference points along the line
    // through points ({xx0, yy0}) and ({xx1, yy1}):
    Double dxx10 = xx1 - xx0;
    Double dyy10 = yy1 - yy0;
    CV_Point2D32F__x_set(reference0, xx0 + dxx10 * -1.0 / 20.0);
    CV_Point2D32F__y_set(reference0, yy0 + dyy10 * -1.0 / 20.0);
    CV_Point2D32F__x_set(reference4, xx0 + dxx10 * 1.0 / 20.0);
    CV_Point2D32F__y_set(reference4, yy0 + dyy10 * 1.0 / 20.0);
    CV_Point2D32F__x_set(reference1, xx0 + dxx10 * 21.0 / 20.0);
    CV_Point2D32F__y_set(reference1, yy0 + dyy10 * 21.0 / 20.0);
    CV_Point2D32F__x_set(reference5, xx0 + dxx10 * 19.0 / 20.0);
    CV_Point2D32F__y_set(reference5, yy0 + dyy10 * 19.0 / 20.0);

    // Determine the points ({xx2, yy2}) and ({xx3, yy3}) that determine
    //a line parrallel to the other side of the quadralatal:
    Double xx2 = x1 + dx21 * 15.0 / 20.0;
    Double yy2 = y1 + dy21 * 15.0 / 20.0;
    Double xx3 = x0 + dx30 * 15.0 / 20.0;
    Double yy3 = y0 + dy30 * 15.0 / 20.0;

    // Set the outside and inside reference points along the line
    // through points ({xx2, yy2}) and ({xx3, yy3}):
    Double dxx32 = xx3 - xx2;
    Double dyy32 = yy3 - yy2;
    CV_Point2D32F__x_set(reference2, xx2 + dxx32 * -1.0 / 20.0);
    CV_Point2D32F__y_set(reference2, yy2 + dyy32 * -1.0 / 20.0);
    CV_Point2D32F__x_set(reference6, xx2 + dxx32 * 1.0 / 20.0);
    CV_Point2D32F__y_set(reference6, yy2 + dyy32 * 1.0 / 20.0);
    CV_Point2D32F__x_set(reference3, xx2 + dxx32 * 21.0 / 20.0);
    CV_Point2D32F__y_set(reference3, yy2 + dyy32 * 21.0 / 20.0);
    CV_Point2D32F__x_set(reference7, xx2 + dxx32 * 19.0 / 20.0);
    CV_Point2D32F__y_set(reference7, yy2 + dyy32 * 19.0 / 20.0);

    return references;
}

Integer Fiducials__points_maximum(Fiducials fiducials,
  CV_Point2D32F_Vector points, Unsigned start_index, Unsigned end_index) {
    // This routine will sweep from {start_index} to {end_index} through
    // {points}.  Using each selected point in {points}, the corresponding
    // value in {image} is sampled.  The minimum of the sampled point is
    // returned.

    // Start with a big value move it down:
    Integer result = 0;

    // Iterate across the {points} from {start_index} to {end_index}:
    for (Unsigned index = start_index; index <= end_index; index++) {
	CV_Point2D32F point = CV_Point2D32F_Vector__fetch1(points, index);
	Integer value = Fiducials__point_sample(fiducials, point);
	//call d@(form@("max[%f%:%f%]:%d%\n\") %
	//  f@(point.x) % f@(point.y) / f@(value))
	if (value > result) {
	// New maximum value:
	    result = value;
	}
    }
    return result;
}

Integer Fiducials__points_minimum(Fiducials fiducials,
  CV_Point2D32F_Vector points, Unsigned start_index, Unsigned end_index) {

    // This routine will sweep from {start_index} to {end_index} through
    // {points}.  Using each selected point in {points}, the corresponding
    // value in {image} is sampled.  The minimum of the sampled point is
    // returned.

    // Start with a big value move it down:
    Integer result = 0x7fffffff;

    // Iterate across the {points} from {start_index} to {end_index}:
    for (Unsigned index = start_index; index <= end_index; index++) {
	CV_Point2D32F point = CV_Point2D32F_Vector__fetch1(points, index);
	Integer value = Fiducials__point_sample(fiducials, point);
	if (value < result) {
	    // New minimum value:
	    result = value;
	}
    }
    return result;
}

void Fiducials__sample_points_compute(
  CV_Point2D32F_Vector corners, CV_Point2D32F_Vector sample_points) {

    // This routine will use the 4 corners in {corners} as a quadralateral
    // to compute an 8 by 8 grid of tag bit sample points and store the
    // results into the the 64 preallocated {CV_Point2D32F} objects in
    // {sample_points}.  The quadralateral must be convex and in the
    // counter-clockwise direction.  Bit 0 will be closest to corners[1],
    // bit 7 will be closest to corners[0], bit 56 closest to corners[2] and
    // bit 63 closest to corners[3].

    CV_Point2D32F corner0 = CV_Point2D32F_Vector__fetch1(corners, 0);
    CV_Point2D32F corner1 = CV_Point2D32F_Vector__fetch1(corners, 1);
    CV_Point2D32F corner2 = CV_Point2D32F_Vector__fetch1(corners, 2);
    CV_Point2D32F corner3 = CV_Point2D32F_Vector__fetch1(corners, 3);

    // Extract the x and y references from {corner0} through {corner3}:
    Double x0 = CV_Point2D32F__x_get(corner0);
    Double y0 = CV_Point2D32F__y_get(corner0);
    Double x1 = CV_Point2D32F__x_get(corner1);
    Double y1 = CV_Point2D32F__y_get(corner1);
    Double x2 = CV_Point2D32F__x_get(corner2);
    Double y2 = CV_Point2D32F__y_get(corner2);
    Double x3 = CV_Point2D32F__x_get(corner3);
    Double y3 = CV_Point2D32F__y_get(corner3);

    // Figure out the vector directions {corner1} to {corner2}, as well as,
    // the vector from {corner3} to {corner0}.  If {corners} specify a
    // quadralateral, these vectors should be approximately parallel:
    Double dx21 = x2 - x1;
    Double dy21 = y2 - y1;
    Double dx30 = x3 - x0;
    Double dy30 = y3 - y0;

    // {index} will cycle through the 64 sample points in {sample_points}:
    Unsigned index = 0;

    // There are ten rows (or columns) enclosed by the quadralateral.
    // (The outermost "white" rows and columns are not enclosed by the
    // quadralateral.)  Since we want to sample the middle 8 rows (or
    // columns), We want a fraction that goes from 3/20, 5/20, ..., 17/20.
    // The fractions 1/20 and 19/20 would correspond to a black border,
    // which we do not care about:
    Double i_fraction = 3.0 / 20.0;
    Double i_increment = 2.0 / 20.0;

    // Loop over the first axis of the grid:
    Unsigned i = 0;
    while (i < 8) {

	// Compute ({xx1},{yy1}) which is a point that is {i_fraction} between
	// ({x1},{y1}) and ({x2},{y2}), as well as, ({xx2},{yy2}) which is a
	// point that is {i_fraction} between ({x0},{y0}) and ({x3},{y3}).
	Double xx1 = x1 + dx21 * i_fraction;
        Double yy1 = y1 + dy21 * i_fraction;
        Double xx2 = x0 + dx30 * i_fraction;
        Double yy2 = y0 + dy30 * i_fraction;

	// Compute the vector from ({xx1},{yy1}) to ({xx2},{yy2}):
	Double dxx21 = xx2 - xx1;
	Double dyy21 = yy2 - yy1;

	// As with {i_fraction}, {j_fraction} needs to sample the
	// the data stripes through the quadralateral with values
	// that range from 3/20 through 17/20:
	Double j_fraction = 3.0 / 20.0;
	Double j_increment = 2.0 / 20.0;

	// Loop over the second axis of the grid:
	Unsigned j = 0;
	while (j < 8) {
	    // Fetch next {sample_point}:
	    CV_Point2D32F sample_point =
	      CV_Point2D32F_Vector__fetch1(sample_points, index);
	    index = index + 1;

            // Write the rvGrid position into the rvGrid array:
	    CV_Point2D32F__x_set(sample_point, xx1 + dxx21 * j_fraction);
	    CV_Point2D32F__y_set(sample_point, yy1 + dyy21 * j_fraction);

	    // Increment {j_faction} to the sample point:
	    j_fraction = j_fraction + j_increment;
	    j = j + 1;
	}

	// Increment {i_fraction} to the next sample striple:
	i_fraction = i_fraction + i_increment;
	i = i + 1;
    }

    CV_Point2D32F sample_point0 =
      CV_Point2D32F_Vector__fetch1(sample_points, 0);
    CV_Point2D32F sample_point7 =
      CV_Point2D32F_Vector__fetch1(sample_points, 7);
    CV_Point2D32F sample_point56 =
      CV_Point2D32F_Vector__fetch1(sample_points, 56);
    CV_Point2D32F sample_point63 =
      CV_Point2D32F_Vector__fetch1(sample_points, 63);

    // clockwise direction.  Bit 0 will be closest to corners[1], bit 7
    // will be closest to corners[0], bit 56 closest to corners[2] and
    // bit 63 closest to corners[3].

    //Fiducials__sample_points_helper("0:7", corner0, sample_point7);
    //Fiducials__sample_points_helper("1:0", corner0, sample_point0);
    //Fiducials__sample_points_helper("2:56", corner0, sample_point56);
    //Fiducials__sample_points_helper("3:63", corner0, sample_point63);
}


void Fiducials__sample_points_helper(
  String label, CV_Point2D32F corner, CV_Point2D32F sample_point) {
    Double corner_x = CV_Point2D32F__x_get(corner);
    Double corner_y = CV_Point2D32F__y_get(corner);
    Double sample_point_x = CV_Point2D32F__x_get(sample_point);
    Double sample_point_y = CV_Point2D32F__y_get(sample_point);
    File__format(stderr, "Label: %s corner: %f:%f sample_point %f:%f\n",
      label, (Integer)corner_x, (Integer)corner_y,
      (Integer)sample_point_x, (Integer)sample_point_y);
}

void Fiducials__tag_record(Unsigned direction, CV_Point2D32F_Vector vector) {
    // This routine will update the contents {Tag} to contain {direction},
    // and {vector}.  {vector} contains four points that form a convex
    // quadralateral in the counter-clockwise direction.  This routine will
    // compute the diagonal and twist values for {tag} as well.

    // Load up the contents of {tag.corners} from {corners} depending
    // upon {direction}:
    Unsigned offset = 0;
    switch (direction) {
      case 0:
	// North mapping:
	//offset := 2
	offset = 0;
	break;
      case 1:
	// East mapping:
	//offset := 1
	offset = 1;
	break;
      case 2:
	// South mapping:
	//offset := 0
	offset = 2;
  	break;
      case 3:
	// West mapping:
	//offset := 3
	offset = 3;
        break;
      default:
	assert (0);
    }

    // Compute {x_center} and {y_center} and fill in {corners}:
    //tag_corners :@= tag.corners
    for (Unsigned point_index = 0; point_index < 4; point_index++) {
	Unsigned corner_index = 0;
	switch (direction) {
	  case 0:
	    corner_index = (3 - point_index + 2) & 3;
	    break;
	  case 1:
	    corner_index = (3 - point_index + 1) & 3;
	    break;
	  case 2:
	    corner_index = (3 - point_index + 0) & 3;
	    break;
	  case 3:
	    corner_index = (3 - point_index + 3) & 3;
	    break;
	  default:
	    assert(0);
	    break;
	}
	//corner :@= vector[corner_index]
	//x :@= corner.x
	//y :@= corner.y
	//tag_corner :@= tag_corners[point_index]
	//tag_corner.x := x
	//tag_corner.y := y
	//point_index := point_index + 1
    }

    // The comment below is out of date:
    //# The Y axis in is image coordinates goes from 0 at the top to
    //# a positive number as it goes towards the bottom.  This is the
    //# opposite direction from from normal cartisian coordinates where
    //# positive Y goes up.  Because my brain can't cope with angles
    //# unless they are in cartisian coordinates, I negate the Y axis
    //# for the purpose of computing the {twist} angles below.  This
    //# flips the direction of the Y axis.

    Double pi = 3.14159265358979323846;

    // Compute {twist}:
    //tag_corner0 = tag_corners[0]
    //tag_corner1 = tag_corners[1]
    //tag_corner2 = tag_corners[2]
    //tag_corner3 = tag_corners[3]

    // Pull out the X and Y coordinates:
    //x0 :@= tag_corner0.x
    //y0 :@= tag_corner0.y
    //x1 :@= tag_corner1.x
    //y1 :@= tag_corner1.y
    //x2 :@= tag_corner2.x
    //y2 :@= tag_corner2.y
    //x3 :@= tag_corner3.x
    //y3 :@= tag_corner3.y

    // Compute the angle of the tag bottom edge to camera X axis:
    //dx01 :@= x0 - x1
    //dy01 :@= y0 - y1
    //twist1 :@= arc_tangent2@(dy01, dx01)
    //dx32 :@= x3 - x2
    //dy32 :@= y3 - y2
    //twist2 :@= arc_tangent2@(dy32, dx32)

    // We want the average angle of {twist1} and {twist2}.  We have
    // be careful about modular arithmetic issues.  Compute the angle
    // change and add in half of that to get the average angle:
    //twist_change :@= angle_between@(twist1, twist2)
    //twist :@= angle_normalize@(twist1 + twist_change / 2.0)
    //tag.twist := twist

    // Compute the X/Y axis deltas for the two diagonals:
    //dx02 :@= x0 - x2
    //dy02 :@= y0 - y2
    //dx13 :@= x1 - x3
    //dy13 :@= y1 - y3

    // Compute the actual diagonals:
    //diagonal02 :@= square_root@(dx02 * dx02 + dy02 * dy02)
    //diagonal13 :@= square_root@(dx13 * dx13 + dy13 * dy13)

    // Compute the average diagonal:
    //diagonal :@= (diagonal02 + diagonal13) / 2.0
    //tag.diagonal := diagonal

    // Compute the center by averagine all for corners:
    //center_x :@= (x0 + x1 + x2 + x3) / 4.0
    //center_y :@= (y0 + y1 + y2 + y2) / 4.0
    //tag.center_x := center_x
    //tag.center_y := center_y

    //if trace
      //call d@(form@(
      //  "%p%id=%d% c0=%2f%,%2f% c1=%2f%,%2f% c2=%2f%,%2f% c3=%2f%,%2f%\n\") %
      //  f@(indent1) % f@(tag.id) %
      //  f@(x0) % f@(y0) % f@(x1) % f@(y1) %
      //  f@(x2) % f@(y2) % f@(x3) / f@(y3))
      //call d@(form@("%p%dx01=%2f% dy01=%2f% dir=%d%\n\") %
      //  f@(indent1) % f@(dx01) % f@(dy01) / f@(tag.direction))
      //call d@(form@("%p%tw1=%2f% tw2=%2f% tw=%2f%\n\") %
      //  f@(indent1) % f@(twist1 * 180.0 / pi) %
      //  f@(twist2 * 180.0 / pi) /  f@(twist * 180.0 / pi))
      //call d@(form@("%p%center_x==%2f% center_y=%2f%\n\") %
      //  f@(indent1) % f@(center_x) / f@(center_y))

    // # For debugging, display everything:
    //if 0f	# 1t
       //#call d@(form@("Mapping:%v% offset:%d%\n\") %
       //#  f@(extractor.mapping_names[direction]) / f@(offset))
       //index :@= 0i
       //while index < 4i
   	    //corner := vector[unsigned@(index)]
	    //vector_x :@= round@CV(corner.x)
	    //vector_y :@= round@CV(corner.y)

	    //#point_x :@= round@CV(get_real_2d@CV(points, index, 0i))
	    //#npoint_y :@= round@CV(get_real_2d@CV(points, index, 1i))
	    //#corner_x :@= round@CV(get_real_2d@CV(corners, index, 0i))
	    //#corner_y :@= round@CV(get_real_2d@CV(corners, index, 1i))

	    //#call d@(form@(
	    //#  "[%d%]: corner_vect=%d%:%d% point=%d%:%d% corner=%d%:%d%\n\") %
	    //#  f@(index) % f@(vector_x) % f@(vector_y) %
	    //#  f@(point_x) % f@(point_y) %
	    //#  f@(corner_x) / f@(corner_y))
	    //index := index + 1i
       //#call d@(form@("corners_vec CW:%l% points CW:%l% corners CW:%l%\n\") %
       //#  f@(is_clockwise@(vector)) % f@(is_clockwise@(points)) /
       //#  f@(is_clockwise@(corners)))

    //if trace
       //call d@(form@("%p%<=record@Tag(T%d%, *)\n\") % f@(indent) / f@(tag.id))
}
		  
void Fiducials__tag_heights_xml_read(
  Fiducials fiducials, String xml_file_name) {
    File xml_in_file = File__open(xml_file_name, "r");
    if (xml_in_file == (File)0) {
	File__format(stderr, "Could not open '%s'\n", xml_file_name);
	assert(0);
    }
    Map map = fiducials->map;
    assert(map != (Map)0);
    Map__tag_heights_xml_read(map, xml_in_file);
    File__close(xml_in_file);
}
