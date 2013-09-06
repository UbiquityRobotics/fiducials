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
    CV_Scalar blue;
    Logical blur;
    CV_Point2D32F_Vector corners_vector;
    CV_Scalar cyan;
    CV_Image debug_image;
    Unsigned debug_index;
    CV_Image edge_image;
    CV_Image gray_image;
    CV_Scalar green;
    CV_Point origin;
    CV_Image original_image;
    CV_Scalar purple;
    CV_Scalar red;
    CV_Point2D32F_Vector references;
    CV_Size size_5x5;
    CV_Size size_m1xm1;
    CV_Memory_Storage storage;
    CV_Term_Criteria term_criteria;
};

extern Fiducials Fiducials__create(CV_Image original_image);
extern void Fiducials__image_show(Fiducials fiducials, Logical show);
extern Unsigned Fiducials__process(Fiducials fiducials);
extern CV_Point2D32F_Vector Fiducials__references_compute(
  Fiducials fiducials, CV_Point2D32F_Vector corners);

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

    Integer term_criteria_type =
      CV__term_criteria_iterations | CV__term_criteria_eps;

    // Create and load *fiducials*:
    Fiducials fiducials = Memory__new(Fiducials);
    fiducials->blue = CV_Scalar__rgb(0.0, 0.0, 1.0);
    fiducials->blur = (Logical)1;
    fiducials->corners_vector = CV_Point2D32F_Vector__create(4);
    fiducials->cyan = CV_Scalar__rgb(0.0, 1.0, 1.0);
    fiducials->debug_image = CV_Image__create(image_size, CV__depth_8u, 3);
    fiducials->debug_index = 0;
    fiducials->edge_image = CV_Image__create(image_size, CV__depth_8u, 1);
    fiducials->gray_image = CV_Image__create(image_size, CV__depth_8u, 1);
    fiducials->green = CV_Scalar__rgb(0.0, 255.0, 0.0);
    fiducials->origin = CV_Point__create(0, 0);
    fiducials->original_image = original_image;
    fiducials->purple = CV_Scalar__rgb(255.0, 0.0, 255.0);
    fiducials->red = CV_Scalar__rgb(255.0, 0.0, 0.0);
    fiducials->references = CV_Point2D32F_Vector__create(8);
    fiducials->size_5x5 = CV_Size__create(5, 5);
    fiducials->size_m1xm1 = CV_Size__create(-1, -1);
    fiducials->storage = storage;
    fiducials->term_criteria = 
      CV_Term_Criteria__create(term_criteria_type, 5, 0.2);
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

	// If we have a 4-sided polygon with an area greater than 500 square
	// pixels, we can explore to see if we have a tag:
	if (CV_Sequence__total_get(polygon_contour) == 4 &&
	  fabs(CV_Sequence__contour_area(polygon_contour,
	  CV__whole_seq, 0)) > 500.0 &&
	  CV_Sequence__check_contour_convexity(polygon_contour)) {
	    // For debugging, display the polygons in red:
	    //File__format(stderr, "Have 4 sides > 500i\n");

	    // Just show the fiducial outlines for *debug_index* of 6:
	    if (debug_index == 6) {
		CV_Scalar red = fiducials->red;
		CV_Image__draw_contours(debug_image,
		  polygon_contour, red, red, 2, 2, 1, origin);
	    }

	    // Copy the 4 corners from {poly_contour} to {corners_vector}:
	    CV_Point2D32F_Vector corners_vector = fiducials->corners_vector;
	    for (Unsigned index = 0; index < 4; index++) {
		CV_Point2D32F corner =
		  CV_Point2D32F_Vector__fetch1(corners_vector, index);
		CV_Point point =
		  CV_Sequence__point_fetch1(polygon_contour, index);
		CV_Point2D32F__point_set(corner, point);

		if (debug_index == 6) {
		    //File__format(stderr,
		    //  "point[%d] x:%f y:%f\n", index, point->x, point->y);
		}
	    }

	    // Now find the sub pixel corners of {corners_vector}:
	    CV_Image__find_corner_sub_pix(gray_image, corners_vector, 4,
	      fiducials->size_5x5, fiducials->size_m1xm1,
	      fiducials->term_criteria);

	    // Ensure that the corners are in a counter_clockwise direction:
	    CV_Point2D32F_Vector__corners_normalize(corners_vector);

	    // For debugging show the 4 corners of the possible tag where
	    //corner0=red, corner1=green, corner2=blue, corner3=purple:
	    if (debug_index == 7) {
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
	      Fiducials__references_compute(fiducials, corners_vector);

	    // Now sample the periphery of the tag and looking for the
	    // darkest white value (i.e. minimum) and the lightest black
	    // value (i.e. maximum):
	    Integer white_darkest =
	      CV_Image__points_minimum(gray_image, references, 0, 3);
	    Integer black_lightest =
	      CV_Image__points_maximum(gray_image, references, 4, 7);

	    // {threshold} should be smack between the two:
	    Integer threshold = (white_darkest + black_lightest) / 2;
	    
	    // For debugging, show the 8 points that are sampled around the
	    // the tag periphery to even decide whether to do further testing.
	    // Show "black" as green crosses, and "white" as green crosses:
	    if (debug_index == 8) {
		CV_Scalar red = fiducials->red;
		CV_Scalar green = fiducials->green;
		for (Unsigned index = 0; index < 8; index++) {
		    CV_Point2D32F reference =
		      CV_Point2D32F_Vector__fetch1(references, index);
		    Integer x = CV__round(CV_Point2D32F__x_get(reference));
		    Integer y = CV__round(CV_Point2D32F__y_get(reference));
		    Integer value =
		      CV_Image__point_sample(gray_image, reference);
		    CV_Scalar color = red;
		    if (value < threshold) {
		        color = green;
		    }
		    CV_Image__cross_draw(debug_image, x, y, color);
		    File__format(stderr, "ref[%d:%d]:%d\n", x, y, value);
		}
	    }
	}
    }

    return 0;
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

Integer CV_Image__points_maximum(CV_Image image,
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
	Integer value = CV_Image__point_sample(image, point);
	//call d@(form@("max[%f%:%f%]:%d%\n\") %
	//  f@(point.x) % f@(point.y) / f@(value))
	if (value > result) {
	// New maximum value:
	    result = value;
	}
    }
    return result;
}


Integer CV_Image__points_minimum(CV_Image image,
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
	Integer value = CV_Image__point_sample(image, point);
	if (value < result) {
	    // New minimum value:
	    result = value;
	}
    }
    return result;
}

Integer CV_Image__point_sample(CV_Image image, CV_Point2D32F point) {
    // This routine will return a sample ...

    Integer x = CV__round(CV_Point2D32F__x_get(point));
    Integer y = CV__round(CV_Point2D32F__y_get(point));
    Integer center = CV_Image__gray_fetch(image, x, y);
    Integer left = CV_Image__gray_fetch(image, x - 1, y);
    Integer right = CV_Image__gray_fetch(image, x + 1, y);
    Integer lower = CV_Image__gray_fetch(image, x, y - 1);
    Integer upper = CV_Image__gray_fetch(image, x, y + 1);
    Integer result = -1;
    if (center >= 0 && left >= 0 && right >= 0 && lower >= 0 && upper >= 0) {
	result = (center + left + right + lower + upper) / 5;
    }
    return result;
}
