// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include "Camera_Tag.h"
#include "Double.h"
#include "Memory.h"
#include "Tag.h"
#include "Unsigned.h"

/// @brief Return the sort order of *camera_tag1* to *camera_tag2*.
/// @param camera_tag1 is the first *Camera_Tag* to compare.
/// @param camera_tag2 is the second *Camera_Tag* to compare.
/// @returns -1, 0, or 1 depending upon sort order.
///
/// *Camera_Tag__compare*() will return the -1 if *camera_tag1* is less
/// than *camera_tag2*, 0 if they are equal, and 1 otherwise.

Integer Camera_Tag__compare(Camera_Tag camera_tag1, Camera_Tag camera_tag2) {
    return Tag__compare(camera_tag1->tag, camera_tag2->tag);
}

/// @brief Initializes *camara_tag* from *tag_id*, *direction*, and *corners*
/// @param camera_tag is the *Camera_Tag* to be initialized.
/// @param tag_id is the fiducial tag identifier.
/// @param direction is the direction that match the fiducial orientation.
/// @param corners are the 4 corners of the fiducial tag.
///
/// *Camera_Tag__initialize*() will initialize *camera_tag*.  *tag_id*
/// is the fiduical tag identifier.  *direction* specifies which of the
/// four fiducial orientations passed the FEC (forward Error Correction)
/// and CRC (Cyclic Redundancy Check) tests to extract the *tag_id*.
/// *camara_tag* has its diagonal, twist and center X/Y coordiante filled
/// from this routine.

void Camera_Tag__initialize(Camera_Tag camera_tag, Tag tag,
  Unsigned direction, CV_Point2D32F_Vector corners, CV_Image debug_image) {
    // We need to remap the 4 corners in *corners* to be oriented as
    // in the crude ASCII art shown below:
    //
    //        2-----------3 ----> twist32 (=P3 - P2)
    //        |           |
    //        |           |
    //        |     C     |
    //        |           |
    //        |           |
    //        1-----------0 ----> twist01 (=P0 - P1)
    //
    // The *direction* argument provides the information to rotate the
    // X/Y valuds in *corners* into this orientation:

    // Fill up *x_corners* and *y_corners* using the *direction* to
    // properly index the 4 corners in *corners*.  I wish this code
    // were a little clearer, but it literally took several hours to
    // figure out.  Basically 4 different orientations of the same
    // fiducial were fed into this code to make sure it computed the
    // correct 4 corners:
    Double x_corners[4];
    Double y_corners[4];
    for (Unsigned index = 0; index < 4; index++) {
	unsigned corner_index = 0;
	switch (direction) {
	  case 0:
	    corner_index = (3 - index + 2) & 3;
	    break;
	  case 1:
	    corner_index = (3 - index + 1) & 3;
	    break;
	  case 2:
	    corner_index = (3 - index + 0) & 3;
	    break;
	  case 3:
	    corner_index = (3 - index + 3) & 3;
	    break;
	  default:
	    assert(0);
	    break;
	}
	CV_Point2D32F corner =
	  CV_Point2D32F_Vector__fetch1(corners, corner_index);
	x_corners[index] = CV_Point2D32F__x_get(corner);
	y_corners[index] = CV_Point2D32F__y_get(corner);
    }

    // Pull out the X and Y coordinates into separate variables:
    Double x0 = x_corners[0];
    Double y0 = y_corners[0];
    Double x1 = x_corners[1];
    Double y1 = y_corners[1];
    Double x2 = x_corners[2];
    Double y2 = y_corners[2];
    Double x3 = x_corners[3];
    Double y3 = y_corners[3];

    // For debugging plot the for colors
    if (debug_image != (CV_Image)0) {
	for (Unsigned index = 0; index < 4; index++) {
	    Integer x = (Integer)x_corners[index];
	    Integer y = (Integer)y_corners[index];
	    CV_Scalar color = (CV_Scalar)0;
	    String text = (String)0;
	    switch (index) {
	      case 0:
		color = CV_Scalar__rgb(255.0, 0.0, 0.0);
		text = "red";
		break;
	      case 1:
		color = CV_Scalar__rgb(0.0, 255.0, 0.0);
		text = "green";
		break;
	      case 2:
		color = CV_Scalar__rgb(0.0, 0.0, 255.0);
		text = "blue";
		break;
	      case 3:
		color = CV_Scalar__rgb(0.0, 255.0, 255.0);
		text = "cyan";
		break;
	      default:
		assert(0);
	    }
	    CV_Image__blob_draw(debug_image, x, y, color);
	    File__format(stderr,
	      "Corner[%d]=(%d:%d) %s\n", index, x, y, text);
	}
    }

    // Compute the angle of the tag bottom edge relative to the camera X axis:
    Double dx01 = x0 - x1;
    Double dy01 = y0 - y1;
    Double twist01 = Double__arc_tangent2(dy01, dx01);
    //File__format(stderr,
    //  "CT_init:id=%d x0:%.2f x1:%.2f dx01:%.2f y0:%.2f y1:%.2f dy01:%.2f\n",
    //  tag->id, x0, x1, dx01, y0, y1, dy01);

    // Compute the angle of the tag top edge relative to the camera X axis:
    Double dx32 = x3 - x2;
    Double dy32 = y3 - y2;
    Double twist32 = Double__arc_tangent2(dy32, dx32);

    // We want the average angle of *twist01* and *twist32*.  We have
    // be careful about modular arithmetic issues.  Compute the angle
    // change and add in half of that to get the average angle:
    Double twist_change = Double__angle_between(twist01, twist32);
    Double twist = Double__angle_normalize(twist01 + twist_change / 2.0);

    Double pi = (Double)3.14159265358979323846264;
    Double r2d = 180.0 / pi;
    //File__format(stderr, "CT_init:id:%d tw01:%.4f tw32:%.4f tw:%.4f\n",
    //  tag->id, twist01 * r2d , twist32 * r2d, twist * r2d);

    // Compute the average of the two diagonals:
    Double dx02 = x0 - x2;
    Double dy02 = y0 - y2;
    Double diagonal02 = Double__square_root(dx02 * dx02 + dy02 * dy02);
    Double dx13 = x1 - x3;
    Double dy13 = y1 - y3;
    Double diagonal13 = Double__square_root(dx13 * dx13 + dy13 * dy13);
    Double diagonal = (diagonal02 + diagonal13) / 2.0;

    // Compute the tag center by averaging for all four corners:
    Double center_x = (x0 + x1 + x2 + x3) / 4.0;
    Double center_y = (y0 + y1 + y2 + y2) / 4.0;

    // Load up *carmera_tag*:
    camera_tag->diagonal = diagonal;
    camera_tag->direction = direction;
    camera_tag->tag = tag;
    camera_tag->twist = twist;
    camera_tag->x = center_x;
    camera_tag->y = center_y;
}

/// @brief Return a new *Camera_Tag* object.
/// @returns new *Camera_Tag* object.
///
/// *Camera_Tag__new*() will return a new *Camera_Tag* object.

Camera_Tag Camera_Tag__new(void) {
    Camera_Tag camera_tag = Memory__new(Camera_Tag);
    camera_tag->diagonal = 0.0;
    camera_tag->direction = 0;
    camera_tag->tag = (Tag)0;
    camera_tag->twist = 0.0;
    camera_tag->x = 0.0;
    camera_tag->y = 0.0;
    return camera_tag;
}




