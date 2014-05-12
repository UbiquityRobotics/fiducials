// Copyright (c) 2013-2014 by Wayne C. Gramlich.  All rights reserved.

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

// Introduction:
//
// This code body takes a camera image that contains one or more fiducial
// tag images and computes an estimated location and orientation for the
// image camera.  This computation is done in conjunction with a map that
// contains the best estimate of the locations of each of the fiducial tags.
// In short:
// 
//    Image_Location = F(Image, Map)
//
// To further complicate things, in fact, the Map is updated as
// side effect of computing the current location.  In the literature,
// this is call SLAM, for Simultaneous Localization And Mapping.
// Thus, what is really going on is:
//
//    (Image_Location, Map') = F(Image, Map)
//
// where Map' is the updated map.
//
// Despite the fact that the map is being updated, it best to treat the
// localization computation different from the Map updating computation.
// Thus, what is really going on is:
//
//    Image_Location = F(Image, Map)
//
//    Map' = G(Image, Map)
//
// where F is the localization function, and G is the map update function.
//
// The sections below are entitled:
//
// * Tag Geometry
// * Coordinate Systems
// * Map Organization
// * Image Tags
// * Localization Computation
// * Fusing with Dead Reckoning
// * Mapping into Robot Coordinates
// * Coordinate Space Transforms
//
// Tag Geometry:
//
// A tag consists of a 12 x 12 matrix of bits that encodes 64-bits of
// information in an 8 x 8 data matrix.  The outermost square of the matrix
// is set to zeros (white squares) and the next level in is 1's
// (black squares).  When viewed in an upright form printed on paper
// it looks as follows (-- = zero, XX = one, ## = bit number):
//
// -- -- -- -- -- -- -- -- -- -- -- --
// -- XX XX XX XX XX XX XX XX XX XX -- 
// -- XX 56 57 58 59 60 61 62 63 XX --
// -- XX 48 49 50 51 52 53 54 55 XX --
// -- XX 40 41 42 43 44 45 45 47 XX --
// -- XX 32 33 34 35 36 37 38 39 XX --
// -- XX 24 25 26 27 28 29 30 31 XX --
// -- XX 16 17 18 19 20 21 22 23 XX --
// -- XX 08 09 10 11 12 13 14 15 XX --
// -- XX 00 01 02 03 04 05 06 07 XX --
// -- XX XX XX XX XX XX XX XX XX XX -- 
// -- -- -- -- -- -- -- -- -- -- -- --
//
// Coordinate Systems:
//
// When it comes to performing the image analysis of a picture to
// compute image camera location and twist there are three coordinate
// systems to consider -- the floor, camera, and robot coordinate systems:
//
//    -	The floor coordinate system is a right handed cartesian
//	coordinate system where one point on the floor is the
//	origin.  The X and Y axes are separated by a 90 degree
//	angle and X axis can be oriented to aligned with either a
//	building feature or an east-west line of latitude or
//	whatever the user chooses.  Conceptually, the floor is
//	viewed from the top looking down.  As long as all length
//	measurements are performed using the same units, the
//	choice of length unit does not actually matter (e.g.
//	meter, centimeter, millimeter, inch, foot, etc.)
//
//    -	The image coordinate system is another right handed cartesian
//	coordinate system that is used to locate the individual
//	pixels in the image.  The image is one of the common image
//	format sizes (320 x 240, 640 x 480, etc.)  For this system,
//	the origin is selected to be the lower left pixel of the
//	image.  For 640 x 480 image, the lower left coordinate is
//	(0, 0) and the upper right pixel coordinate is (639, 479).
//
//	While most computer imaging packages (e.g. OpenCV) typically
//      use a left-handed coordinate system where the origin is in
//      the upper left,	we will apply a transformation that causes
//      the origin to be placed in the lower left.  All distances are
//      measured in units of pixels.  The reason for using a right
//      handed coordinate system is because trigonometric functions
//      are implicitly right-handed.
//
//    -	The robot coordinate system is another right handed cartesian
//	coordinate system where the origin is located in the center
//	between the two drive wheels.  The X axis of the robot
//	coordinate system goes through the front of the robot.
//      Realistically, the robot coordinate system is not part
//      of the algorithms, instead this coordinate system is called
//      out just to point out that it is there.
//
// Map Organization:
//
// Conceptually there a bunch of square fiducial tags (hereafter called
// just tags) scattered on the floor.  (Yes, we know they are actually
// on the ceiling, we'll deal with that detail later.)  There is a map
// that records for each tag, the following information in an ordered
// quintuple:
//
//    (mtid, mtx, mty, mttw, mtdiag)
//
// where:
//
//    mtid	(Map Tag IDentifier) is the tag identifier (a number between
//              0 and 2^16-1),
//    mtx	(Map Tag X) is the x coordinate of the tag center in
//              floor coordinates,
//    mty	(Map Tag Y) is the y coordinate of the tag center in
//              floor coordinates,
//    mttw	(Map Tag TWist) is the angle (twist) of the bottom
//              tag edge and the floor X axis, and
//    mtdiag	(Map Tag DIAGonal) is the length of a tag diagonal
//              (both diagonals of a square have the same length) in
//              in floor coordinates.
//
// Eventually, the quintuple should probably be upgraded to 1) an
// identifier, 2) an X/Y/Z location and 3) a W/X/Y/Z quaternion of
// for the tag orientation.  That will come later.  For now we will
// just use the quintuple.
//
// Conceptually, the floor is printed out on a large sheet of paper
// with all tags present.  The camera image is printed out on a piece
// of translucent material (e.g. acetate) with the same dimensions
// as the floor sheet.  This translucent image is placed on the floor
// sheet and moved around until the image tag(s) align with the
// corresponding tag(s) on the floor sheet.  The center of translucent
// image is now sitting directly on top of the image location on the
// floor sheet.  The amount of "twist" on the image relate to the
// floor coordinate system complete the location computation.
// Hopefully, this mental image of what we are trying to accomplish
// will help as you wade through the math below.
//
// Conceptually, the camera is going to be placed a fixed distance
// above the floor and take an image of rectangular area of the floor.
// For each tag in the image, the image processing algorithm
// computes the following:
//
//	(camx, camy, camtw)
//
// where:
//
//    camx	(CAMera X) is the camera center X coordinate in the
//		floor coordinate system,
//    camy	(CAMera Y) is the camera center Y coordinate in the
//              floor coordinate system, and
//    camtw	(CAMera TWist) is the angle (twist) of the rectangle bottom
//		tag edge with respect to the floor coordinate system X axis.
//
// If there are multiple tags present in the image, there will be
// multiple (camx, camy, camtw) triples computed.  These triples
// should all be fairly close to one another.  There is a final
// fusion step that fuses them all together into a single result.
//
// Image Tags:
//
// In each image there may be zero, one or more visible tags.  If
// there are zero tags present, there is nothing to work with and
// the camera location can not be computed.  When there are one or
// more tags, we tag detection algorithm computes the following
// information:
//
//    (tid, tc0, tc1, tc2, tc3)
//
//    tid	(Tag IDentifier) is the tag identifier,
//    tc0       (Tag Corner 0) is actually (tc0x, tc0y) the location
//              if the tag corner 0 in image coordinated,
//    tc1       (Tag Corner 1) is actually (tc0x, tc0y) the location
//              if the tag corner 0 in image coordinated,
//    tc2       (Tag Corner 2) is actually (tc0x, tc0y) the location
//              if the tag corner 0 in image coordinated, and
//    tc3       (Tag Corner 3) is actually (tc0x, tc0y) the location
//              if the tag corner 0 in image coordinated.
//
// The 4 tag corners go in a clockwise direction around the tag
// (when viewed from above, of course.)  The tag looks as follows
// in ASCII art:
//
//      +Y
//       ^
//       |
//       | tc2                         tc3
//       |     ** ** ** ** ** ** ** **
//       |     ** ** ** ** ** ** ** **
//       |     ** ** ** ** ** ** ** **
//       |     ** ** ** ** ** ** ** **
//       |     ** ** ** ** ** ** ** **
//       |     ** ** ** ** ** ** ** **
//       |     15 14 13 12 11 10  9  8
//       |      7  6  5  4  3  2  1  0
//       | tc1                         tc0
//       |
//       +--------------------------------> +X
//
// Localization Computation:
//
// The localization algorithm operates as follows:
//
//    foreach tag in image:
//        (tid, tc0, tc1, tc2, tc3) = tag_extract(image)
//        (mtid, mtx, mty, mttw, mtdiag) = Map(tid)
//        (camx, camy, camtw) = F(tc0, tc1, tc2, tc3, mtx, mty, mttw, mtdiag)
//
// Now let's get into the detail of the localization function F().
//
// Using (tc0, tc1, tc2, tc3), we compute the following:
//
//    tagctrx	(TAG CenTeR X) is the tag center X coordinate in
//              image coordinates,
//    tagctry	(TAG CenTeR y) is the tag center X coordinate in
//              image coordinates,
//    tagdiag	(TAG DIAGonal) is the average diagonal length in image
//              coordinates,
//    tagtw	(TAG TWist) is the angle of the lower tag edge with
//              respect to the image coordinate X axis.
//
// These values are computed as follows:
//
//   tagctrx = (tc0x + tc1x + tc2x + tc3x) / 4         # The average of the X's
//   tagctry = (tc0y + tc1y + tc2y + ct3y) / 4         # The average of the Y's
//   tagdiag1 = sqrt( (tc0x-tc2x)^2 + (tc0y-tc2y)^2) ) # First diagonal
//   tagdiag2 = sqrt( (tc1x-tc3x)^2 + (tc1y-tc3y)^2) ) # Second diagonal
//   tagdiag = (tagdiag1 + tagdiag2) / 2               # Avg. of both diagonals
//   tagtw = arctangent2(tc0y - tc1y, tc0x - tc1y)     # Bottom tag edge angle
//
// The image center is defined as:
//
//	(imgctrx, imgctry)
//
// where
//
//    imgctrx	(IMG CenTeR X) is the image center X coordinate in image
//              coordinates, and
//    imgctry	(IMG CenTeR Y) is the image center Y coordinate in image
//              coordinates.
//
// for an image that is 640 x 480, the image center is (320, 240).
// (Actually, it should be (319.5, 239.5).)
//
// Using the image center the following values are computed:
//
//    tagdist	   (TAG DISTance) is the distance from the tag center to
//                 image center in image coordinates, and
//    tagbear      (TAG BEARing) is the angle from the tag center to the
//                 image center in image coordinates.
//
// These two values are computed as:
//
//    tagdist = sqrt( (imgctrx - tagctrx)^2 + (imgctry - tagctry)^2) )
//    tagbear = arctangent2( imgctry - tagctry, imgctrx - tagctry)
//
// tagdist is in image coordinate space and we will need the same distance
// in floor coordinate space.  This is done using tagdiag and mtd (the
// map tag diagonal length.)
//
//    flrtagdist = tagdist * (mtd / tagdiag )
//
// where flrtagdist stands for FLooR TAG DISTance.
//
// Now we need to compute to angles that are measured relative to the
// floor X axis:
//
//    camtw	(CAMera TWist) is the direction that the camera X axis points
//		relative to the floor X Axis, and
//    cambear	(CAMera BEARing) is the direction to the camera center relative
//		to the floor X Axis.
//
// camtw and cambear are computed as:
//
//    camtw = mttw - tagtw
//
//    cambear = camtw + tagbear
//
// Now camx, and camy are computed as follows:
//
//    camx = mtx + flrtagdist * cos(cambear)
//    camy = mty + flrtagdist * sin(cambear)
//
// Thus, the final result of (camx, camy, ctw) has been determined.
//
// Mapping into Robot Coordinates:
//
// Once we know the camera location and orientation,
// (camx, camy, camtw), we need to compute the ordered triple:
//
//    (rx, ry, rbear)
//
// where
//
//    rx	(Robot X) is the X coordinate of the robot center in
//              floor coordinates,
//    ry	(Robot Y) is the Y coordinate of the robot center in
//              floor coordinates, and
//    rbear	(Robot BEARing) is the bearing angle of the robot X axis
//              to the floor coordinate X axis.
//
// These values are computed as a function:
//
//    (rx, ry, rbear) = F( (camx, camy, camtw) )
//
// There are two constants needed to do this computation:
//
//    robdist	(ROBot DISTance) is the distance from the camera
//              center to the robot center in floor coordinates, and
//    robcamtw	(ROBot CAMera TWist) is the angle from the angle from
//              camera X axis to the robot X axis.
//
// Both robdist and robcamtw are constants that can be directly measured
// from the camera placement relative to the robot origin.
//
// Now (rx, ry, rtw) can be computed as follows:
//
//    rtw = camtw + robcamtw
//    rx = camx + robdist * cos(rcamtw)
//    ry = camy + robdist * sin(rcamtw)
//
// That covers the localization processing portion of the algorithm.
//
// Fusing with Dead Reckoning:
//
// The robot dead reckoning system keeps track of the robot position
// using wheel encoders.  These values are represented in the ordered
// triple:
//
//    (ex, ey, etw)
//
// where
//
//    ex	(Encoder X) is the robot X coordinate in floor coordinates,
//    ey	(Encoder Y) is the robot Y coordinate in floor coordinates, and
//    etw	(Encoder TWist) is the robot X axis twist relative to the
//              floor coordinate X axis.
//
// (rx, ry, rtw) and (ex, ey, etw) are supposed to be the same.  Over
// time, small errors will accumulate in the computation of (ex, ey, etw).
// (rx, ry, rtw) can be used to reset (ex, ey, etw).
//
// Coordinate Space Transforms:
//
// That pretty much summarizes what the basic algorithm does.
//
// What remains is to do the transformations that place the tags
// on the ceiling.  There are three transformations.
//
//    tags lift		The tags are lifted straight up from the floor
//			to the ceiling.  The person looking on down
//			from above would see no changes in tag position
//			or orientation (ignoring parallax issues.)
//
//    camera flip	The camera is rotated 180 degrees around the
//			camera X axis.  The causes the Y axis to change
//			its sign.
//
//    image framing	For historical reasons, the camera image API
//			provides the camera image with the image origin
//			in upper left hand corner, whereas the all of
//			the math above assumes that image origin is
//			in the lower left corner.  It turns out this
//			is just changes the Y axis sign again.
//
// The bottom line is that the "camera flip" and the "image framing"
// transformation cancel one another out.  Thus, the there is no
// work needed to tweak the equations above.


/// @brief Callback routine that prints a new *Arc* object when it shows up.
/// @param announce_object is unused (other routines might us it).
/// @param from_id is the tag identifier that has the lower tag id number.
/// @param from_x is the X coordinate of the from tag.
/// @param from_y is the Y coordinate of the from tag.
/// @param from_z is the Z coordinate of the from tag.
/// @param to_id is the tag identifier that has the higher tag id number.
/// @param to_x is the X coordinate of the to tag.
/// @param to_y is the Y coordinate of the to tag.
/// @param to_z is the Z coordinate of the to tag.
/// @param goodness is the distance between the arc center point and the camera.
/// @param in_spanning_tree is true if the arc is in the spanning tree.
///
/// *Fiducials__arc_announce*() is a callback routine that can be called
/// whenever an arc data structure is modified.  The announce routine is
/// specified as an argument to *Map__create*().

// FIXME: Why isn't the from_twist and to_twist included???!!!
// FIXME: Why don't we just pass the *Arc* object???!!!

void Fiducials__arc_announce(void *announce_object,
  Integer from_id, Double from_x, Double from_y, Double from_z,
  Integer to_id, Double to_x, Double to_y, Double to_z,
  Double goodness, Logical in_spanning_tree) {
    File__format(stderr,
      "Arc: from=(%d, %f, %f, %f,) to=(%d, %f, %f, %f) %f %d\n",
      from_id, from_x, from_y, from_z,
      to_id, to_x, to_y, to_z,
      goodness, in_spanning_tree);
}

/// @brief Callback routine that prints out location when it changes.
/// @param announce_object is unused.
/// @param id is ???
/// @param x is the X location of the robot.
/// @param y is the Y location of the robot.
/// @param z is the Y location of the robot (currently ignored.)
/// @param bearing is the robot bearin in radians.
///
/// *Fiducials__location_announce*() is a callback routine that can be
/// calld to print out the location information.  The location announce
/// routine is a field of the *Fiducials_Create__Struct*.

//FIXME: Why don't we just pass in a *Location* object???!!!

void Fiducials__location_announce(void *announce_object, Integer id,
  Double x, Double y, Double z, Double bearing) {
    File__format(stderr,
      "Location: id=%d x=%f y=%f bearing=%f\n", id, x, y, bearing);
}

/// @brief Callback routine tthat prints out the fidicial information.
/// @param announce_object is unused.
/// @param id is the tag id.
/// @param direction specifies (0-3) which of the 4 possible fiducial
///        orientations matched.
/// @param world_diagonal is the diagonal measured in world coordinate.
/// @param x1 is the X coordinate of corner1 in camera coordinates.
/// @param y1 is the y coordinate of corner1 in camera coordinates.
/// @param x2 is the X coordinate of corner2 in camera coordinates.
/// @param y2 is the y coordinate of corner2 in camera coordinates.
/// @param x3 is the X coordinate of corner3 in camera coordinates.
/// @param y3 is the y coordinate of corner3 in camera coordinates.
/// @param x4 is the X coordinate of corner4 in camera coordinates.
/// @param y4 is the y coordinate of corner4 in camera coordinates.
///
/// *Fiducials__fiducial_announce*() will announce finding a fiducial
/// in a camera image.  This callback routine is supplied as an
/// argument to *Fiducials__create*().

void Fiducials__fiducial_announce(void *announce_object,
    Integer id, Integer direction, Double world_diagonal,
    Double x1, Double y1, Double x2, Double y2,
    Double x3, Double y3, Double x4, Double y4) {
    File__format(stderr,
       "Fiducial: id=%d dir=%d diag=%.2f (%.2f,%.2f), " /* + */
       "(%.2f,%.2f), (%.2f,%.2f), (%.2f,%.2f)",
       id, direction, world_diagonal, x1, y1, x2, y2, x3, y3, x4, y4);
}

/// @brief Sets the original image for *fiducials.
/// @param fiducials is the *Fiducials* object to use.
/// @param image is the new image to use as the original image.
///
/// *Fiducials__image_set*() will set the original image for *fiducials*
/// to *image*.

void Fiducials__image_set(Fiducials fiducials, CV_Image image) {
    fiducials->original_image = image;
}

/// @brief Is a HighGUI interface to show the current image.
/// @param fiducials is the *Fiducials* object that contains the image.
/// @param show is true to force the HighGUI interface to activate.
///
/// *Fiducials__image_show*() will cause an image to be shown with
/// at each of the various stages of the recognition cycle.  This
/// only occurs if *show* is *true*.
///
///  The character commands are:
///
/// * '/033' -- Escape from program.
/// * '+' -- View next stage in processing pipeline.
/// * '-' -- View previous stage in processing pipeline.
/// * '<' -- Goto beginning of processing pipeline.
/// * 'b' -- Toggle image blur.
/// * 'f' -- Flip fiducials allong X axis

//FIXME: Is there any point to having *show* set to false???!!!

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

/// @brief Create and return a *Fiducials* object.
/// @param original_image is the image to start with.
/// @param fiducials_create is a *Fiducials_Create* object that
///        specifies the various features to enable or disable.
///
/// *Fiducials__create*() creates and returns a *Fiducials* object
/// using the values in *fiduicials_create*.

//FIXME: Change this code so that the image size is determined from
// the first image that is processed.  This allows the image size
// to change in midstream!!!

Fiducials Fiducials__create(
  CV_Image original_image, Fiducials_Create fiducials_create)
{
    // Create *image_size*:
    Unsigned width = CV_Image__width_get(original_image);
    Unsigned height = CV_Image__height_get(original_image);
    CV_Size image_size = CV_Size__create(width, height);
    CV_Memory_Storage storage = CV_Memory_Storage__create(0);

    // Grab some values from *fiducials_create*:
    String_Const fiducials_path = fiducials_create->fiducials_path;
    String_Const lens_calibrate_file_name =
      fiducials_create->lens_calibrate_file_name;
    Memory announce_object = fiducials_create->announce_object;
    Fiducials_Arc_Announce_Routine arc_announce_routine =
      fiducials_create->arc_announce_routine;
    Fiducials_Location_Announce_Routine location_announce_routine =
      fiducials_create->location_announce_routine;
    Fiducials_Tag_Announce_Routine tag_announce_routine =
      fiducials_create->tag_announce_routine;
    Fiducials_Fiducial_Announce_Routine  fiducial_announce_routine = 
      fiducials_create->fiducial_announce_routine; 
    String_Const log_file_name = fiducials_create->log_file_name;
    String_Const map_base_name = fiducials_create->map_base_name;
    String_Const tag_heights_file_name =
      fiducials_create->tag_heights_file_name;

    // Get *log_file* open if *log_file_name* is not null:
    File log_file = stderr;
    if (log_file_name != (String_Const)0) {
	String full_log_file_name =
	  String__format("%s/%s", fiducials_path, log_file_name);
	log_file = File__open(log_file_name, "w");
	String__free(full_log_file_name);
    }
    File__format(log_file, "CV width=%d CV height = %d\n", width, height);

    Integer term_criteria_type =
      CV__term_criteria_iterations | CV__term_criteria_eps;

    // The north/west/south/east mappings must reside in static
    // memory rather than on the stack:

    static Logical north_mapping[64] = {
	//corner1	      corner0
	 0,  1,  2,  3,  4,  5,  6,  7,
	 8,  9, 10, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 50, 51, 52, 53, 54, 55,
	56, 57, 58, 59, 60, 61, 62, 63,
	//corner2	      corner3
    };

    static Logical west_mapping[64] = {
	//corner1	      corner0
	 7, 15, 23, 31, 39, 47, 55, 63,
	 6, 14, 22, 30, 38, 46, 54, 62,
	 5, 13, 21, 29, 37, 45, 53, 61,
	 4, 12, 20, 28, 36, 44, 52, 60,
	 3, 11, 19, 27, 35, 43, 51, 59,
	 2, 10, 18, 26, 34, 42, 50, 58,
	 1,  9, 17, 25, 33, 41, 49, 57,
	 0,  8, 16, 24, 32, 40, 48, 56,
	//corner2	      corner3
    };

    static Logical south_mapping[64] = {
	//corner1	      corner0
	63, 62, 61, 60, 59, 58, 57, 56,
	55, 54, 53, 52, 51, 50, 49, 48,
	47, 46, 45, 44, 43, 42, 41, 40,
	39, 38, 37, 36, 35, 34, 33, 32,
	31, 30, 29, 28, 27, 26, 25, 24,
	23, 22, 21, 20, 19, 18, 17, 16,
	15, 14, 13, 12, 11, 10,  9,  8,
	 7,  6,  5,  4,  3,  2,  1,  0,
	//corner2	      corner3
    };

    static Logical east_mapping[64] = {
	//corner1	      corner0
	56, 48, 40, 32, 24, 16,  8,  0,
	57, 49, 41, 33, 25, 17,  9,  1,
	58, 50, 42, 34, 26, 18, 10,  2,
	59, 51, 43, 35, 27, 19, 11,  3,
	60, 52, 44, 36, 28, 20, 12,  4,
	61, 53, 45, 37, 29, 21, 13,  5,
	62, 54, 46, 38, 30, 22, 14,  6,
	63, 55, 47, 39, 31, 23, 15,  7,
	//corner2	      corner3
    };

    static Logical north_mapping_flipped[64] = {
	//corner1	      corner0
	 7,  6,  5,  4,  3,  2,  1,  0,
	15, 14, 13, 12, 11, 10,  9,  8,
	23, 22, 21, 20, 19, 18, 17, 16,
	31, 30, 29, 28, 27, 26, 25, 24,
	39, 38, 37, 36, 35, 34, 33, 32,
	47, 46, 45, 44, 43, 42, 41, 40,
	55, 54, 53, 52, 51, 50, 49, 48,
	63, 62, 61, 60, 59, 58, 57, 56,
	 //corner2	      corner3
    };

    static Logical west_mapping_flipped[64] = {
	//corner1	      corner0
	63, 55, 47, 39, 31, 23, 15, 7,
	62, 54, 46, 38, 30, 22, 14, 6,
	61, 53, 45, 37, 29, 21, 13, 5,
	60, 52, 44, 36, 28, 20, 12, 4,
	59, 51, 43, 35, 27, 19, 11, 3,
	58, 50, 42, 34, 26, 18, 10, 2,
	57, 49, 41, 33, 25, 17,  9, 1,
	56, 48, 40, 32, 24, 16,  8, 0,
	//corner2	      corner3
    };

    static Logical south_mapping_flipped[64] = {
	//corner1	      corner0
	56, 57, 58, 59, 60, 61, 62, 63, 
	48, 49, 50, 51, 52, 53, 54, 55,
	40, 41, 42, 43, 44, 45, 46, 47,
	32, 33, 34, 35, 36, 37, 38, 39,
	24, 25, 26, 27, 28, 29, 30, 31,
	16, 17, 18, 19, 20, 21, 22, 23,
	 8,  9, 10, 11, 12, 13, 14, 15, 
	 0,  1,  2,  3,  4,  5,  6,  7,
	//corner2	      corner3
    };

    static Logical east_mapping_flipped[64] = {
	//corner1	      corner0
	 0,  8, 16, 24, 32, 40, 48, 56,
	 1,  9, 17, 25, 33, 41, 49, 57,
	 2, 10, 18, 26, 34, 42, 50, 58,
	 3, 11, 19, 27, 35, 43, 51, 59,
	 4, 13, 20, 28, 36, 44, 52, 60,
	 5, 13, 21, 29, 37, 45, 53, 61,
	 6, 14, 22, 30, 38, 46, 54, 62,
	 7, 15, 23, 31, 39, 47, 55, 63,
	 //corner2	      corner3
    };

    // The north/west/south/east mappings must reside in static
    // memory rather than on the stack:

    static Logical *mappings[4] = {
	&north_mapping_flipped[0],
	&west_mapping_flipped[0],
	&south_mapping_flipped[0],
	&east_mapping_flipped[0],
    };

    //for (Unsigned index = 0; index < 4; index++) {
    //	File__format(log_file, "mappings[%d]=0x%x\n", index, mappings[index]);
    //}

    CV_Image map_x = (CV_Image)0;
    CV_Image map_y = (CV_Image)0;
    if (lens_calibrate_file_name != (String)0) {
	String full_lens_calibrate_file_name =
	  String__format("%s/%s", fiducials_path, lens_calibrate_file_name);
	assert (CV__undistortion_setup(
	  full_lens_calibrate_file_name, width, height, &map_x, &map_y) == 0);
	String__free(full_lens_calibrate_file_name);
    }

    // Create the *map*:
    Map map = Map__create(fiducials_path, map_base_name, announce_object,
      arc_announce_routine, tag_announce_routine,
      tag_heights_file_name, "Fiducials__new:Map__create");

    Fiducials_Results results =
      Memory__new(Fiducials_Results, "Fiducials__create");
    results->map_changed = (Logical)0;

    // Create and load *fiducials*:
    Fiducials fiducials = Memory__new(Fiducials, "Fiducials__create");
    fiducials->arc_announce_routine = arc_announce_routine;
    if (fiducial_announce_routine != NULL) 
       fiducials->fiducial_announce_routine = fiducial_announce_routine;
    else
       fiducials->fiducial_announce_routine = Fiducials__fiducial_announce;
    fiducials->announce_object = announce_object;
    fiducials->blue = CV_Scalar__rgb(0.0, 0.0, 1.0);
    fiducials->blur = (Logical)1;
    fiducials->camera_tags =
      List__new("Fiducials__create:List__new:camera_tags"); // <Camera_Tag>
    fiducials->camera_tags_pool =
      List__new("Fiducials__create:List__new:camera_tags_pool"); // <Camera_Tag>
    fiducials->corners = CV_Point2D32F_Vector__create(4);
    fiducials->current_visibles =
      List__new("Fiducials__create:List_new:current_visibles"); // Tag
    fiducials->cyan = CV_Scalar__rgb(0.0, 1.0, 1.0);
    fiducials->debug_image = CV_Image__create(image_size, CV__depth_8u, 3);
    fiducials->debug_index = 0;
    fiducials->edge_image = CV_Image__create(image_size, CV__depth_8u, 1);
    fiducials->fec = FEC__create(8, 4, 4);
    fiducials->gray_image = CV_Image__create(image_size, CV__depth_8u, 1);
    fiducials->green = CV_Scalar__rgb(0.0, 255.0, 0.0);
    fiducials->image_size = image_size;
    fiducials->last_x = 0.0;
    fiducials->last_y = 0.0;
    fiducials->location_announce_routine = location_announce_routine;
    fiducials->locations =
      List__new("Fiducials__create:List__new:locations"); // <Location>
    fiducials->locations_path =
      List__new("Fiducials__create:List__new:locations_path"); // <Location>
    fiducials->log_file = log_file;
    fiducials->map = map;
    fiducials->map_x = map_x;
    fiducials->map_y = map_y;
    fiducials->mappings = &mappings[0];
    fiducials->origin = CV_Point__create(0, 0);
    fiducials->original_image = original_image;
    fiducials->path = fiducials_path;
    fiducials->previous_visibles =
      List__new("Fiducials__create:List__new:previous_visibles"); // Tag
    fiducials->purple = CV_Scalar__rgb(255.0, 0.0, 255.0);
    fiducials->red = CV_Scalar__rgb(255.0, 0.0, 0.0);
    fiducials->references = CV_Point2D32F_Vector__create(8);
    fiducials->results = results;
    fiducials->sample_points = CV_Point2D32F_Vector__create(64);
    fiducials->size_5x5 = CV_Size__create(5, 5);
    fiducials->size_m1xm1 = CV_Size__create(-1, -1);
    fiducials->sequence_number = 0;
    fiducials->storage = storage;
    fiducials->temporary_gray_image =
      CV_Image__create(image_size, CV__depth_8u, 1);
    fiducials->weights_index = 0;
    fiducials->term_criteria = 
      CV_Term_Criteria__create(term_criteria_type, 5, 0.2);
    fiducials->y_flip = (Logical)0;
    fiducials->black = CV_Scalar__rgb(0, 0, 0);

    return fiducials;
}

/// @brief will release the storage associated with *fiducials*.
/// @param fiducials is the *Fiducials* object to release.
///
/// *Fiducials__free*() releases the storage associated with *fiducials*.

void Fiducials__free(Fiducials fiducials) {
    // Write the map out if it changed:
    Map__save(fiducials->map);

    // Free up some *CV_Scalar* colors:
    CV_Scalar__free(fiducials->blue);
    CV_Scalar__free(fiducials->cyan);
    CV_Scalar__free(fiducials->green);
    CV_Scalar__free(fiducials->purple);
    CV_Scalar__free(fiducials->red);
    CV_Scalar__free(fiducials->black);

    // Free up some *SV_Size* objects:
    CV_Size__free(fiducials->image_size);
    CV_Size__free(fiducials->size_5x5);
    CV_Size__free(fiducials->size_m1xm1);

    // Free up the storage associated with *locations*:
    List /* <Location> */ locations = fiducials->locations;
    Unsigned locations_size = List__size(locations);
    for (Unsigned index = 0; index < locations_size; index++) {
	Location location = List__fetch(locations, index);
	// Kludge: memory double free?!!!
	//Location__free(location);
    }

    List /* <Location> */ locations_path = fiducials->locations_path;
    Unsigned locations_path_size = List__size(locations_path);
    for (Unsigned index = 0; index < locations_path_size; index++) {
	Location location = List__fetch(locations_path, index);
	// Kludge: memory double free?!!!
	//Location__free(location);
    }

    // Free up the storage associated with *camera_tags_pool*:
    List /* <Camera_Tag> */ camera_tags_pool = fiducials->camera_tags_pool;
    Unsigned pool_size = List__size(camera_tags_pool);
    for (Unsigned index = 0; index < pool_size; index++) {
	Camera_Tag camera_tag =
	  (Camera_Tag)List__fetch(camera_tags_pool, index);
	Camera_Tag__free(camera_tag);
    }

    // Free up the *List*'s:
    List__free(camera_tags_pool);
    List__free(fiducials->camera_tags);
    List__free(fiducials->current_visibles);
    List__free(fiducials->previous_visibles);
    List__free(locations);
    List__free(locations_path);

    // Relaase the *Map*:
    Map__free(fiducials->map);

    // Finally release *fiducials*:
    Memory__free((Memory)fiducials);
}

/// @brief Force the map associated with *fiducials* to be saved.
/// @param fiducials is the *Fiducials* object to save the map of.
///
/// *Fiducials__map_save*() will cause the map associated with
/// *fiducials* to be saved.

void Fiducials__map_save(Fiducials fiducials) {
    Map__save(fiducials->map);
}

/// @brief Process the current image associated with *fiducials*.
/// @param fiducials is the *Fiducials* object to use.
/// @returns a *Fiducials_Results* that contains information about
///          how the processing worked.
///
/// *Fiducials__process*() will process *fiducials* to determine
/// the robot location.

Fiducials_Results Fiducials__process(Fiducials fiducials) {
    // Clear *storage*:
    CV_Memory_Storage storage = fiducials->storage;
    CV_Memory_Storage__clear(storage);

    // Grab some values from *fiducials*:
    List /*<Tag>*/ current_visibles = fiducials->current_visibles;
    CV_Image debug_image = fiducials->debug_image;
    Unsigned debug_index = fiducials->debug_index;
    CV_Image edge_image = fiducials->edge_image;
    CV_Image gray_image = fiducials->gray_image;
    List /*<Location>*/ locations = fiducials->locations;
    File log_file = fiducials->log_file;
    CV_Image original_image = fiducials->original_image;
    List /*<Tag>*/ previous_visibles = fiducials->previous_visibles;
    Fiducials_Results results = fiducials->results;
    CV_Image temporary_gray_image = fiducials->temporary_gray_image;
    Fiducials_Location_Announce_Routine location_announce_routine =
      fiducials->location_announce_routine;
    Unsigned sequence_number = fiducials->sequence_number++;

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
	File__format(log_file, "no contours found\n");
    }

    // For *debug_index* 4, show the *edge_image* *contours*:
    if (debug_index == 5) {
	//File__format(log_file, "Draw red contours\n");
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
	//File__format(log_file, "contours_count=%d\n", contours_count);

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
	    //File__format(log_file, "Draw green contours\n");
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
	    //File__format(log_file, "Have 4 sides > 500i\n");

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
		    //File__format(log_file,
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
		    File__format(log_file,
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
		    File__format(log_file, "ref[%d:%d]:%d\n", x, y, value);
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
		    //File__format(log_file,
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
			File__format(log_file,
			  "dir=%d Tag[0]=0x%x Tag[1]=0x%x\n",
			  direction_index, tag_bytes[0], tag_bytes[1]);
		    }

		    // Now we need to do some FEC (Forward Error Correction):
		    FEC fec = fiducials->fec;
		    if (FEC__correct(fec, tag_bytes, 8)) {
			// We passed FEC:
			if (debug_index == 11) {
			    File__format(log_file, "FEC correct\n");
			}

			// Now see if the two CRC's match:
			Unsigned computed_crc = CRC__compute(tag_bytes, 2);
			Unsigned tag_crc = (tag_bytes[3] << 8) | tag_bytes[2];
			if (computed_crc == tag_crc) {
			    // Yippee!!! We have a tag:
			    // Compute {tag_id} from the the first two bytes
			    // of {tag_bytes}:
			    Unsigned tag_id =
			      (tag_bytes[1] << 8) | tag_bytes[0];

			    if (debug_index == 11) {
				File__format(log_file,
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

			    double vertices[4][2];
			    for (Unsigned index = 0; index < 4; index++) {
			      CV_Point2D32F pt = CV_Point2D32F_Vector__fetch1(corners, index);
			      vertices[index][0] = pt->x;
			      vertices[index][1] = pt->y;
			    }			    
                            fiducials->fiducial_announce_routine(
                                fiducials->announce_object, tag_id,
				direction_index, tag->world_diagonal,
                                vertices[0][0], vertices[0][1],
				vertices[1][0], vertices[1][1],
                                vertices[2][0], vertices[2][1],
                                vertices[3][0], vertices[3][1]);

			    if (debug_index == 11) {
				Camera_Tag__initialize(camera_tag, tag,
				  direction_index, corners, debug_image);
			    } else {
				Camera_Tag__initialize(camera_tag, tag,
				  direction_index, corners, (CV_Image)0);
			    }
			    List__append(current_visibles, (Memory)tag,
			      "Fiduicals__create:List_append:current_visibles");
			    File__format(log_file, "Tag: %d x=%f y=%f\n",
			      tag->id, tag->x, tag->y);

			    // Record the maximum *camera_diagonal*:
			    Double camera_diagonal = camera_tag->diagonal;
			    Double diagonal =
			      camera_diagonal;
			    if (diagonal  > tag->diagonal) {
				tag->diagonal = diagonal;
                                tag->updated = (Logical)1;
			    }

			    // Append *camera_tag* to *camera_tags*:
			    List__append(camera_tags, (Memory)camera_tag,
			      "Fiducials__Create:List__append:camera_tags");
			    //File__format(log_file,
			    //  "Found %d\n", camera_tag->tag->id);
			}
		    }
		}
	    }
	}
    }

    // Just for consistency sort *camera_tags*:
    List__sort(camera_tags, (List__Compare__Routine)Camera_Tag__compare);

    // Sweep through all *camera_tag* pairs to generate associated *Arc*'s:
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
		if (Map__arc_update(map,
		  camera_tag1, camera_tag2, gray_image, sequence_number) > 0) {
		    results->map_changed = (Logical)1;
		}
	    }
	}
    }

    List__trim(locations, 0);
    results->image_interesting = (Logical)0;
    if (camera_tags_size > 0) {
	Double pi = 3.14159265358979323846264;
	Unsigned half_width = CV_Image__width_get(gray_image) >> 1;
	Unsigned half_height = CV_Image__height_get(gray_image) >> 1;
	//File__format(log_file,
	//  "half_width=%d half_height=%d\n", half_width, half_height);
	for (Unsigned index = 0; index < camera_tags_size; index++) {
	    Camera_Tag camera_tag = (Camera_Tag)List__fetch(camera_tags, index);
	    Tag tag = camera_tag->tag;
	    //File__format(log_file,
	    //  "[%d]:tag_id=%d tag_x=%f tag_y=%f tag_twist=%f\n",
	    //  index, tag->id, tag->x, tag->y, tag->twist * 180.0 / pi);
	    Double camera_dx = camera_tag->x - half_width;
	    Double camera_dy = camera_tag->y - half_height;
	    //File__format(log_file,
	    //  "[%d]:camera_dx=%f camera_dy=%f camera_twist=%f\n",
	    //  index, camera_dx, camera_dy, camera_tag->twist * 180.0 / pi);
	    Double polar_distance = Double__square_root(
	      camera_dx * camera_dx + camera_dy * camera_dy);
	    Double polar_angle = Double__arc_tangent2(camera_dy, camera_dx);
	    //File__format(log_file,
	    //  "[%d]:polar_distance=%f polar_angle=%f\n", index,
	    //  polar_distance, polar_angle * 180.0 / pi);
	    Double floor_distance = 
	      polar_distance * tag->world_diagonal / tag->diagonal;
	    Double angle =
	      Double__angle_normalize(polar_angle + pi - camera_tag->twist);
	    //File__format(log_file,
	    //  "[%d]:floor_distance=%f angle=%f\n",
	    //  index, floor_distance, angle * 180.0 / pi);
	    Double x = tag->x + floor_distance * Double__cosine(angle);
	    Double y = tag->y + floor_distance * Double__sine(angle);
	    Double bearing =
	      Double__angle_normalize(camera_tag->twist + tag->twist);

	    // FIXME: Kludge,  There is a sign error somewhere in the code
	    // causes the "sign" on the X axis to be inverted.  We kludge
	    // around the problem with the following disgusting code:
	    bearing = Double__angle_normalize(bearing - pi / 2.0);
	    bearing = -bearing;
	    bearing = Double__angle_normalize(bearing + pi / 2.0);

	    //File__format(log_file, "[%d]:x=%f:y=%f:bearing=%f\n",
	    //  index, x, y, bearing * 180.0 / pi);
	    Unsigned location_index = List__size(locations);
	    Location location = Location__create(tag->id,
	      x, y, bearing, floor_distance, location_index);
	    List__append(locations,
	      (Memory)location, "Fiducials__process:locations");
	}

	// Compute closest location:
	Location closest_location = (Location)0;
	Unsigned locations_size = List__size(locations);
	for (Unsigned index = 0; index < locations_size; index++) {
	  Location location = (Location)List__fetch(locations, index);
	    if (closest_location == (Location)0) {
		closest_location = location;
	    } else {
		if (location->goodness < closest_location->goodness) {
		    closest_location = location;
		}
	    }
	}

	if (closest_location != (Location)0) {
	    List /* <Location> */ locations_path = fiducials->locations_path;
	    List__append(locations_path, (Memory)closest_location,
	     "Fiducials__create:List__append:locations");
	    File__format(log_file,
	      "Fiducials__process:Location: " /* + */
	      "x=%f y=%f bearing=%f goodness=%f index=%d\n",
	      closest_location->x, closest_location->y,
	      closest_location->bearing * 180.0 / pi,
	      closest_location->goodness, closest_location->index);

	    Double change_dx = closest_location->x - fiducials->last_x;
	    Double change_dy = closest_location->y - fiducials->last_y;
	    Double change = Double__square_root(
	      change_dx * change_dx + change_dy * change_dy);
	    if (change > 0.1) {
		results->image_interesting = (Logical)1;
	    }
	    fiducials->last_x = closest_location->x;
	    fiducials->last_y = closest_location->y;

	    // send rviz marker message here
	    File__format(log_file,
	      "Location: id=%d x=%f y=%f bearing=%f\n",
	      closest_location->id, closest_location->x, closest_location->y,
	      closest_location->bearing);
	    location_announce_routine(fiducials->announce_object,
	      closest_location->id, closest_location->x, closest_location->y,
	      /* z */ 0.0, closest_location->bearing);
	}
    }

    // Visit each *current_tag* in *current_visibles*:
    Unsigned current_visibles_size = List__size(current_visibles);
    for (Unsigned current_visibles_index = 0;
      current_visibles_index < current_visibles_size;
      current_visibles_index++) {
	Tag current_visible =
	  (Tag)List__fetch(current_visibles, current_visibles_index);
	//File__format(log_file, "Current[%d]:%d\n",
	//  current_visibles_index, current_visible->id);

	// Always announce *current_visible* as visible:
	current_visible->visible = (Logical)1;
        if( current_visible->updated ) {
	    Map__tag_announce(map, current_visible,
	        (Logical)1, original_image, sequence_number);
            current_visible->updated = (Logical)0;
        }
    }

    // Identifiy tags that are no longer visible:
    Unsigned previous_visibles_size = List__size(previous_visibles);
    for (Unsigned previous_visibles_index = 0;
       previous_visibles_index < previous_visibles_size;
       previous_visibles_index++) {
	Tag previous_visible =
	  (Tag)List__fetch(previous_visibles, previous_visibles_index);
	//File__format(log_file, "Previous[%d]:%d\n",
	//  previous_visibles_index, previous_visible->id);

	// Now look to see if *previous_visible* is in *current_visibles*:
	Tag current_visible = (Tag)0;
	for (Unsigned current_visibles_index = 0;
	  current_visibles_index < current_visibles_size;
	  current_visibles_index++) {
	    current_visible = 
	      (Tag)List__fetch(current_visibles, current_visibles_index);
	    if (current_visible == previous_visible) {
		break;
	    }
	    current_visible = (Tag)0;
	}	

	// *current_visible* is null if it was not found:
	if (current_visible == (Tag)0) {
	    // Not found => announce the tag as no longer visible:
	    previous_visible->visible = (Logical)0;
	    Map__tag_announce(map,
	      previous_visible, (Logical)0, original_image, sequence_number);
	}
    }
    // Clear *previous_visibles* and swap *current_visible* with
    // *previous_visibles*:
    List__trim(previous_visibles, 0);
    fiducials->current_visibles = previous_visibles;
    fiducials->previous_visibles = current_visibles;
    //File__format(log_file, "current_visibles=0x%x previous_visibles=0x%x\n",
    //  current_visibles, previous_visibles);

    // Clean out *camera_tags*:
    List__all_append(fiducials->camera_tags_pool, camera_tags);
    List__trim(camera_tags, 0);

    // Flip the debug image:
    if (fiducials->y_flip) {
	CV_Image__flip(debug_image, debug_image, 0);
    }

    // Update the map:
    Map__update(map, original_image, sequence_number);

    File__format(log_file, "\n");
    File__flush(log_file);

    return results;
}

/// @brief Helper routine to sample a point from the image in *fiducials*.
/// @param fiducials is the *Fiducials* object that contains the image.
/// @param point is the point location to sample.
/// @returns weighted sample value.
///
/// *Fiducials__point_sample*() will return a weighted sample value for
/// *point* in the image associated with *fiducials*.  The weight algorithm
/// is controlled by the *weights_index* field of *fiducials*.  The returned
/// value is between 0 (black) to 255 (white).

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

/// @brief Force *corners* to be counter-clockwise.
/// @param corners is the list of 4 fiducial corners.
///
/// *CV_Point2D32F_Vector__corners_normalize*() will force *corners*
/// to be counter-clockwise.  Note there is no check to ensure that
/// *corners* actually has 4 values in it.

//FIXME: Does this routine belong in CV.c???!!!  
//FIXME: Should this be merged with *CV_Point2D32F_Vector__is_clockwise*???!!!

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

/// @brief Return true if *corners* is in a clockwise direction.
/// @param corners is a vector of 4 fiducials corners to test.
/// @returns true if the corners are clockwiase and false otherwise.
///
/// *CV_Point2D32F_Vector__is_clockwise*() will return true if the 4 fiducial
/// corners in *corners* are clockwise and false otherwise.

Logical CV_Point2D32F_Vector__is_clockwise(CV_Point2D32F_Vector corners) {

    // Extract the three corners:
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

/// @brief Return 8 sample locations to determine if a quadralateral is
///        worth testing for quadralateral'ness.
/// @param fiducials is the *Fiducals* object that contains the image.
/// @param corners is the 4 potential fiducial corners.
/// @returns a vector 8 places to test for ficial'ness.
///
/// *Fiducials__references_compute*() 4 corner points in *corners* to
/// compute 8 reference points that are returned.  The first 4 reference
/// points will be just outside of the quadrateral formed by *corners*
/// (i.e. the white bounding box) and the last 4 reference points are
/// on the inside (i.e. the black bounding box).  The returned vector
/// is perminately allocated in *fiducials*, so it does not need to have
/// it storage released.

CV_Point2D32F_Vector Fiducials__references_compute(
  Fiducials fiducials, CV_Point2D32F_Vector corners) {

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
    // a line parrallel to the other side of the quadralatal:
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

/// @brief Return the maximum value of the points in *points*.
/// @param fiducials is the *Fiducials* object that contains the image.
/// @param points is the vector of points to sample.
/// @param start_index is the first index to start with.
/// @param end_index is the last index to end with.
/// @returns the maximum sampled value.
///
///
/// *Fiducials__points_maximum*() will sweep from *start_index* to
/// *end_index* through *points*.  Using each selected point in *points*},
/// the corresponding value in *image* is sampled.  The minimum of the
/// sampled point is returned.

Integer Fiducials__points_maximum(Fiducials fiducials,
  CV_Point2D32F_Vector points, Unsigned start_index, Unsigned end_index) {

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

/// @brief Return the minimum value of the points in *points*.
/// @param fiducials is the *Fiducials* object that contains the image.
/// @param points is the vector of points to sample.
/// @param start_index is the first index to start with.
/// @param end_index is the last index to end with.
/// @returns the minimum sampled value.
///
/// *Fiducials__points_minimum*() will sweep from *start_index* to
/// *end_index* through *points*.  Using each selected point in *points*},
/// the corresponding value in *image* is sampled.  The minimum of the
/// sampled point is returned.

Integer Fiducials__points_minimum(Fiducials fiducials,
  CV_Point2D32F_Vector points, Unsigned start_index, Unsigned end_index) {

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

/// @brief Compute the fiducial locations to sample using *corners*
/// @param corners is the the 4 of the fiducials.
/// @param sample_points is the 64 vector of points that are computed.
///
/// *Fiducials__sample_points_compute*() will use the 4 corners in 
/// *corners* as a quadralateral to compute an 8 by 8 grid of tag
/// bit sample points and store the results into the the 64
/// preallocated *CV_Point2D32F* objects in *sample_points*.
/// The quadralateral must be convex and in the counter-clockwise
/// direction.  Bit 0 will be closest to corners[1], bit 7 will be
/// closest to corners[0], bit 56 closest to corners[2] and bit 63
/// closest to corners[3].

//FIXME: sample points comes from *fiducials*, so shouldn't have
// have *fiducials* as a argument instead of *sample_points*???!!!

void Fiducials__sample_points_compute(
  CV_Point2D32F_Vector corners, CV_Point2D32F_Vector sample_points) {

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

// Used in *Fiducials__sample_points*() for debugging:
//
//void Fiducials__sample_points_helper(
//  String_Const label, CV_Point2D32F corner, CV_Point2D32F sample_point) {
//    Double corner_x = CV_Point2D32F__x_get(corner);
//    Double corner_y = CV_Point2D32F__y_get(corner);
//    Double sample_point_x = CV_Point2D32F__x_get(sample_point);
//    Double sample_point_y = CV_Point2D32F__y_get(sample_point);
//    File__format(stderr, "Label: %s corner: %f:%f sample_point %f:%f\n",
//      label, (Integer)corner_x, (Integer)corner_y,
//      (Integer)sample_point_x, (Integer)sample_point_y);
//}

/// @brief Print out tag update information.
/// @param announce_object is an opaque object from *Map*->*announce_object*.
/// @param id is the tag id.
/// @param x is the tag X location.
/// @param y is the tag Y location.
/// @param z is the tag Z location.
/// @param twist is the tag twist in radians.
/// @param diagonal is the tag diagonal distance.
/// @param distance_per_pixel is the distance per pixel.
/// @param visible is (*Logical*)1 if the tag is currently in camera
///        field of view, and (*Logical*)0 otherwise.
/// @param hop_count is the hop count along the spanning tree to the origin.
///
/// *Fiducials_tag_announce*() is a tag announce routine that can be
/// fed into *Fiducials__create*() as a routine to call each time a
/// tag is updated.

void Fiducials__tag_announce(void *announce_object, Integer id,
  Double x, Double y, Double z, Double twist, Double diagonal,
  Double distance_per_pixel, Logical visible, Integer hop_count) {
    String visible_text = "";
    if (!visible) {
	visible_text = "*** No longer visible ***";
    }
    File__format(stderr, "id=%d x=%f y=%f twist=%f %s\n",
      id, x, y, twist, visible_text);
}

static struct Fiducials_Create__Struct fiducials_create_struct =
{
    (String_Const)0,				// fiducials_path
    (String_Const)0,				// lens_calibrate_file_name
    (void *)0,					// announce_object
    (Fiducials_Arc_Announce_Routine)0,		// arc_announce_routine
    (Fiducials_Location_Announce_Routine)0,	// location_announce_routine
    (Fiducials_Tag_Announce_Routine)0,		// tag_announce_routine
    (Fiducials_Fiducial_Announce_Routine)0,    	// fiducial_announce_routine
    (String_Const)0,				// log_file_name
    (String_Const)0,				// map_base_name
    (String_Const)0,				// tag_heights_file_name
};

/// @brief Returns the one and only *Fiducials_Create* object.
/// @returns the one and only *Fiducials_Create* object.
///
/// *Fiducials_Create__one_and_only*() will return the one and only
/// *Fiducials_Create* object.  This object needs to be initalized
/// prior to calling *Fiduciasl__create*().

Fiducials_Create Fiducials_Create__one_and_only(void)
{
    return &fiducials_create_struct;
}
