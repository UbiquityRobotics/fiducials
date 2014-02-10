// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

// This program will display a grey scale image on the screen in real time.

#include <assert.h>

// If *PTGREY* is not defined, we make sure it is defined as 0:
#if !defined(PTGREY)
#define PTGREY 0
#endif

// The Point Gray cameras have their own library interface to access them.
#if PTGREY
#include "C/FlyCapture2_C.h"
#endif // PTGREY

#include "Character.h"
#include "CV.h"
#include "FC2.h"
#include "File.h"
#include "Fiducials.h"
#include "High_GUI2.h"
#include "Integer.h"
#include "Memory.h"
#include "String.h"
#include "Unsigned.h"

/// @brief A video display routine that can capture images.
/// @param arguments_size is the number of command line arguments (plus 1.)
/// @param arguments is the command line arguments vector.
/// @returns 0 for success and 1 for failure.
///
/// *main*() opens a camera (or video file) and allows the user to capture
/// images by typing the [space] key.

int main(int arguments_size, char * arguments[]) {
    if (arguments_size <= 1) {
	// No arguments; let the user know the usage:
	File__format(stderr,
	  "Usage: Video_Capture camera_number [capture_base_name]\n");
	return 1;
    } else {
        // Deal with the command line *arguments*:
	String capture_base_name = "video_capture";
	String argument1 = arguments[1];
	if (arguments_size > 2) {
	    // Override *capture_base_name*:
	    capture_base_name = arguments[2];
	}

	// Figure whether to open a video file or a camera;
	Unsigned camera_number = 0;
	if (Character__is_decimal_digit(argument1[0])) {
	    // Open the camera:
	    camera_number = String__to_unsigned(argument1);
	}

	// Print FlyCapture2 Library version:
	FC2_Version version = FC2__library_version_get();
	File__format(stderr, "FlyCapture2 Library Version: %d.%d.%d.%d\n",
	  version->major, version->minor, version->type, version->build);

	// Get a camera camera:
	FC2_Camera camera = FC2_Camera__create();

	// Figure out how many cameras are connected:
	Unsigned number_of_cameras = FC2_Camera__number_of_cameras_get(camera);

	// Make sure we have enough cameras:
	if (camera_number < number_of_cameras) {
	    // Get the *cammera_identifier* for *camera_number*:
	    FC2_Camera_Identifier camera_identifier =
	      FC2_Camera__identifier_fetch(camera, camera_number);

	    // Connect to *camera_indentifier*:
	    FC2_Camera__connect(camera, camera_identifier);
	
	    // Print out some *camera_information*:
	    FC2_Camera_Information camera_information =
	      FC2_Camera__information_get(camera);
	    File__format(stderr,
	      "Serial number %u\n", camera_information->serialNumber);
	    File__format(stderr,
	      "Camera model %s\n", camera_information->modelName);
	    File__format(stderr,
	      "Camera vendor %s\n", camera_information->vendorName);
	    File__format(stderr, "Sensor %s\n", camera_information->sensorInfo);
	    File__format(stderr,
	      "Resolution %s\n", camera_information->sensorResolution);
	    File__format(stderr,
	      "Firmware version %s\n", camera_information->firmwareVersion);
	    File__format(stderr,
	      "Firmware build time %s\n",
	      camera_information->firmwareBuildTime);

	    // Start up *camera*;
	    FC2_Camera__capture_start(camera);

	    // Allocate a *camera_image* and *converted_image*:
	    FC2_Image camera_image = FC2_Image__create();
	    FC2_Image converted_image = FC2_Image__create();

	    // Create the window to display the video into:
	    String window_name = "Video_Capture";
	    CV__named_window(window_name, CV__window_auto_size);
	    //cvResizeWindow(window_name, 1000, 800);

	    Fiducials fiducials = (Fiducials)0;

	    // Do a video loop:
	    CV_Image display_image = (CV_Image)0;
	    Unsigned capture_number = 0;
	    while (1) {
		// Retrieve *camera_image* from *camera*:
		FC2_Camera__image_retrieve(camera, camera_image);

		// For some reason, converting the image from grey to color
		// causes the frame rate to dramatically increase.  This is
		// a mystery to us, but since it works, we do it:
		FC2_Image__convert(camera_image,
		  converted_image, FC2_PIXEL_FORMAT_BGR);

		// The first time through, we allocate *display_image*:
		if (display_image == (CV_Image)0){
		    // Grab some values out of *camera_image*:
		    Unsigned columns = converted_image->cols;
		    Unsigned rows = converted_image->rows;
		    Unsigned stride = converted_image->stride;
		    Unsigned data_size = converted_image->dataSize;
		    Memory image_data = FC2_Image__data_get(converted_image);

		    // Print some stuff for debugging:
		    File__format(stderr, "columns: %d\n", columns);
		    File__format(stderr, "rows: %d\n", rows);
		    File__format(stderr, "stride: %d\n", stride);
		    File__format(stderr, "data_size: %d\n", data_size);
		    File__format(stderr, "image_data: 0x%x\n", image_data);

		    // Allocate *display_image* and make it share *image_data*
		    // with the ...
		    CV_Size display_image_size = CV_Size__create(columns, rows);
		    display_image =
		      CV_Image__header_create(display_image_size,
		      IPL_DEPTH_8U, 3);
		    display_image->imageData = image_data;

		    fiducials = Fiducials__create(display_image,
		      (String_Const)0, (void *)0, Fiducials__arc_announce,
		      Fiducials__location_announce, Map__tag_announce,
		      (String_Const)0, (String_Const)0, (String_Const)0);
		    fiducials->debug_index = 11;
		}

		// Show the image:
		//CV_Image__show(display_image, window_name);
		Fiducials__process(fiducials);
		CV_Image__show(fiducials->debug_image, window_name);

		// Deal with character input key stroke:
		Character character = CV__wait_key(1) & 0xff;
		if (character == '\033') {
		    // [Esc] key causes program to escape:
		    break;
		} else if (character == '+') {
		    fiducials->debug_index += 1;
		    File__format(stderr,
		      "debug_index=%d\n", fiducials->debug_index);
		} else if (character == '-') {
		    fiducials->debug_index -= 1;
		    File__format(stderr,
		      "debug_index=%d\n", fiducials->debug_index);
		} else if (character == '>') {
		    fiducials->weights_index += 1;
		    File__format(stderr,
		      "weights_index=%d\n", fiducials->weights_index);
		} else if (character == '<') {
		    fiducials->weights_index -= 1;
		    File__format(stderr,
		      "weights_index=%d\n", fiducials->weights_index);
		} else if (character == ' ') {
		    // Write out image out to file system as a .pnm file:

		    // For some reason String__format() does not work
		    // on CygWin running on Windows 7 right now:
		    char file_name[200];
		    (void)sprintf(file_name, "%s-%02d.pnm",
		      capture_base_name, capture_number);
		    CV_Image__pnm_write(display_image, file_name);
		    File__format(stderr,
		      "Wrote display_image out to file '%s'\n", file_name);
		    capture_number += 1;
		    //String__free(file_name);
		}
	    }

	    // Release unneeded storage:
	    CV__destroy_window(window_name);
	    //FIXME: Release *display_image*:
	    //FC2_Camera__free(camera);
	    Memory__free((Memory)camera_information);
	    Memory__free((Memory)camera_identifier);
	    FC2_Image__free(camera_image);
	    FC2_Image__free(converted_image);
	} else {
	    File__format(stderr, "Camera %d is not availble.\n", camera_number);
	}
    }

    return 0;
}
