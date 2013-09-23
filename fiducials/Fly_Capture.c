// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

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

Integer main(Integer arguments_size, String arguments[]) {
    if (arguments_size <= 1) {
	// No arguments; let the user know the usage:
	File__format(stderr,
	  "Usage: Video_Capture camera_number [capture_base_name]\n");
	return 1;
    } else {
        // Deal with the *arguments*:
	String capture_base_name = "video_capture";
	String argument1 = arguments[1];
	if (arguments_size > 2) {
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

	// Get a camera context:
	FC2_Context context = FC2__context_create();
	assert(context != (FC2_Context)0);

	// Figure out how many cameras are connected:
	Integer number_of_cameras = FC2__number_of_cameras_get(context);
	assert(number_of_cameras > 0);

	// Get the *cammera_identifier* for *camera_number*:
	FC2_Camera_Identifier camera_identifier =
	  FC2__camera_fetch(context, camera_number);
	assert(camera_identifier != (FC2_Camera_Identifier)0);

	// Connect to *camera_indentifier*:
	assert(FC2__camera_connect(context, camera_identifier) == FC2_ERROR_OK);
	Memory__free((Memory)camera_identifier);
	
	// Print out some *camera_information*:
	FC2_Camera_Information camera_information =
	  FC2__camera_information_get(context);
	assert(camera_information != (FC2_Camera_Information)0);
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
	  "Firmware build time %s\n", camera_information->firmwareBuildTime);

	// Start up the camera;
	assert(FC2__capture_start(context) == FC2_ERROR_OK);

	// Allocate a *camera_image*:
	FC2_Image camera_image = FC2__image_create();

	// Create the window to display the video into:
	String window_name = "Video_Capture";
	CV__named_window(window_name, CV__window_auto_size);

	// Do a video loop:
	CV_Image display_image = (CV_Image)0;
	Unsigned capture_number = 0;
	while (1) {
	    // Retrieve the image
	    FC2__image_retrieve(context, camera_image);

	    // The first time through, we allocate *display_image*:
	    if (display_image == (CV_Image)0){
		// Grab some values out of *camera_image*:
		Unsigned columns = camera_image->cols;
		Unsigned rows = camera_image->rows;
		Unsigned stride = camera_image->stride;
		Unsigned data_size = camera_image->dataSize;
		unsigned char *image_data = FC2__image_data_get(camera_image);

		// Print some stuff for debugging:
		File__format(stderr, "columns: %d\n", columns);
		File__format(stderr, "rows: %d\n", rows);
		File__format(stderr, "stride: %d\n", stride);
		File__format(stderr, "data_size: %d\n", data_size);
		File__format(stderr, "image_data: 0x%x\n", image_data);

		// Allocate *display_image* and make it share *image_data*
		// with the 
		CV_Size display_image_size = CV_Size__create(columns, rows);
		display_image =
		  CV_Image__header_create(display_image_size, IPL_DEPTH_8U, 1);
		display_image->imageData = image_data;
	    }

	    // Show the image:
	    CV_Image__show(display_image, window_name);

	    // Deal with character input key stroke:
	    Character character = CV__wait_key(1);
	    if (character == '\033') {
		// [Esc] key causes program to escape:
		break;
	    } else if (character == ' ') {
		// Write out image out to file system as a .tga file:
		File__format(stderr, "base=%s, number=%d\n",
		  capture_base_name, capture_number);
		//String file_name = String__format("%s-%02d.tga",
		//  capture_base_name, capture_number);
		char file_name[100];
		(void)sprintf(file_name, "%s-%02d.tga",
		  capture_base_name, capture_number);
		CV__tga_write(display_image, file_name);
		File__format(stderr,
		  "Wrote display_image out to file '%s'\n", file_name);
		capture_number += 1;
		//String__free(file_name);
	    }
	}

	// Clean up and leave:
	CV__destroy_window(window_name);
	// Release *display_image*:
	FC2__image_destroy(camera_image);
    }

    return 0;
}
