// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>

#include "Character.h"
#include "CV.h"
#include "File.h"
#include "High_GUI2.h"
#include "Integer.h"
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
    CV_Capture capture = (CV_Capture)0;
    String capture_base_name = "video_capture";

    if (arguments_size <= 1) {
	// No arguments; let the user know the usage:
	File__format(stderr,
	  "Usage: Video_Capture camera_number [capture_base_name]\n");
	return 1;
    } else {
        // Grab the arguments:
	String argument1 = arguments[1];
	if (arguments_size > 2) {
	    capture_base_name = arguments[2];
	}

	// Figure whether to open a video file or a camera;
	if (Character__is_decimal_digit(argument1[0])) {
	    // Open the camera:
	    Unsigned camera_number = String__to_unsigned(argument1);
	    Integer camera_flags = CV__capture_any + (Integer)camera_number;
	    capture = CV_Capture__create_camera(camera_flags);
	    if (capture == (CV_Capture)0) {
		File__format(stderr,
		  "Could not open camara %d\n", camera_number);
		return 1;
	    }

	    // Set the frame size:
	    CV_Capture__set_property(capture,
	      CV_Capture__property_frame_width, (Double)640);
	    CV_Capture__set_property(capture,
	      CV_Capture__property_frame_height, (Double)480);
	} else {
	    // Open a video file format:
	    capture = CV_Capture__create_file(argument1);
	    if (capture == (CV_Capture)0) {
		File__format(stderr,
		  "Could not open video file '%s'\n", argument1);
		return 1;
	    }
	}
    }
    // We should not be able to here without a open *capture*:
    assert(capture != (CV_Capture)0);

    // Create the window to display the video into:
    String window_name = "Video_Capture";
    CV__named_window(window_name, CV__window_auto_size);

    // Do a video loop:
    Unsigned capture_number = 0;
    while (1) {
        // Grab a frame from the video source:
	CV_Image frame = CV_Capture__query_frame(capture);
	if (frame == (CV_Image)0) {
	    // When *frame* is null, the video source is at end-of-file
	    // or disconnected:
	    break;
	}
	
	// Show the image:
	CV_Image__show(frame, window_name);

	// Deal with key character:
	Character character = CV__wait_key(33);
	if (character == '\033') {
	    // [Esc] key causes program to escape:
	    break;
	} else if (character == ' ') {
	    // Write out image out to file system as a .tga file:
	    String file_name =
	      String__format("%s-%02d.pnm", capture_base_name, capture_number);
	    CV_Image__pnm_write(frame, file_name);
	    File__format(stderr, "Wrote frame out to file '%s'\n", file_name);
	    capture_number += 1;
	    String__free(file_name);
	}
    }

    // Clean up and leave:
    CV_Capture__release(capture);
    CV__destroy_window(window_name);

    return 0;
}
