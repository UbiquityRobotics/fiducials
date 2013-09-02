// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include "Character.h"
#include "CV.h"
#include "File.h"
#include "High_GUI2.h"
#include "Integer.h"
#include "Logical.h"
#include "String.h"
#include "Unsigned.h"

extern void CV__image_show(CV_Image original_image, Logical show);

Integer main(Unsigned arguments_size, String arguments[]) {
    File__format(stdout, "Hello\n");
    if (arguments_size <= 1) {
	File__format(stderr, "Usage: Demo *.tga\n");
    } else {
        String image_file_name = arguments[1];
	CV_Image original_image = (CV_Image)0;
	original_image = CV__tga_read(original_image, image_file_name);
	CV__image_show(original_image, (Logical)1);
    }
    return 0;
}


// This routinew will show {original_image} on the screen along
// with a primitive debugging interface to showing how the debugging
// is going.

void CV__image_show(/* Extractor extractor, */
  CV_Image original_image, Logical show) {

    CV_Image debug_image = (CV_Image)0;
    //debug_image :@= extractor.debug_image
    //extractor.width := original_image.width
    //extractor.height := original_image.height

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
	//call extract@(extractor, original_image, debug_index, 0xffffff00)

	// Display either *original_image* or *debug_image*:
	if (show) {
	    if (debug_index == 0) {
		CV__show_image(window_name, original_image);
	    } else {
		CV__show_image(window_name, debug_image);
	    }
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
	    //extractor.blur := !extractor.blur
	    //call d@(form@("blur = %l%\n\") / f@(extractor.blur))
	    break;
	  default:
	    // Deal with unknown {control_character}:
	    if ((Unsigned)control_character <= 127) {
		File__format(stderr,
		  "Unknown control character %d\n", control_character);
	    }
	    break;
	}

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


