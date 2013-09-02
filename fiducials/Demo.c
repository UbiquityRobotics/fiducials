// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include "CV.h"
#include "File.h"
#include "Integer.h"
#include "String.h"
#include "Unsigned.h"

Integer main(Unsigned arguments_size, String arguments[]) {
    File__format(stdout, "Hello\n");
    if (arguments_size <= 1) {
	File__format(stderr, "Usage: Demo *.tga\n");
    } else {
        String image_file_name = arguments[1];
	CV_Image original_image = (CV_Image)0;
	original_image = CV__tga_read(original_image, image_file_name);
	CV__tga_write(original_image, "/tmp/foo.tga");
    }
    return 0;
}



