// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>

#include "FC2.h"
#include "File.h"
#include "Logical.h"
#include "Memory.h"

unsigned char *FC2__image_data_get(FC2_Image image) {
    unsigned char *image_data = (unsigned char *)0;
    FC2_Error error = fc2GetImageData(image, &image_data);
    assert(error == FC2_ERROR_OK);
    return image_data;
}

void FC2__image_destroy(FC2_Image image) {
    FC2_Error error = fc2DestroyImage(image);
    assert (error == FC2_ERROR_OK);
    Memory__free((Memory)image);
}

void FC2__image_retrieve(FC2_Context context, FC2_Image image) {
    FC2_Error error = fc2RetrieveBuffer(context, image);
    assert(error == FC2_ERROR_OK);
}

FC2_Image FC2__image_create(void) {
    FC2_Image image = Memory__new(FC2_Image);
    FC2_Error error = fc2CreateImage(image);
    assert (error == FC2_ERROR_OK);
    return image;
}

FC2_Error FC2__capture_start(FC2_Context context) {
    FC2_Error error = fc2StartCapture(context);
    return error;
}

FC2_Error FC2__time_stamping_set(FC2_Context context, Logical enable) {
    FC2_Embedded_Image_Information embedded_image_information =
      Memory__new(FC2_Embedded_Image_Information);

    FC2_Error error = fc2GetEmbeddedImageInfo(context, embedded_image_information);
    if (error == FC2_ERROR_OK) {
	if (embedded_image_information->timestamp.available != 0) {
	    embedded_image_information->timestamp.onOff = enable;
	}
	error = fc2SetEmbeddedImageInfo(context, embedded_image_information);
    }

    Memory__free((Memory)embedded_image_information);
    return error;
}

FC2_Camera_Information FC2__camera_information_get(FC2_Context context) {
    FC2_Camera_Information camera_information = Memory__new(FC2_Camera_Information);
    FC2_Error error = fc2GetCameraInfo(context, camera_information);
    if (error != FC2_ERROR_OK) {
	Memory__free((Memory)camera_information);
	camera_information = (FC2_Camera_Information)0;
    }
    return camera_information;
}

FC2_Error FC2__camera_connect(
  FC2_Context context, FC2_Camera_Identifier camera_identifier) {
    return fc2Connect(context, camera_identifier);
}

FC2_Context FC2__context_create(void) {
    FC2_Context context = (FC2_Context)0;
    FC2_Error error = fc2CreateContext(&context);
    if (error != FC2_ERROR_OK) {
        context = (FC2_Context)0;
    }
    return context;
}

FC2_Camera_Identifier FC2__camera_fetch(FC2_Context context, Unsigned index) {
    FC2_Camera_Identifier camera_identifier = Memory__new(FC2_Camera_Identifier);
    FC2_Error error = fc2GetCameraFromIndex(context, index, camera_identifier);
    if (error != FC2_ERROR_OK) {
	Memory__free((Memory)camera_identifier);
	camera_identifier = (FC2_Camera_Identifier)0;
    }
    return camera_identifier;
}

Integer FC2__number_of_cameras_get(FC2_Context context) {
    Integer number_of_cameras = -1;
    if (fc2GetNumOfCameras(context, &number_of_cameras) != FC2_ERROR_OK) {
	number_of_cameras = -1;
    }
    return number_of_cameras;
}


FC2_Version FC2__library_version_get(void) {
    static fc2Version version_storage;
    FC2_Version version = &version_storage;
    fc2GetLibraryVersion(version);
    return version;
}

