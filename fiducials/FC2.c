// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>

#include "FC2.h"
#include "File.h"
#include "Logical.h"
#include "Memory.h"

/// @brief Returns a *FC2_Version* object for the Fly Capture library.
/// @returns *FC2_Version* object.
///
/// *FC2__library_version_get*() will return the *FC2_Version* object
/// for the library.  This is a statically allocated object, so it does
/// not need to be freed.

FC2_Version FC2__library_version_get(void) {
    static fc2Version version_storage;
    FC2_Version version = &version_storage;
    fc2GetLibraryVersion(version);
    return version;
}

/// @brief Starts capturing images using *camera*.
/// @param camera to use for image capturing.
///
/// *FC2_Camera__capture_start*() will start capaturing images using *camera*.
/// Use *FC2_Camera__retrieve_image*() to actually get an image.
/// An assertion error occurs for any error.

void FC2_Camera__capture_start(FC2_Camera camera) {
    FC2_Error error = fc2StartCapture(camera);
    assert(error == FC2_ERROR_OK);
}

/// @brief Returns a new *FC2_Camera* object.
/// @returns new *FC2_Camera* object.
///
/// *FC2_Camera__create*() will create a new camera object.  Use
/// *F2C_Camera__connect*() to bind the returned object to an actual camera.
/// An assertion error occurs for any error.

FC2_Camera FC2_Camera__create(void) {
    FC2_Camera camera = (FC2_Camera)0;
    // The FC2 library calls everyting a "context" rather than a camera.
    FC2_Error error = fc2CreateContext(&camera);
    assert (error == FC2_ERROR_OK);
    return camera;
}

/// @brief Connects *camera* to the camera that matches *camera_identifier*.
/// @param camera is the *FC2_Camera* object to connect with.
/// @param camera_identifier is the camera identifier of the camera
///        to connect to.
///
/// *FC2_Camera__connect*() connects the *FC2_Camera* object *camera*
/// the camera associated with *camera_identifier*.  An assertion error
/// occurs if any failure occurs.

void FC2_Camera__connect(
  FC2_Camera camera, FC2_Camera_Identifier camera_identifier) {
    FC2_Error error = fc2Connect(camera, camera_identifier);
    assert(error == FC2_ERROR_OK);
}


/// @brief Returns a *FC2_Camera_Indentifier* for the *index*'th camera.
/// @param camera is just used a handle.
/// @param index is the camera to fetch.
/// @returns a *FC2_Camera_Identifier* for the selected camera.
///
/// *FC2_Camera__identifier_fetch*() will return a *FC2_Camera_Indentifier*
/// for the *index*'th camera.  An assertion error occurs if there is no
/// *index*'th camera.  Use *Memory__free*() to release the storage for
/// the returned *FC2_Camera_Identifier* object.

FC2_Camera_Identifier FC2_Camera__identifier_fetch(
  FC2_Camera camera, Unsigned index) {
    FC2_Camera_Identifier camera_identifier =
      Memory__new(FC2_Camera_Identifier);
    FC2_Error error = fc2GetCameraFromIndex(camera, index, camera_identifier);
    assert (error == FC2_ERROR_OK);
    return camera_identifier;
}

/// @brief Releases the storage associated with *camera*.
/// @param camera storage to release.
///
/// * FC2_Camera__free*() will release the storage associated with *camera*.

void FC2_Camera__free(FC2_Camera camera) {
    // The FC2 library calls a camera a context:
    FC2_Error error = fc2DestroyContext(camera);
    assert (error == FC2_ERROR_OK);
}

/// @brief Grabs the next image from *camera* and stores it into *image*.
/// @param camera to fetch image from.
/// @param image to store result into.
///
/// *FC2__image_retrieve*() will retrieve an image from *camara* and store
/// the result into *image*.  An assertion failure will occur for any error.

void FC2_Camera__image_retrieve(FC2_Camera camera, FC2_Image image) {
    FC2_Error error = fc2RetrieveBuffer(camera, image);
    assert(error == FC2_ERROR_OK);
}

/// @brief Returns an *FC2_Camera_Information* object for *camera*.
/// @param camera to get information for.
/// @returns *FC2_Camera_Information* object.
///
/// *FC2_Camera__information_get*() will return a *FC2_Camera_Information*
/// object for *camera*.  An assertion failure will occur for any error.
/// The returned *FC2_Camera_Information* object can be released using
/// *Memory__free*().

FC2_Camera_Information FC2_Camera__information_get(FC2_Camera camera) {
    FC2_Camera_Information camera_information =
      Memory__new(FC2_Camera_Information);
    FC2_Error error = fc2GetCameraInfo(camera, camera_information);
    assert (error == FC2_ERROR_OK);
    return camera_information;
}

/// @brief Returns the number of available cameras.
/// @param camera to use as a handle.
/// @returns the total number of accessible cameras.
///
/// *FC2_Camera__number_of_cameras_get*() will return the total number of
/// accessible cameras.  An assertion failure occurs for any error.

Unsigned FC2_Camera__number_of_cameras_get(FC2_Camera camera) {
    Unsigned number_of_cameras = 0;
    if (fc2GetNumOfCameras(camera, &number_of_cameras) != FC2_ERROR_OK) {
	number_of_cameras = -1;
    }
    return number_of_cameras;
}

/// @brief Convert *from_image* into *pixel_format* and store the result
///        into *to_image*.
/// @param from_image is the image to convert from.
/// @param to_image is the image to convert to.
/// @param pixel_format is the desired format.
///
/// *FC2_Image__convert*() will convert *from_image* into *pixel_format* and
/// store the result into *to_image*.  An assertion failure occurs on any
/// error.  The permitted values for *pixel_format* are:
///
/// * *FC2_PIXEL_FORMAT_MONO8*
/// * *FC2_PIXEL_FORMAT_411YUV8*
/// * *FC2_PIXEL_FORMAT_422YUV8*
/// * *FC2_PIXEL_FORMAT_444YUV8*
/// * *FC2_PIXEL_FORMAT_RGB8*
/// * *FC2_PIXEL_FORMAT_MONO16*
/// * *FC2_PIXEL_FORMAT_RGB16*
/// * *FC2_PIXEL_FORMAT_S_MONO16*
/// * *FC2_PIXEL_FORMAT_S_RGB16*
/// * *FC2_PIXEL_FORMAT_RAW8*
/// * *FC2_PIXEL_FORMAT_RAW16*
/// * *FC2_PIXEL_FORMAT_MONO12*
/// * *FC2_PIXEL_FORMAT_RAW12*
/// * *FC2_PIXEL_FORMAT_BGR*
/// * *FC2_PIXEL_FORMAT_BGRU*
/// * *FC2_PIXEL_FORMAT_RGB*
/// * *FC2_PIXEL_FORMAT_RGBU*
/// * *FC2_PIXEL_FORMAT_BGR16*
/// * *FC2_PIXEL_FORMAT_BGRU16*
/// * *FC2_PIXEL_FORMAT_422YUV8_JPEG*

void FC2_Image__convert(
  FC2_Image from_image, FC2_Image to_image, FC2_Pixel_Format pixel_format) {
    FC2_Error error = fc2ConvertImageTo(pixel_format, from_image, to_image);
    assert (error == FC2_ERROR_OK);
}

/// @brief Create and return a new *FC2_Image* object.
/// @returns new *FC2_Image* object.
///
/// *FC2_Image__create*() will return a new *FC2_Image* object suitable
/// for containing a camera image.  An assertion error occurs on any error.

FC2_Image FC2_Image__create(void) {
    FC2_Image image = Memory__new(FC2_Image);
    FC2_Error error = fc2CreateImage(image);
    assert (error == FC2_ERROR_OK);
    return image;
}

/// @brief Returns pointer to *image* data.
/// @param image to grab data from.
/// @returns pointer to *image* data.
///
/// *FC2__image_data_get*() will return a pointer to the image data
/// associated with *image*.  An assertion error occurs on any error.

Memory FC2_Image__data_get(FC2_Image image) {
    unsigned char *image_data = (unsigned char *)0;
    FC2_Error error = fc2GetImageData(image, &image_data);
    assert(error == FC2_ERROR_OK);
    return (Memory)image_data;
}

/// @brief Will release the storage associated with *image*.
/// @param image to release.
///
/// *FC2_Image__free*() will release the storage associated with *image*.

void FC2_Image__free(FC2_Image image) {
    FC2_Error error = fc2DestroyImage(image);
    assert (error == FC2_ERROR_OK);
    Memory__free((Memory)image);
}


