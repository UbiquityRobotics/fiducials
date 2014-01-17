// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(FC2_H_INCLUDED)
#define FC2_H_INCLUDED 1

// #include's:
#include "C/FlyCapture2_C.h"
#include "Memory.h"
#include "Unsigned.h"

#ifdef __cplusplus
extern "C" {
#endif
/// @brief FC2 package overview documenation is here (please click).
///
/// The FC2 package is just a bunch of wrapper routines for the FC2 library.
/// All of the error detection is pushed into the wrappers to make the
/// code that uses the wrappers easier to read.  The two primary data
/// structures are *FC2_Camera* and *FC2_Image*.  The remaining data
/// structures are pretty trivial and obvious.
typedef int FC2_Overview_Documeation;

/// @brief *FC2_Camera* represents one FC2 library camera.
typedef void *FC2_Camera;

/// @brief *FC2_Camera_Identifier* is an FC2 library global unique
/// camera identifier.
typedef struct _fc2PGRGuid *FC2_Camera_Identifier;

/// @brief *FC2_Camera_Information* is information about a specfic FC2 library
/// camera.
typedef struct _fc2CameraInfo *FC2_Camera_Information;

/// @brief *FC2_Image* is one image from an FC2 library camera.
typedef struct _fc2Image *FC2_Image;

/// @brief *FC2_Version* contains FC2 library version information.
typedef struct _fc2Version *FC2_Version;

/// @brief *FC2_Error* is an FC2 internal error code.
typedef fc2Error FC2_Error;

/// @brief *FC2_Pixel_Format* is an enumeration of different possible FC2
/// library image formats.
typedef enum _fc2PixelFormat FC2_Pixel_Format;

// Routine prototypes:

extern FC2_Version FC2__library_version_get(void);

// *FC2_Camara* routines:
extern void FC2_Camera__capture_start(FC2_Camera camera);
extern FC2_Camera FC2_Camera__create(void);
extern void FC2_Camera__connect(
  FC2_Camera camera, FC2_Camera_Identifier camera_indentifier);
extern FC2_Camera_Identifier FC2_Camera__identifier_fetch(
  FC2_Camera camera, Unsigned index);
extern void FC2_Camera__image_retrieve(FC2_Camera camera, FC2_Image image);
extern FC2_Camera_Information FC2_Camera__information_get(FC2_Camera camera);
extern Unsigned FC2_Camera__number_of_cameras_get(FC2_Camera camera);

// *FC2_Image* routines:
extern void FC2_Image__convert(
  FC2_Image from_image, FC2_Image to_image, FC2_Pixel_Format pixel_format);
extern FC2_Image FC2_Image__create(void);
extern Memory FC2_Image__data_get(FC2_Image image);
extern void FC2_Image__free(FC2_Image image);

#ifdef __cplusplus
}
#endif
#endif // !defined(FC2_H_INCLUDED)
