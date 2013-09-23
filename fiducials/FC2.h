// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(FC2_H_INCLUDED)
#define FC2_H_INCLUDED 1

typedef struct _fc2Version *FC2_Version;
typedef void *FC2_Context;
typedef struct _fc2PGRGuid *FC2_Camera_Identifier;
typedef struct _fc2CameraInfo *FC2_Camera_Information;
typedef struct _fc2EmbeddedImageInfo *FC2_Embedded_Image_Information;
typedef struct _fc2Image *FC2_Image;

#include "C/FlyCapture2_C.h"

#include "Unsigned.h"


typedef fc2Error FC2_Error;

// Routine prototypes:

unsigned char *FC2__image_data_get(FC2_Image image);
extern void FC2__image_retrieve(FC2_Context context, FC2_Image image);
extern void FC2__image_destroy(FC2_Image image);
extern FC2_Image FC2__image_create(void);
extern FC2_Error FC2__capture_start(FC2_Context context);
extern FC2_Error FC2__time_stamping_set(FC2_Context context, Logical enable);
extern FC2_Camera_Information FC2__camera_information_get(FC2_Context context);
extern FC2_Error FC2__camera_connect(
  FC2_Context context, FC2_Camera_Identifier camera_indentifier);
extern FC2_Context FC2__context_create(void);
extern FC2_Camera_Identifier FC2__camera_fetch(FC2_Context context, Unsigned index);
extern FC2_Version FC2__library_version_get(void);
extern Integer FC2__number_of_cameras_get(FC2_Context context);

#endif // !defined(FC2_H_INCLUDED)
