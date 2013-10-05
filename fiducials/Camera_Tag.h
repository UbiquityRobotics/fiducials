// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(CAMERA_TAG_H_INCLUDED)
#define CAMERA_TAG_H_INCLUDED 1

/// @brief *Camera_Tag* is a pointer to a *Camera_Tag__Struct* object:
typedef struct Camera_Tag__Struct *Camera_Tag;

#include "CV.h"
#include "Double.h"
#include "Tag.h"
#include "Unsigned.h"

/// @brief *Camera_Tag* represents information about a fiducial tag in
/// a camera frame.
struct Camera_Tag__Struct {
    /// @brief The average diagonal of the tag measured in camera pixels.
    Double diagonal;

    /// @brief The direction (0-3) that matched the fiducial tag.
    Unsigned direction;

    /// @brief The tag associated with *id*.
    Tag tag;

    /// @brief The amount the fiducial tag is twisted from the camera x axis
    /// measured in radians.
    Double twist;

    /// @brief The X coordinate of the fiducial tag center in camera pixels.
    Double x;

    /// @brief The Y coordinate of the fiducial tag center in camera pixels.
    Double y;
};

// *Camera_Tag* routines:

extern Integer Camera_Tag__compare(
  Camera_Tag camera_tag1, Camera_Tag camera_tag2);
extern Camera_Tag Camera_Tag__new(void);
extern void Camera_Tag__initialize(Camera_Tag camera_tag, Tag tag,
  Unsigned direction, CV_Point2D32F_Vector corners, CV_Image debug_image);

#endif // !defined(CAMERA_TAG_H_INCLUDED)
