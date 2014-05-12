// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(CAMERA_TAG_H_INCLUDED)
#define CAMERA_TAG_H_INCLUDED 1

#include "CV.hpp"
#include "Tag.hpp"

/// @brief *Camera_Tag* represents information about a fiducial tag in
/// a camera frame.
class CameraTag {
  public:
    /// @brief The average diagonal of the tag measured in camera pixels.
    double diagonal;

    /// @brief The direction (0-3) that matched the fiducial tag.
    unsigned int direction;

    /// @brief The tag associated with *id*.
    Tag tag;

    /// @brief The amount the fiducial tag is twisted from the camera x axis
    /// measured in radians.
    double twist;

    /// @brief The X coordinate of the fiducial tag center in camera pixels.
    double x;

    /// @brief The Y coordinate of the fiducial tag center in camera pixels.
    double y;

    static std::vector<CameraTag*> free_pool;
  public:
    static bool less(CameraTag * tag1, CameraTag * tag2);

    CameraTag();

    void initialize(Tag tag, unsigned int direction,
        CV_Point2D32F_Vector corners, CV_Image debug_image);

    void* operator new(size_t sz);
    void operator delete(void*);
};

#endif // !defined(CAMERA_TAG_H_INCLUDED)
