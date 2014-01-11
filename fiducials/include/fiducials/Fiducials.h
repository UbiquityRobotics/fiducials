// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(FIDUCIALS_H_INCLUDED)
#define FIDUCIALS_H_INCLUDED 1

typedef struct Fiducials__Struct *Fiducials;

#include <assert.h>
#include <sys/time.h>

#include "Camera_Tag.h"
#include "Character.h"
#include "CRC.h"
#include "CV.h"
#include "Double.h"
#include "File.h"
#include "FEC.h"
#include "Float.h"
#include "High_GUI2.h"
#include "Integer.h"
#include "List.h"
#include "Logical.h"
#include "Map.h"
#include "String.h"
#include "Tag.h"
#include "Unsigned.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef void (*Fiducials_Location_Announce_Routine)(void *object, Integer id,
  Double x, Double y, Double z, Double bearing);
typedef void (*Fiducials_Tag_Announce_Routine)(void *object, Integer id,
  Double x, Double y, Double z, Double twist, Double dx, Double dy, Double dz);
typedef Logical Mapping[64];
typedef struct timeval *Time_Value;

struct Fiducials__Struct {
    void *announce_object;
    CV_Scalar black;
    CV_Scalar blue;
    Logical blur;
    List /* <Camera_Tag> */ camera_tags;
    List /* <Camera_Tag> */ camera_tags_pool;
    CV_Point2D32F_Vector corners;
    CV_Scalar cyan;
    CV_Image debug_image;
    Unsigned debug_index;
    CV_Image edge_image;
    FEC fec;
    CV_Image gray_image;
    CV_Scalar green;
    Fiducials_Location_Announce_Routine location_announce_routine;
    List /*<Location>*/ locations;
    Map map;
    CV_Point origin;
    CV_Image original_image;
    Logical **mappings;
    CV_Image map_x;
    CV_Image map_y;
    CV_Scalar purple;
    CV_Scalar red;
    CV_Point2D32F_Vector references;
    CV_Point2D32F_Vector sample_points;
    CV_Size size_5x5;
    CV_Size size_m1xm1;
    CV_Memory_Storage storage;
    Fiducials_Tag_Announce_Routine tag_announce_routine;
    Logical tag_bits[64];	// FIXME: Make this Logical *tag_bits;
    CV_Image temporary_gray_image;
    CV_Term_Criteria term_criteria;
    Unsigned weights_index;
    Logical y_flip;
};

extern void Fiducials__location_announce(void *object, Integer id,
  Double x, Double y, Double z, Double bearing);
extern void Fiducials__sample_points_compute(
  CV_Point2D32F_Vector corners, CV_Point2D32F_Vector sample_points);
extern CV_Point2D32F_Vector Fiducials__references_compute(
  Fiducials fiducials, CV_Point2D32F_Vector corners);
extern Fiducials Fiducials__create(
  CV_Image original_image, String lens_calibrate_file_name,
  void *announce_object,
  Fiducials_Location_Announce_Routine location_announce_routine,
  Fiducials_Tag_Announce_Routine tag_announce_routine);
extern void Fiducials__image_set(Fiducials fiducials, CV_Image image);
extern void Fiducials__image_show(Fiducials fiducials, Logical show);
extern Unsigned Fiducials__process(Fiducials fiducials);
extern Integer Fiducials__point_sample(
  Fiducials fiducials, CV_Point2D32F point);
extern void Fiducials__sample_points_helper(
  String label, CV_Point2D32F corner, CV_Point2D32F sample_point);
extern void Fiducials__tag_heights_xml_read(
  Fiducials fiducials, String xml_file_name);
extern Integer Fiducials__points_maximum(Fiducials fiducials,
  CV_Point2D32F_Vector points, Unsigned start_index, Unsigned end_index);
extern Integer Fiducials__points_minimum(Fiducials fiducials,
  CV_Point2D32F_Vector points, Unsigned start_index, Unsigned end_index);

#ifdef __cplusplus
}
#endif
#endif // !defined(FIDUCIALS_H_INCLUDED)
