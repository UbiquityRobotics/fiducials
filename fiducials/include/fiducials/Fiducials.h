// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(FIDUCIALS_H_INCLUDED)
#define FIDUCIALS_H_INCLUDED 1

typedef struct Fiducials__Struct *Fiducials;
typedef struct Fiducials_Create__Struct *Fiducials_Create;
typedef struct Fiducials_Results__Struct *Fiducials_Results;

#include <assert.h>
#include <sys/time.h>

// #include scalar typedef's first so we can define the announce routine
// typedef's:
#include "Double.h"
#include "Integer.h"
#include "Logical.h"

// Define the announce routine typedef's:
typedef void (*Fiducials_Arc_Announce_Routine)(void *announce_object,
  Integer from_id, Double from_x, Double from_y, Double from_z,
  Integer to_id, Double to_x, Double to_y, Double to_z,
  Double goodness, Logical in_spanning_tree);

typedef void (*Fiducials_Location_Announce_Routine)(void *announce_object,
  Integer id, Double x, Double y, Double z, Double bearing);

typedef void (*Fiducials_Tag_Announce_Routine)(void *announce_object,
  Integer id, Double x, Double y, Double z, Double twist,
  Double diagonal, Double distance_per_pixel,
  Logical visible, Integer hop_count);

typedef void (*Fiducials_Fiducial_Announce_Routine)(void *announce_object,
    Integer id, Integer direction, Double world_diagonal,
    Double x1, Double y1, Double x2, Double y2,
    Double x3, Double y3, Double x4, Double y4);


// #include everything else:
#include "Camera_Tag.h"
#include "Character.h"
#include "CRC.h"
#include "CV.h"
#include "File.h"
#include "FEC.h"
#include "Float.h"
#include "High_GUI2.h"
#include "List.h"
#include "Map.h"
#include "String.h"
#include "Tag.h"
#include "Unsigned.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef Logical Mapping[64];
typedef struct timeval *Time_Value;

struct Fiducials__Struct {
    Fiducials_Arc_Announce_Routine arc_announce_routine;
    void *announce_object;
    CV_Scalar black;
    CV_Scalar blue;
    Logical blur;
    List /* <Camera_Tag> */ camera_tags;
    List /* <Camera_Tag> */ camera_tags_pool;
    CV_Point2D32F_Vector corners;
    List /* <Tag> */current_visibles;
    CV_Scalar cyan;
    CV_Image debug_image;
    Unsigned debug_index;
    CV_Image edge_image;
    FEC fec;
    CV_Image gray_image;
    CV_Scalar green;
    CV_Size image_size;
    Double last_x;
    Double last_y;
    Fiducials_Location_Announce_Routine location_announce_routine;
    Fiducials_Fiducial_Announce_Routine fiducial_announce_routine;
    List /* <Location> */ locations;
    List /* <Location> */ locations_path;
    File log_file;
    Map map;
    CV_Point origin;
    CV_Image original_image;
    Logical **mappings;
    CV_Image map_x;
    CV_Image map_y;
    String_Const path;
    List /* <Tag> */ previous_visibles;
    CV_Scalar purple;
    CV_Scalar red;
    CV_Point2D32F_Vector references;
    Fiducials_Results results;
    CV_Point2D32F_Vector sample_points;
    Unsigned sequence_number;
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

struct Fiducials_Create__Struct {
    String_Const fiducials_path;
    String_Const lens_calibrate_file_name;  
    Memory announce_object;
    Fiducials_Arc_Announce_Routine arc_announce_routine;
    Fiducials_Location_Announce_Routine location_announce_routine;
    Fiducials_Tag_Announce_Routine tag_announce_routine;
    Fiducials_Fiducial_Announce_Routine fiducial_announce_routine;
    String_Const log_file_name;
    String_Const map_base_name;
    String_Const tag_heights_file_name;
};

struct Fiducials_Results__Struct {
    Logical map_changed;
    Logical image_interesting;
};

extern void Fiducials__arc_announce(void *announce_object,
  Integer from_id, Double from_x, Double from_y, Double from_z,
  Integer to_id, Double to_x, Double to_y, Double to_z,
  Double goodness, Logical in_spanning_tree);
extern Fiducials Fiducials__create(
  CV_Image original_image, Fiducials_Create fiducials_create);
extern void Fiducials__free(Fiducials fiduicals);
extern void Fiducials__image_set(Fiducials fiducials, CV_Image image);
extern void Fiducials__image_show(Fiducials fiducials, Logical show);
extern void Fiducials__location_announce(void *object, Integer id,
  Double x, Double y, Double z, Double bearing);
extern Integer Fiducials__point_sample(
  Fiducials fiducials, CV_Point2D32F point);
extern Integer Fiducials__points_maximum(Fiducials fiducials,
  CV_Point2D32F_Vector points, Unsigned start_index, Unsigned end_index);
extern Integer Fiducials__points_minimum(Fiducials fiducials,
  CV_Point2D32F_Vector points, Unsigned start_index, Unsigned end_index);
extern Fiducials_Results Fiducials__process(Fiducials fiducials);
extern CV_Point2D32F_Vector Fiducials__references_compute(
  Fiducials fiducials, CV_Point2D32F_Vector corners);
extern void Fiducials__sample_points_compute(
  CV_Point2D32F_Vector corners, CV_Point2D32F_Vector sample_points);
extern void Fiducials__sample_points_helper(
  String_Const label, CV_Point2D32F corner, CV_Point2D32F sample_point);
extern void Fiducials__tag_announce(void *announce_object,
  Integer id, Double x, Double y, Double z, Double twist,
  Double diagonal, Double distance_per_pixel,
  Logical visible, Integer hop_count);
extern Fiducials_Create Fiducials_Create__one_and_only(void);

#ifdef __cplusplus
}
#endif
#endif // !defined(FIDUCIALS_H_INCLUDED)
