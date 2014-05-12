// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(FIDUCIALS_H_INCLUDED)
#define FIDUCIALS_H_INCLUDED 1

typedef struct Fiducials__Struct *Fiducials;
typedef struct Fiducials_Create__Struct *Fiducials_Create;
typedef struct Fiducials_Results__Struct *Fiducials_Results;

#include <assert.h>
#include <sys/time.h>

typedef void (*Fiducials_Arc_Announce_Routine)(void *announce_object,
  int from_id, double from_x, double from_y, double from_z,
  int to_id, double to_x, double to_y, double to_z,
  double goodness, bool in_spanning_tree);

typedef void (*Fiducials_Location_Announce_Routine)(void *announce_object,
  int id, double x, double y, double z, double bearing);

typedef void (*Fiducials_Tag_Announce_Routine)(void *announce_object,
  int id, double x, double y, double z, double twist,
  double diagonal, double distance_per_pixel,
  bool visible, int hop_count);

typedef void (*Fiducials_Fiducial_Announce_Routine)(void *announce_object,
    int id, int direction, double world_diagonal,
    double x1, double y1, double x2, double y2,
    double x3, double y3, double x4, double y4);


// #include everything else:
#include "CRC.hpp"
#include "CV.hpp"
#include "File.hpp"
#include "FEC.hpp"
#include "High_GUI2.hpp"
#include "Map.hpp"
#include "String.hpp"
#include "Tag.hpp"

class CameraTag;

typedef bool Mapping[64];
typedef struct timeval *Time_Value;

struct Fiducials__Struct {
    Fiducials_Arc_Announce_Routine arc_announce_routine;
    void *announce_object;
    CV_Scalar black;
    CV_Scalar blue;
    bool blur;
    std::vector<CameraTag * > camera_tags;
    CV_Point2D32F_Vector corners;
    std::vector<Tag> current_visibles;
    CV_Scalar cyan;
    CV_Image debug_image;
    unsigned int debug_index;
    CV_Image edge_image;
    FEC fec;
    CV_Image gray_image;
    CV_Scalar green;
    CV_Size image_size;
    double last_x;
    double last_y;
    Fiducials_Location_Announce_Routine location_announce_routine;
    Fiducials_Fiducial_Announce_Routine fiducial_announce_routine;
    std::vector<Location*> locations;
    std::vector<Location*> locations_path;
    File log_file;
    Map map;
    CV_Point origin;
    CV_Image original_image;
    bool **mappings;
    CV_Image map_x;
    CV_Image map_y;
    String_Const path;
    std::vector<Tag> previous_visibles;
    CV_Scalar purple;
    CV_Scalar red;
    CV_Point2D32F_Vector references;
    Fiducials_Results results;
    CV_Point2D32F_Vector sample_points;
    unsigned int sequence_number;
    CV_Size size_5x5;
    CV_Size size_m1xm1;
    CV_Memory_Storage storage;
    Fiducials_Tag_Announce_Routine tag_announce_routine;
    bool tag_bits[64];        // FIXME: Make this bool *tag_bits;
    CV_Image temporary_gray_image;
    CV_Term_Criteria term_criteria;
    unsigned int weights_index;
    bool y_flip;
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
    bool map_changed;
    bool image_interesting;
};

extern void Fiducials__arc_announce(void *announce_object,
  int from_id, double from_x, double from_y, double from_z,
  int to_id, double to_x, double to_y, double to_z,
  double goodness, bool in_spanning_tree);
extern Fiducials Fiducials__create(
  CV_Image original_image, Fiducials_Create fiducials_create);
extern void Fiducials__free(Fiducials fiduicals);
extern void Fiducials__image_set(Fiducials fiducials, CV_Image image);
extern void Fiducials__image_show(Fiducials fiducials, bool show);
extern void Fiducials__location_announce(void *object, int id,
  double x, double y, double z, double bearing);
extern int Fiducials__point_sample(
  Fiducials fiducials, CV_Point2D32F point);
extern int Fiducials__points_maximum(Fiducials fiducials,
  CV_Point2D32F_Vector points, unsigned int start_index, unsigned int end_index);
extern int Fiducials__points_minimum(Fiducials fiducials,
  CV_Point2D32F_Vector points, unsigned int start_index, unsigned int end_index);
extern Fiducials_Results Fiducials__process(Fiducials fiducials);
extern CV_Point2D32F_Vector Fiducials__references_compute(
  Fiducials fiducials, CV_Point2D32F_Vector corners);
extern void Fiducials__sample_points_compute(
  CV_Point2D32F_Vector corners, CV_Point2D32F_Vector sample_points);
extern void Fiducials__sample_points_helper(
  String_Const label, CV_Point2D32F corner, CV_Point2D32F sample_point);
extern void Fiducials__tag_announce(void *announce_object,
  int id, double x, double y, double z, double twist,
  double diagonal, double distance_per_pixel,
  bool visible, int hop_count);
extern Fiducials_Create Fiducials_Create__one_and_only(void);

#endif // !defined(FIDUCIALS_H_INCLUDED)
