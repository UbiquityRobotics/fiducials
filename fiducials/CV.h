// Copyright (c) 2010 by Wayne C. Gramlich.  All rights reserved.

#if !defined(CV_C_H_INCLUDED)
#define CV_C_H_INCLUDED 1

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "Logical.h"
#include "Double.h"
#include "Integer.h"
#include "Memory.h"
#include "String.h"

typedef CvContour *CV_Contour;
typedef IplImage *CV_Image;
typedef CvMat *CV_Matrix;
typedef CvMemStorage *CV_Memory_Storage;
typedef CvPoint *CV_Point;
typedef CvPoint2D32f *CV_Point2D32F;
typedef CvPoint2D32f *CV_Point2D32F_Vector;
typedef CvScalar *CV_Scalar;
typedef CvSeq *CV_Sequence;
typedef CvSize *CV_Size;
typedef CvSlice *CV_Slice;
typedef CvTermCriteria *CV_Term_Criteria;

extern Integer CV__chain_approx_simple;
extern Integer CV__adaptive_thresh_gaussian_c;
extern Integer CV__depth_8u;
extern Integer CV__gaussian;
extern Integer CV__gray_to_rgb;
extern Integer CV__poly_approx_dp;
extern Integer CV__retr_list;
extern Integer CV__rgb_to_gray;
extern Integer CV__thresh_binary;
extern Integer CV__window_auto_size;

extern Integer CV__round(Double value);
extern Integer CV__undistortion_setup(String calibrate_file_name,
  Integer width, Integer height, CV_Image *mapx, CV_Image *mapy);

extern void CV_Image__adaptive_threshold(CV_Image source_image,
  CV_Image destination_image, Double maximum_value, Integer adaptive_method,
  Integer threshold_type, Integer block_size, Double parameter1);
extern void CV_Image__blob_draw(
  CV_Image image, Integer x, Integer y, CV_Scalar color);
extern Integer CV_Image__channels_get(CV_Image image);
extern void CV_Image__convert_color(
  CV_Image source_image, CV_Image destination_image, Integer conversion_code);
extern void CV_Image__copy(
  CV_Image source_image, CV_Image destination_image, CV_Image mask);
extern CV_Image CV_Image__create(
  CV_Size size, Unsigned depth, Unsigned channels);
extern CV_Image CV_Image__header_create(
  CV_Size size, Unsigned depth, Unsigned channels);
extern void CV_Image__cross_draw(
  CV_Image image, Integer x, Integer y, CV_Scalar color);
extern void CV_Image__draw_contours(CV_Image image, CV_Sequence contour,
  CV_Scalar external_color, CV_Scalar hole_color, Integer maximal_level,
  Integer thickness, Integer line_type, CV_Point offset);
extern CV_Sequence CV_Image__find_contours(CV_Image image,
  CV_Memory_Storage storage, Integer header_size, Integer mode,
  Integer method, CV_Point point);
extern void CV_Image__find_corner_sub_pix(CV_Image image,
  CV_Point2D32F_Vector corners, Integer count, CV_Size window,
  CV_Size zero_zone, CV_Term_Criteria criteria);
extern void CV_Image__flip(
  CV_Image from_image, CV_Image to_image, Integer flip_code);
extern Integer CV_Image__gray_fetch(CV_Image image, Integer x, Integer y);
extern Integer CV_Image__height_get(CV_Image image);
extern Integer CV_Image__points_maximum(CV_Image image,
  CV_Point2D32F_Vector points, Unsigned start_index, Unsigned end_index);
extern Integer CV_Image__points_minimum(CV_Image image,
  CV_Point2D32F_Vector points, Unsigned start_index, Unsigned end_index);
extern Integer CV_Image__point_sample(CV_Image image, CV_Point2D32F point);
extern void CV_Image__remap(CV_Image source_image, CV_Image destination_image,
  CV_Image map_x, CV_Image map_y, Integer flags, CV_Scalar fill_value);
extern Integer CV_Image__save(
  CV_Image image, String file_name, Integer *parameters);
extern void CV_Image__smooth(CV_Image source_image, CV_Image destination_image,
  Integer smooth_type, Integer parameter1, Integer parameter2,
  Double parameter3, Double parameter4);
extern CV_Image CV_Image__pnm_read(String file_base_name);
extern void CV_Image__pnm_write(CV_Image image, String file_base_name);
extern CV_Image CV_Image__tga_read(CV_Image image, String file_name);
extern void CV_Image__tga_write(CV_Image image, String file_name);
extern Integer CV_Image__width_get(CV_Image image);

extern Integer CV__term_criteria_iterations;
extern Integer CV__term_criteria_eps;
extern CV_Term_Criteria CV_Term_Criteria__create(
  Integer type, Integer maximum_iterations, Double epsilon);

extern void CV_Memory_Storage__clear(CV_Memory_Storage storage);
extern CV_Memory_Storage CV_Memory_Storage__create(Integer block_size);

extern CV_Point CV_Point__create(Integer x, Integer y);
extern Integer CV_Point__y_get(CV_Point point);
extern Integer CV_Point__x_get(CV_Point point);

extern void CV_Point2D32F_Vector__corners_normalize(
  CV_Point2D32F_Vector corners);
extern CV_Point2D32F_Vector CV_Point2D32F_Vector__create(Unsigned size);
extern CV_Point2D32F CV_Point2D32F_Vector__fetch1(
  CV_Point2D32F_Vector vector,  Unsigned index);
extern Logical CV_Point2D32F_Vector__is_clockwise(CV_Point2D32F_Vector corners);

extern void CV_Point2D32F__point_set(CV_Point2D32F point2d32f, CV_Point point);
extern Double CV_Point2D32F__x_get(CV_Point2D32F point);
extern void CV_Point2D32F__x_set(CV_Point2D32F point, Double x);
extern Double CV_Point2D32F__y_get(CV_Point2D32F point);
extern void CV_Point2D32F__y_set(CV_Point2D32F point, Double y);

extern CV_Scalar CV_Scalar__create(
  Double value0, Double value1, Double value2, Double value3);
extern CV_Scalar CV_Scalar__rgb(Double red, Double green, Double blue);

extern CV_Sequence CV_Sequence__approximate_polygon(CV_Sequence contour,
  Integer header_size, CV_Memory_Storage storage, Integer method,
  Integer parameter1, Double parameter2);
extern Double CV_Sequence__arc_length(
 CV_Sequence contour, CV_Slice slice, Integer is_closed);
extern Logical CV_Sequence__check_contour_convexity(CV_Sequence contour);
extern Double CV_Sequence__contour_area(
  CV_Sequence contour, CV_Slice slice, Integer oriented);
extern CV_Sequence CV_Sequence__next_get(CV_Sequence sequence);
extern CV_Point CV_Sequence__point_fetch1(CV_Sequence sequence, Unsigned index);
extern Integer CV_Sequence__total_get(CV_Sequence sequence);

extern CV_Size CV_Size__create(Integer width, Integer height);

extern void CV__release_image(CV_Image image);

#endif // !defined(CV_C_H_INCLUDED)
