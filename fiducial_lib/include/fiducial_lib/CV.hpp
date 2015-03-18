// Copyright (c) 2010 by Wayne C. Gramlich.  All rights reserved.

#if !defined(CV_C_H_INCLUDED)
#define CV_C_H_INCLUDED 1

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "Memory.hpp"
#include "String.hpp"

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

extern int CV__chain_approx_simple;
extern int CV__adaptive_thresh_gaussian_c;
extern int CV__depth_8u;
extern int CV__gaussian;
extern int CV__gray_to_rgb;
extern int CV__poly_approx_dp;
extern int CV__retr_list;
extern int CV__rgb_to_gray;
extern int CV__thresh_binary;
extern int CV__window_auto_size;

extern int CV__round(double value);
extern int CV__undistortion_setup(String_Const calibrate_file_name,
  int width, int height, CV_Image *mapx, CV_Image *mapy);

extern void CV_Image__adaptive_threshold(CV_Image source_image,
  CV_Image destination_image, double maximum_value, int adaptive_method,
  int threshold_type, int block_size, double parameter1);
extern void CV_Image__blob_draw(
  CV_Image image, int x, int y, CV_Scalar color);
extern int CV_Image__channels_get(CV_Image image);
extern void CV_Image__convert_color(
  CV_Image source_image, CV_Image destination_image, int conversion_code);
extern void CV_Image__copy(
  CV_Image source_image, CV_Image destination_image, CV_Image mask);
extern CV_Image CV_Image__create(
  CV_Size size, unsigned int depth, unsigned int channels);
extern CV_Image CV_Image__header_create(
  CV_Size size, unsigned int depth, unsigned int channels);
extern void CV_Image__cross_draw(
  CV_Image image, int x, int y, CV_Scalar color);
extern void CV_Image__draw_contours(CV_Image image, CV_Sequence contour,
  CV_Scalar external_color, CV_Scalar hole_color, int maximal_level,
  int thickness, int line_type, CV_Point offset);
extern CV_Sequence CV_Image__find_contours(CV_Image image,
  CV_Memory_Storage storage, int header_size, int mode,
  int method, CV_Point point);
extern void CV_Image__find_corner_sub_pix(CV_Image image,
  CV_Point2D32F_Vector corners, int count, CV_Size window,
  CV_Size zero_zone, CV_Term_Criteria criteria);
extern void CV_Image__flip(
  CV_Image from_image, CV_Image to_image, int flip_code);
extern int CV_Image__gray_fetch(CV_Image image, int x, int y);
extern int CV_Image__height_get(CV_Image image);
extern int CV_Image__points_maximum(CV_Image image,
  CV_Point2D32F_Vector points, unsigned int start_index, unsigned int end_index);
extern int CV_Image__points_minimum(CV_Image image,
  CV_Point2D32F_Vector points, unsigned int start_index, unsigned int end_index);
extern int CV_Image__point_sample(CV_Image image, CV_Point2D32F point);
extern void CV_Image__remap(CV_Image source_image, CV_Image destination_image,
  CV_Image map_x, CV_Image map_y, int flags, CV_Scalar fill_value);
extern int CV_Image__save(
  CV_Image image, String_Const file_name, int *parameters);
extern void CV_Image__smooth(CV_Image source_image, CV_Image destination_image,
  int smooth_type, int parameter1, int parameter2,
  double parameter3, double parameter4);
extern CV_Image CV_Image__pnm_read(String_Const file_base_name);
extern void CV_Image__pnm_write(CV_Image image, String_Const file_base_name);
extern CV_Image CV_Image__tga_read(CV_Image image, String_Const file_name);
extern void CV_Image__tga_write(CV_Image image, String_Const file_name);
extern int CV_Image__width_get(CV_Image image);

extern int CV__term_criteria_iterations;
extern int CV__term_criteria_eps;
extern CV_Term_Criteria CV_Term_Criteria__create(
  int type, int maximum_iterations, double epsilon);

extern void CV_Memory_Storage__clear(CV_Memory_Storage storage);
extern CV_Memory_Storage CV_Memory_Storage__create(int block_size);

extern CV_Point CV_Point__create(int x, int y);
extern int CV_Point__y_get(CV_Point point);
extern int CV_Point__x_get(CV_Point point);

extern void CV_Point2D32F_Vector__corners_normalize(
  CV_Point2D32F_Vector corners);
extern CV_Point2D32F_Vector CV_Point2D32F_Vector__create(unsigned int size);
extern CV_Point2D32F CV_Point2D32F_Vector__fetch1(
  CV_Point2D32F_Vector vector,  unsigned int index);
extern bool CV_Point2D32F_Vector__is_clockwise(CV_Point2D32F_Vector corners);

extern void CV_Point2D32F__point_set(CV_Point2D32F point2d32f, CV_Point point);
extern double CV_Point2D32F__x_get(CV_Point2D32F point);
extern void CV_Point2D32F__x_set(CV_Point2D32F point, double x);
extern double CV_Point2D32F__y_get(CV_Point2D32F point);
extern void CV_Point2D32F__y_set(CV_Point2D32F point, double y);

extern CV_Scalar CV_Scalar__create(
  double value0, double value1, double value2, double value3);
extern void CV_Scalar__free(CV_Scalar cv_scalar);
extern CV_Scalar CV_Scalar__rgb(double red, double green, double blue);

extern CV_Sequence CV_Sequence__approximate_polygon(CV_Sequence contour,
  int header_size, CV_Memory_Storage storage, int method,
  int parameter1, double parameter2);
extern double CV_Sequence__arc_length(
 CV_Sequence contour, CV_Slice slice, int is_closed);
extern bool CV_Sequence__check_contour_convexity(CV_Sequence contour);
extern double CV_Sequence__contour_area(
  CV_Sequence contour, CV_Slice slice, int oriented);
extern CV_Sequence CV_Sequence__next_get(CV_Sequence sequence);
extern CV_Point CV_Sequence__point_fetch1(CV_Sequence sequence, unsigned int index);
extern int CV_Sequence__total_get(CV_Sequence sequence);

extern CV_Size CV_Size__create(int width, int height);
extern void CV_Size__free(CV_Size cv_size);

extern void CV__release_image(CV_Image image);

#endif // !defined(CV_C_H_INCLUDED)
