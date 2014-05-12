// Copyright (c) 2010, 2013 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string.h>

#include "CV.hpp"
#include "Double.hpp"
#include "File.hpp"
#include "String.hpp"

static CvSlice whole_sequence;		// CV_WHOLE_SEQ (see below)
CV_Slice CV__whole_seq = &whole_sequence;

void CV_Slice__Initialize(void)
{
    whole_sequence = CV_WHOLE_SEQ;
}

// Depth constants:
int CV__depth_1u = IPL_DEPTH_1U;
int CV__depth_8u = IPL_DEPTH_8U;
int CV__depth_16u = IPL_DEPTH_16U;
int CV__depth_8s = IPL_DEPTH_8S;
int CV__depth_16s = IPL_DEPTH_16S;
int CV__depth_32s = IPL_DEPTH_32S;
int CV__depth_32f = IPL_DEPTH_32F;
int CV__depth_64f = IPL_DEPTH_64F;

int CV__load_image_any_color = CV_LOAD_IMAGE_ANYCOLOR;
int CV__load_image_any_depth = CV_LOAD_IMAGE_ANYDEPTH;
int CV__load_image_color = CV_LOAD_IMAGE_COLOR;
int CV__load_image_gray_scale = CV_LOAD_IMAGE_GRAYSCALE;
int CV__load_image_unchanged = CV_LOAD_IMAGE_UNCHANGED;
int CV__window_auto_size = CV_WINDOW_AUTOSIZE;

// Color space conversion codes:
int CV__bgr_to_bgra = CV_BGR2BGRA;
int CV__rgb_to_rgba = CV_RGB2RGBA;
int CV__bgra_to_bgr = CV_BGRA2BGR;
int CV__rgba_to_rgb = CV_RGBA2RGB;
int CV__bgr_to_rgba = CV_BGR2RGBA;
int CV__rgb_to_bgra = CV_RGB2BGRA;
int CV__rgba_to_bgr = CV_RGBA2BGR;
int CV__bgra_to_rgb = CV_BGRA2RGB;
int CV__bgr_to_rgb = CV_BGR2RGB;
int CV__rgb_to_bgr = CV_RGB2BGR;
int CV__brga_to_rgba = CV_BGRA2RGBA;
int CV__rgba_to_brga = CV_RGBA2BGRA;
int CV__bgr_to_gray = CV_BGR2GRAY;
int CV__rgb_to_gray = CV_RGB2GRAY;
int CV__gray_to_brg = CV_GRAY2BGR;
int CV__gray_to_rgb = CV_GRAY2RGB;
int CV__gray_to_bgra = CV_GRAY2BGRA;
int CV__gray_to_rgba = CV_GRAY2RGBA;
int CV__brga_to_gray = CV_BGRA2GRAY;
int CV__rgba_to_gray = CV_RGBA2GRAY;
int CV__bgr_to_bgr565 = CV_BGR2BGR565;
int CV__rgb_to_bgr565 = CV_RGB2BGR565;
int CV__bgr565_to_bgr = CV_BGR5652BGR;
int CV__bg4565_to_rgb = CV_BGR5652RGB;
int CV__bgra_to_bgr565 = CV_BGRA2BGR565;
int CV__rgba_to_bgr565 = CV_RGBA2BGR565;
int CV__bgr565_to_bgra = CV_BGR5652BGRA;
int CV__bgr565_to_rgba = CV_BGR5652RGBA;
int CV__gray_to_bgr565 = CV_GRAY2BGR565;
int CV__bgr565_to_gray =  CV_BGR5652GRAY;
int CV__bgr_to_bgr555 = CV_BGR2BGR555;
int CV__rgb_to_bgr555 = CV_RGB2BGR555;
int CV__bgr555_to_bgr = CV_BGR5552BGR;
int CV__bgr555_to_rgb = CV_BGR5552RGB;
int CV__bgra_to_bgr555 = CV_BGRA2BGR555;
int CV__rgba_to_bgr555 = CV_RGBA2BGR555;
int CV__bgr555_to_bgra = CV_BGR5552BGRA;
int CV__bgr555_to_rgba = CV_BGR5552RGBA;
int CV__gray_to_bgr555 = CV_GRAY2BGR555;
int CV__bgr555_to_gray = CV_BGR5552GRAY;
int CV__bgr_to_xyz = CV_BGR2XYZ;
int CV__rgb_to_xyz = CV_RGB2XYZ;
int CV__xyz_to_bgr = CV_XYZ2BGR;
int CV__xyz_to_rgb = CV_XYZ2RGB;
int CV__bgr_to_ycrcb = CV_BGR2YCrCb;
int CV__rgb_to_ycrcb = CV_RGB2YCrCb;
int CV__ycrcb_to_bgr = CV_YCrCb2BGR;
int CV__ycrcb_to_rgb = CV_YCrCb2RGB;
int CV__bgr_to_hsv = CV_BGR2HSV;
int CV__rgb_to_hsv = CV_RGB2HSV;
int CV__bgr_to_lab = CV_BGR2Lab;
int CV__rgb_to_lab = CV_RGB2Lab;
int CV__bayerbg_to_bgr = CV_BayerBG2BGR;
int CV__bayergb_to_bgr = CV_BayerGB2BGR;
int CV__bayerrg_to_bgr = CV_BayerRG2BGR;
int CV__bayergr_to_bgr = CV_BayerGR2BGR;
int CV__bayerbg_to_rgb = CV_BayerBG2RGB;
int CV__bayergb_to_rgb = CV_BayerGB2RGB;
int CV__bayerrg_to_rgb = CV_BayerRG2RGB;
int CV__bayergr_to_rgb = CV_BayerGR2RGB;
int CV__bgr_to_luv = CV_BGR2Luv;
int CV__rgb_to_luv = CV_RGB2Luv;
int CV__bgr_to_hls = CV_BGR2HLS;
int CV__rgb_to_hls = CV_RGB2HLS;
int CV__hsv_to_bgr = CV_HSV2BGR;
int CV__hsv_to_rgb = CV_HSV2RGB;
int CV__lab_to_bgr = CV_Lab2BGR;
int CV__lab_to_rgb = CV_Lab2RGB;
int CV__luv_to_bgr = CV_Luv2BGR;
int CV__luv_to_rgb = CV_Luv2RGB;
int CV__hls_to_bgr = CV_HLS2BGR;
int CV__hls_to_rgb = CV_HLS2RGB;

int CV__adaptive_thresh_mean_c = CV_ADAPTIVE_THRESH_MEAN_C;
int CV__adaptive_thresh_gaussian_c = CV_ADAPTIVE_THRESH_GAUSSIAN_C;
int CV__thresh_binary = CV_THRESH_BINARY;
int CV__thresh_binary_inv = CV_THRESH_BINARY_INV;

// Types for Matrices and images:
int CV__u8 = CV_8U;
int CV__s8 = CV_8S;
int CV__u16 = CV_16U;
int CV__s16 = CV_16S;
int CV__s32 = CV_32S;
int CV__f32 = CV_32F;
int CV__f64 = CV_64F;
int CV__user_type = CV_USRTYPE1;

int CV__u8c1 = CV_8UC1;
int CV__u8c2 = CV_8UC2;
int CV__u8c3 = CV_8UC3;
int CV__u8c4 = CV_8UC4;

int CV__s8c1 = CV_8SC1;
int CV__s8c2 = CV_8SC2;
int CV__s8c3 = CV_8SC3;
int CV__s8c4 = CV_8SC4;

int CV__u16c1 = CV_16UC1;
int CV__u16c2 = CV_16UC2;
int CV__u16c3 = CV_16UC3;
int CV__u16c4 = CV_16UC4;

int CV__s16c1 = CV_16SC1;
int CV__s16c2 = CV_16SC2;
int CV__s16c3 = CV_16SC3;
int CV__s16c4 = CV_16SC4;

int CV__s32c1 = CV_32SC1;
int CV__s32c2 = CV_32SC2;
int CV__s32c3 = CV_32SC3;
int CV__s32c4 = CV_32SC4;

int CV__f32c1 = CV_32FC1;
int CV__f32c2 = CV_32FC2;
int CV__f32c3 = CV_32FC3;
int CV__f32c4 = CV_32FC4;

int CV__f64c1 = CV_64FC1;
int CV__f64c2 = CV_64FC2;
int CV__f64c3 = CV_64FC3;
int CV__f64c4 = CV_64FC4;

int CV__auto_step = CV_AUTO_STEP;

int CV__blur_no_scale = CV_BLUR_NO_SCALE;
int CV__blur = CV_BLUR;
int CV__gaussian = CV_GAUSSIAN;
int CV__median = CV_MEDIAN;
int CV__bilateral = CV_BILATERAL;

// *CV* routines:

void CV__draw_chessboard_corners(CV_Image image, CV_Size pattern_size,
  CV_Point2D32F_Vector corners, int count, bool pattern_was_found) {
    cvDrawChessboardCorners(image,
      *pattern_size, corners, count, pattern_was_found);
}

CV_Image CV__clone_image(CV_Image image) {
    return cvCloneImage(image);
}

//void
//CV__calibrate_camera2(
//  CV_Matrix object_points,
//  CV_Matrix image_points,
//  CV_Matrix point_counts,
//  CV_Size image_size,
//  CV_Matrix camera_matrix,
//  CV_Matrix distortion_coefficients, 
//  CV_Matrix rotation_vectors,
//  CV_Matrix translation_vectors,
//  int flags)
//{
//    cvCalibrateCamera2(object_points, image_points, point_counts,
//      *image_size, camera_matrix, distortion_coefficients, rotation_vectors,
//      translation_vectors, flags);
//}

int CV__find_chessboard_corners(CV_Image image, CV_Size pattern_size,
  CV_Point2D32F_Vector corners, int flags) {
    int corner_count;
    int result;

    result = cvFindChessboardCorners(image,
      *pattern_size, corners, &corner_count, flags);
    if (result != 0) {
	result = corner_count;
    }
    return result;
}

void CV__find_extrinsic_camera_params2(
  CV_Matrix object_points,
  CV_Matrix image_points,
  CV_Matrix camera,
  CV_Matrix distortion_coefficients,
  CV_Matrix rotation_vector,
  CV_Matrix translation_vector,
  int use_extrinsic_guess)
{
  cvFindExtrinsicCameraParams2(object_points, image_points, camera,
    distortion_coefficients, rotation_vector, translation_vector,
    use_extrinsic_guess);
}

int CV__fourcc(char character1, char character2,
  char character3, char character4) {
    return CV_FOURCC(character1, character2, character3, character4);
}

int CV__gemm_a_t = CV_GEMM_A_T;
int CV__gemm_b_t = CV_GEMM_B_T;
int CV__gemm_c_t = CV_GEMM_C_T;

void CV__gemm(CV_Matrix a, CV_Matrix b, double alpha, CV_Matrix c, double beta,
  CV_Matrix d,int transpose_a_b_c) {
    return cvGEMM(a, b, alpha, c, beta, d, transpose_a_b_c);
}


Double CV__get_real_2d(CV_Matrix matrix, int row, int column) {
    return cvGetReal2D(matrix, row, column);
}

void CV__init_undistort_map(CV_Matrix camera_matrix, 
  CV_Matrix distortion_coefficients, CV_Matrix mapx, CV_Matrix mapy) {
    cvInitUndistortMap(camera_matrix, distortion_coefficients, mapx, mapy);
}

//CV_Matrix
//CV__load(
//  String file_name,
//  CV_Memory_Storage storage,
//  String name,
//  String real_name)
//{
//    char *simple_file_name;
//    char *simple_name;
//    const char *simple_real_name;
//
//    simple_name = (char *)0;
//    simple_real_name = (const char *)0;
//    return cvLoad(file_name, storage, simple_name, &simple_real_name);
//}

void CV__release_image(CV_Image image) {
    cvReleaseImage(&image);
}

int CV__inter_linear = CV_INTER_LINEAR;
int CV__warp_fill_outliers = CV_WARP_FILL_OUTLIERS;

void CV__rodrigues2(
  CV_Matrix rotation_vector, CV_Matrix rotation_matrix, CV_Matrix jacobian) {
    cvRodrigues2(rotation_vector, rotation_matrix, jacobian);
}

void CV__set_identity(CV_Matrix matrix, CV_Scalar scalar) {
    cvSetIdentity(matrix, *scalar);
}

void CV__set_real_2d(
  CV_Matrix matrix, int row, int column, Double value) {
    cvSetReal2D(matrix, row, column, value);
}

void CV__set_zero(CV_Matrix matrix) {
    cvSetZero(matrix);
}

int CV__round(Double value) {
    return cvRound(value);
}

// Read the calibration file and generate the undistortion maps
// in:
//   calibrate_file_name   -  camera calibration file
//   w h                   -  width and height of images to undistort
// out:
//   mapx, mapy,           - undistortion maps

int CV__undistortion_setup(String_Const calibrate_file_name,
 int width, int height, CV_Image *mapx, CV_Image *mapy) {
    Double fcx, fcy, ccx, ccy;
    Double kc[4];
  
    // Open *calibrate_file_name*:
    File file = File__open(calibrate_file_name, "r");
    if (file == (File)0) {
        File__format(stderr, "Could not open \"%s\"\n", calibrate_file_name);
        return -1;
    }

    // Scan in the calibration values:
    //  format is fc - focal length, cc, principal point, kc distortion vector
    int x = fscanf(file, "fc %lf %lf cc %lf %lf kc %lf %lf %lf %lf", 
       &fcx, &fcy, &ccx, &ccy, &kc[0], &kc[1], &kc[2], &kc[3]);
    if (x != 8) {
        File__format(stderr, "Expected 8 parameters got %d\n", x);
        return -1;
    }
    File__close(file);
    
    // Create *intrisic* matrix:
    double intrinsic_vector[9] = {
        fcx,   0, ccx,
          0, fcy, ccy,
          0,   0,   1
    }; 
    CvMat intrinsic = cvMat(3, 3, CV_64FC1, intrinsic_vector);
    //printf("intrinsic matrix\n");
    //dumpMat(&intrinsic);

    // Create *distortion* matrix*:
    CvMat distortion = cvMat(1, 4, CV_64FC1, kc);
    //printf("distortion matrix\n");
    //dumpMat(&distortion);

    *mapx = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    *mapy = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    cvInitUndistortMap(&intrinsic, &distortion, *mapx, *mapy);

    return 0;
}

// *CV_Image* routines:

void CV_Image__adaptive_threshold(CV_Image source_image,
  CV_Image destination_image, Double maximum_value, int adaptive_method,
  int threshold_type, int block_size, Double parameter1) {
    cvAdaptiveThreshold(source_image, destination_image, maximum_value,
      adaptive_method, threshold_type, block_size, parameter1);
}

void CV_Image__blob_draw(
  CV_Image image, int x, int y, CV_Scalar color) {
    // Draw a small cross at the indicated point.
    uchar red = cvRound(color->val[0]);
    uchar green = cvRound(color->val[1]);
    uchar blue = cvRound(color->val[2]);
    uchar *data = (uchar *)image->imageData;
    int width_step = image->widthStep;

    // Sanity check the values.
    if (x < 2 || y < 2 || x >= (image->width - 2) || y >= (image->height - 2)) {
	return;
    }

    // Draw away:
    for (int i = -2; i <= 2; i++) {
	for (int j = -2; j <= 2; j++) {
	  uchar *pixel = &(data + width_step * (y + j))[(x + i) * 3];
	  pixel[0] = red; pixel[1] = green; pixel[2] = blue;
	}
    }
}

CV_Image CV_Image__create(CV_Size size, unsigned int depth, unsigned int channels) {
    return cvCreateImage(*size, depth, channels);
}

int CV_Image__channels_get(CV_Image image) {
    return image->nChannels;
}

void CV_Image__convert_color(
 CV_Image source_image, CV_Image destination_image, int conversion_code) {
    cvCvtColor(source_image, destination_image, conversion_code);
}

void CV_Image__copy(
  CV_Image source_image, CV_Image destination_image, CV_Image mask) {
    cvCopy(source_image, destination_image, mask);
}

void CV_Image__cross_draw(
  CV_Image image, int x, int y, CV_Scalar color) {
    // Draw a small cross at the indicated point.
    uchar *pixel;
    uchar *pixel_lt;
    uchar *pixel_rt;
    uchar *pixel_up;
    uchar *pixel_dn;
    uchar red = cvRound(color->val[0]);
    uchar green = cvRound(color->val[1]);
    uchar blue = cvRound(color->val[2]);
    uchar *data = (uchar *)image->imageData;
    int width_step = image->widthStep;

    // Sanity check the values.
    if (x < 1 || y < 1 || x >= (image->width - 1) || y >= (image->height - 1)) {
	return;
    }

    // Get the pixels to be colored.
    pixel = &((data + width_step * y))[x * 3];
    pixel_lt = &((data + width_step * y))[(x - 1) * 3];
    pixel_rt = &((data + width_step * y))[(x + 1) * 3];
    pixel_up = &((data + width_step * (y - 1)))[x * 3];
    pixel_dn = &((data + width_step * (y + 1)))[x * 3];

    // Draw the cross.
    pixel[0] = red; pixel[1] = green; pixel[2] = blue;
    pixel_lt[0] = red; pixel_lt[1] = green; pixel_lt[2] = blue;
    pixel_rt[0] = red; pixel_rt[1] = green; pixel_rt[2] = blue;
    pixel_up[0] = red; pixel_up[1] = green; pixel_up[2] = blue;
    pixel_dn[0] = red; pixel_dn[1] = green; pixel_dn[2] = blue;
}

void CV_Image__draw_contours(CV_Image image, CV_Sequence contour,
  CV_Scalar external_color, CV_Scalar hole_color, int maximal_level,
  int thickness, int line_type, CV_Point offset) {
  cvDrawContours(image, contour, *external_color,
    *hole_color, maximal_level, thickness, line_type, *offset);
}

// {mode} constants:
int CV__retr_external = CV_RETR_EXTERNAL;
int CV__retr_list = CV_RETR_LIST;
int CV__retr_ccomp = CV_RETR_CCOMP;
int CV__retr_tree = CV_RETR_TREE;

// {method} constants:
int CV__chain_code = CV_CHAIN_CODE;
int CV__chain_approx_none = CV_CHAIN_APPROX_NONE;
int CV__chain_approx_simple = CV_CHAIN_APPROX_SIMPLE;
int CV__chain_approx_tc89_l1 = CV_CHAIN_APPROX_TC89_L1;
int CV__chain_approx_tc89_kcos = CV_CHAIN_APPROX_TC89_KCOS;
int CV__chain_link_runs = CV_LINK_RUNS;

int CV__calib_cb_adaptive_thresh = CV_CALIB_CB_ADAPTIVE_THRESH;
int CV__calib_cb_normalize_image = CV_CALIB_CB_NORMALIZE_IMAGE;
int CV__calib_cb_filter_quads = CV_CALIB_CB_FILTER_QUADS;

CV_Sequence CV_Image__find_contours(CV_Image image, CV_Memory_Storage storage,
  int header_size, int mode, int method, CV_Point point) {
    int result;
    CV_Sequence contours;

    contours = (CV_Sequence)0;
    result = cvFindContours(image,
      storage, &contours, sizeof(CvContour), mode, method, *point);
    return contours;
}  

unsigned int CV_Image__depth_get(CV_Image image) {
    return image->depth;
}

unsigned int CV_Image__fetch3(
  CV_Image image, unsigned int x, unsigned int y, unsigned int channel) {
    unsigned char *pointer = cvPtr2D(image, y, x, (int *)0);
    return pointer[channel];
}

int CV_Image__gray_fetch(CV_Image image, int x, int y) {
    int result = -1;
    if (0 <= x && x < image->width && 0 <= y && y < image->height) {
	result =
	  (int)(((uchar *)image->imageData + image->widthStep * y))[x];
    }
    return result;
}

void CV_Image__find_corner_sub_pix(CV_Image image, CV_Point2D32F_Vector corners,
  int count, CV_Size window, CV_Size zero_zone, CV_Term_Criteria criteria) {
    cvFindCornerSubPix(image, corners, count, *window, *zero_zone, *criteria);
}

void CV_Image__flip(CV_Image from_image, CV_Image to_image, int flip_code) {
    cvFlip(from_image, to_image, flip_code);
}

CV_Image CV_Image__header_create(
  CV_Size size, unsigned int depth, unsigned int channels) {
    return cvCreateImageHeader(*size, depth, channels);
}

int CV_Image__height_get(CV_Image image) {
    return image->height;
}

/// @brief Reads in a *CV_Image* in from the .pnm file named *file_name*.
/// @param file_name is the base name (excluding suffix) to write out to.
/// @returns the *CV_Image* corresponding to the file read in.
///
/// *CV_Image__pnm_read*() will reads in and return a *CV_Image* in from
/// the file .pnm file named *file_name*.

CV_Image CV_Image__pnm_read(String_Const file_name) {
    unsigned int size = String__size(file_name);
    assert (String__equal(file_name + size - 4, ".pnm"));
    CV_Image image = cvLoadImage(file_name, CV_LOAD_IMAGE_UNCHANGED);
    if (image == (CV_Image)0) {
	File__format(stderr, "Unable to open file '%s'\n", file_name);
	assert(0);
    }
    return image;
}

/// @brief Writes *image* out to .pnm file named *file_name*.
/// @param image to write out.
/// @param file_name is the *file_name to write out to.
///
/// *CV_Image__pnm_write*() will write *image* out to the file named
/// *file_base_name.pnm.

void CV_Image__pnm_write(CV_Image image, String_Const file_name) {
    unsigned int size = String__size(file_name);
    assert (String__equal(file_name + size - 4, ".pnm"));
    cvSaveImage(file_name, image, (int *)0);
}

void CV_Image__remap(CV_Image source_image, CV_Image destination_image,
  CV_Image map_x, CV_Image map_y, int flags, CV_Scalar fill_value) {
    cvRemap(source_image,
      destination_image, map_x, map_y, flags, *fill_value);
}

void CV_Image__smooth(CV_Image source_image, CV_Image destination_image,
  int smooth_type, int parameter1, int parameter2,
  Double parameter3, Double parameter4) {
    cvSmooth(source_image, destination_image, smooth_type, parameter1,
      parameter2, parameter3, parameter4);
}

void CV_Image__store3(
  CV_Image image, unsigned int x, unsigned int y, unsigned int channel, unsigned int value) {
    unsigned char *pointer = cvPtr2D(image, y, x, (int *)0);
    pointer[channel] = (unsigned char)value;
}

/// @brief Read in a .tga file.
/// @param image to read into (or null).
/// @param tga_file_name is the file name of the .tga file.
/// @returns image from .tga file.
///
/// *CV__tga_read will read the contents of {tga_file_name} into
/// {image}.  If the sizes do not match, {image} is released
/// and new {CV_Image} object of the right size is allocated,
/// filled and returned.  In either case, the returned {CV_Image}
/// object containing the read in image data is returned.

CV_Image CV_Image__tga_read(CV_Image image, String_Const tga_file_name) {
    // Open *tga_in_file*:
    File tga_in_file = File__open(tga_file_name, "rb");
    if (tga_in_file == (File)0) {
	File__format(stderr,
	  "Unable to open '%s' for reading\n", tga_file_name);
	assert (0);
    }

    // Read .tga header from *tga_in_file*:
    File__byte_read(tga_in_file);			// identsize
    File__byte_read(tga_in_file);			// colourmaptype
    unsigned int image_type =
      File__byte_read(tga_in_file);		       // imagetype (3=>raw b&w)
    File__little_endian_short_read(tga_in_file);	// colourmapstart
    File__little_endian_short_read(tga_in_file);	// colourmaplength
    File__byte_read(tga_in_file);			// colourmapbits
    File__little_endian_short_read(tga_in_file);	// xstart
    File__little_endian_short_read(tga_in_file);	// ystart
    unsigned int width =
      File__little_endian_short_read(tga_in_file);	// width
    unsigned int height =
      File__little_endian_short_read(tga_in_file);	// height
    unsigned int bpp = File__byte_read(tga_in_file);	// bits per pixel
    File__byte_read(tga_in_file);			// descriptor

    // Compare {image_type}, {bpp}, {width} and {height} with {image}.
    if (image != (CV_Image)0 && (unsigned int)image->width == width &&
      (unsigned int)image->height == height && (unsigned int)image->depth == 8) {
	// {width}, {height}, and {depth} match {image}:
	unsigned int channels = (unsigned int)image->nChannels;
	if (image_type == 3 && channels == 1 && bpp == 8) {
	    // We have a gray mode .tga match; do nothing:
	} else if (image_type == 2 && channels == 3 && bpp == 24) {
	    // We have a color mode .tga file; do nothing:
	} else {
	    // No match:
	    CV__release_image(image);
	    image = (CV_Image)0;
	}
    } else {
        CV__release_image(image);
	image = (CV_Image)0;
    }
    // If {image} is not equal to {null@CV_Image} we can reuse {image}:

    // Figure out whether .tga file is gray scale or color:
    bool gray_mode = (bool)0;
    if (image_type == 3 && bpp == 8) {
	gray_mode = (bool)1;
    } else if (image_type == 2 && bpp == 24) {
	// Color mode:
	gray_mode = (bool)0;
    } else {
	// Something else:
	File__format(stderr, "'%s' has image type=%d and bpp=%d\n",
	  tga_file_name, image_type, bpp);
	assert (0);
    }

    // Allocate a new {image} if we need to:
    if (image == (CV_Image)0) {
	CV_Size size = CV_Size__create((int)width, (int)height);
	if (gray_mode) {
	    image = CV_Image__create(size, 8, 1);
	} else {
	    image = CV_Image__create(size, 8, 3);
	}
    }

    // Read the .tga data into {image}:
    for (unsigned int row = 0; row < height; row++) {
	unsigned int j = height - row - 1;
	for (unsigned int column = 0; column < width; column++) {
	    unsigned int i = column;
	    if (gray_mode) {
		unsigned int gray = fgetc(tga_in_file);
		CV_Image__store3(image, column, row, 0, gray);
	    } else {
	        //unsigned int red = File__byte_read(tga_in_file);
		//unsigned int green = File__byte_read(tga_in_file);
		//unsigned int blue = File__byte_read(tga_in_file);
		unsigned int red = fgetc(tga_in_file);
		unsigned int green = fgetc(tga_in_file);
		unsigned int blue = fgetc(tga_in_file);
		unsigned char *pointer = cvPtr2D(image, row, column, (int *)0);
		pointer[0] = red;
		pointer[1] = green;
		pointer[2] = blue;
		//CV_Image__store3(image, i, j, 0, red);
		//CV_Image__store3(image, i, j, 1, green);
		//CV_Image__store3(image, i, j, 2, blue);
	    }
	}
    }

    // Close {tga_in_file}:
    File__close(tga_in_file);

    return image;
}

/// @brief Write *image* out to *file_name* in .tga format.
/// @param image to write out.
/// @param file_name to write *image* to.
///
/// *CV_Image__tga_write*() will write *image* out to *file_name* in
/// .tga format.

void CV_Image__tga_write(CV_Image image, String_Const file_name) {
    unsigned int channels = (unsigned int)image->nChannels;
    unsigned int depth = (unsigned int)image->depth;
    unsigned int height = (unsigned int)image->height;
    unsigned int width = (unsigned int)image->width;
    assert (depth == 8);

    unsigned int bpp = 0;
    unsigned int image_type = 0;	// 2=>color; 3=>b&w:
    bool gray_mode = (bool)0;
    if (channels == 1) {
	// Gray scale:
	bpp = 8;
	gray_mode = (bool)1;
	image_type = 3;
    } else if (channels == 3) {
	// Color:
	bpp = 24;
	gray_mode = (bool)0;
	image_type = 2;
    } else {
	assert (0);
    }

    // Open {file_name} file for writing:
    File tga_out_file = File__open(file_name, "wb");
    if (tga_out_file == (File)0) {
	File__format(stderr, "Could not open '%s for writing.\n", file_name);
	assert (0);
    }

    // Write .tga header:
    File__byte_write(tga_out_file, 0);			// identsize
    File__byte_write(tga_out_file, 0);			// colourmaptype
    File__byte_write(tga_out_file, image_type);		// type (3=b&w)
    File__little_endian_short_write(tga_out_file, 0);	// colormapstart
    File__little_endian_short_write(tga_out_file, 0);	// colourmaplen
    File__byte_write(tga_out_file, 0);			// colourmapbits
    File__little_endian_short_write(tga_out_file, 0);	// xstart
    File__little_endian_short_write(tga_out_file, 0);	// ystart
    File__little_endian_short_write(tga_out_file, width); // width
    File__little_endian_short_write(tga_out_file, height); //height
    File__byte_write(tga_out_file, bpp);		// bits/pixel
    File__byte_write(tga_out_file, 0);			// descriptor

    // Write out the .tga file data:
    for (unsigned int row = 0; row < height; row++) {
	unsigned int j = height - row - 1;
	for (unsigned int column = 0; column < width; column++) {
	    unsigned int i = column;
	    if (gray_mode) {
	        unsigned int gray = CV_Image__fetch3(image, i, j, 0);
		File__byte_write(tga_out_file, gray);
	    } else {
		unsigned int index = height - row - 1;
		unsigned int red = CV_Image__fetch3(image, i, j, 0);
		unsigned int green = CV_Image__fetch3(image, i, j, 1);
		unsigned int blue = CV_Image__fetch3(image, i, j, 2);
		File__byte_write(tga_out_file, red);
		File__byte_write(tga_out_file, green);
		File__byte_write(tga_out_file, blue);
	    }
	}
    }

    // Close the .tga file:
    File__close(tga_out_file);
}

int CV_Image__width_get(CV_Image image) {
    return (unsigned int)image->width;
}

// *CV_Matrix* routines:

int CV_Matrix__columns_get(CV_Matrix matrix) {
    return matrix->cols;
}

int CV_Matrix__rows_get(CV_Matrix matrix) {
    return matrix->rows;
}

void CV_Matrix__save(CV_Matrix matrix, const char * file_name) {
    CvAttrList attributes;

    attributes = cvAttrList((const char **)0, (CvAttrList *)0);
    cvSave(file_name,
      matrix, (const char *)0, (const char *)0, attributes);
}

// *CV_Memory_Storage* routines:

void CV_Memory_Storage__clear(CV_Memory_Storage storage) {
    cvClearMemStorage(storage);
}

CV_Memory_Storage CV_Memory_Storage__create(int block_size) {
    return cvCreateMemStorage(block_size);
}

// *CV_Point* routines:

CV_Point CV_Point__create(int x, int y) {
    unsigned int malloc_bytes = sizeof *((CV_Point)0);
    // (void)printf("CV_Point__create: malloc_bytes=%d\n", malloc_bytes);
    CV_Point point = (CV_Point) malloc(sizeof *((CV_Point *)0) );

    point->x = x;
    point->y = y;
    return point;
}

int CV_Point__x_get(CV_Point point) {
    return point->x;
}

void CV_Point__x_set(CV_Point point, int x) {
    point->x = x;
}

int CV_Point__y_get(CV_Point point) {
    return point->y;
}

void CV_Point__y_set(CV_Point point, int y) {
    point->y = y;
}

// *CV_Point2D32F* routines:

CV_Point2D32F CV_Point2D32F__create(Double x, Double y) {
    unsigned int malloc_bytes = sizeof *((CV_Point2D32F)0);
    // (void)printf("CV_Point2D32F__create: malloc_bytes=%d\n",
    //     malloc_bytes);
    CV_Point2D32F point = (CV_Point2D32F)malloc(malloc_bytes);

    point->x = x;
    point->y = y;
    return point;
}

Double CV_Point2D32F__x_get(CV_Point2D32F point) {
    return point->x;
}

void CV_Point2D32F__x_set(CV_Point2D32F point, Double x) {
    point->x = x;
}

Double CV_Point2D32F__y_get(CV_Point2D32F point) {
    return point->y;
}

void CV_Point2D32F__y_set(CV_Point2D32F point, Double y) {
    point->y = y;
}

void CV_Point2D32F__point_set(CV_Point2D32F point2d32f, CV_Point point) {
    int x = CV_Point__x_get(point);
    int y = CV_Point__y_get(point);
    (void)CV_Point2D32F__x_set(point2d32f, (double)x);
    (void)CV_Point2D32F__y_set(point2d32f, (double)y);
}

// *CV_Point2D32F_Vector* routines:

CV_Point2D32F_Vector CV_Point2D32F_Vector__create(unsigned int size) {
    unsigned int malloc_bytes = size * sizeof *((CV_Point2D32F)0);
    // (void)printf("CV_Point2D32F_Vector__create: size=%d malloc_bytes=%d\n",
    //   size, malloc_bytes);
    CV_Point2D32F vector = (CV_Point2D32F)malloc(malloc_bytes);
    unsigned int index;

    for (index = 0; index < size; index++) {
        vector[index].x = 0.0;
        vector[index].y = 0.0;
    }
    return vector;
}

CV_Point2D32F CV_Point2D32F_Vector__fetch1(
  CV_Point2D32F_Vector vector,  unsigned int index) {
    return &vector[index];
}

// CV_Scalar *routines*:

CV_Scalar CV_Scalar__create(
  Double value0, Double value1, Double value2, Double value3) {
    CV_Scalar scalar = Memory__new(CV_Scalar, "CV_Scalar__create");
    scalar->val[0] = value0;
    scalar->val[1] = value1;
    scalar->val[2] = value2;
    scalar->val[3] = value3;
    return scalar;
}

void CV_Scalar__free(CV_Scalar cv_scalar) {
    Memory__free((Memory)cv_scalar);
}

// This routine will return a {CV_Scalar} that encodes {red}, {green},
// and {blue} as a color.

CV_Scalar CV_Scalar__rgb(Double red, Double green, Double blue) {
    return CV_Scalar__create(blue, green, red, 0.0);
}

// *CV_Sequence* routines:

int CV__poly_approx_dp = CV_POLY_APPROX_DP;

CV_Sequence CV_Sequence__approximate_polygon(CV_Sequence contour,
  int header_size, CV_Memory_Storage storage, int method,
  int parameter1, Double parameter2) {
    //(void)printf("sizeof=%d method=%d param1=%d\n",
    //  sizeof(CvContour), method, parameter1);
    return cvApproxPoly(contour,
      sizeof(CvContour), storage, method, parameter1, parameter2);
}

Double CV_Sequence__arc_length(
  CV_Sequence contour, CV_Slice slice, int is_closed) {
    return cvArcLength(contour, *slice, is_closed);
}    

bool CV_Sequence__check_contour_convexity(CV_Sequence contour) {
    return (bool)(cvCheckContourConvexity(contour) ? 1 : 0);
}

Double CV_Sequence__contour_area(
  CV_Sequence contour, CV_Slice slice, int oriented) {
    return cvContourArea(contour, *slice, oriented);
}

CV_Sequence CV_Sequence__next_get(CV_Sequence sequence) {
    return sequence->h_next;
}

CV_Point CV_Sequence__point_fetch1(CV_Sequence sequence, unsigned int index) {
    return (CV_Point)cvGetSeqElem(sequence, index);
}

int CV_Sequence__total_get(CV_Sequence sequence) {
    return sequence->total;
}

// *CV_Size* routines:

CV_Size CV_Size__create(int width, int height) {
    CV_Size size = Memory__new(CV_Size, "CV_Size__create");
    size->width = (int)width;
    size->height = (int)height;
    return size;
}

extern void CV_Size__free(CV_Size cv_size) {
    Memory__free((Memory)cv_size);
}

// *CV_Slice* routines:
CV_Slice CV_Slice__create(int start_index, int end_index) {
    unsigned int malloc_bytes = sizeof *((CV_Slice)0);
    // (void)printf("CV_Slice__create: malloc_bytes=%d\n", malloc_bytes);
    CV_Slice slice = (CV_Slice)malloc(malloc_bytes);

    slice->start_index = start_index;
    slice->end_index = end_index;
    return slice;
}

// *CV_Term* rouitines:
int CV__term_criteria_iterations = CV_TERMCRIT_ITER;
int CV__term_criteria_eps = CV_TERMCRIT_EPS;

CV_Term_Criteria CV_Term_Criteria__create(
  int type, int maximum_iterations, Double epsilon) {
    unsigned int malloc_bytes = sizeof *((CV_Term_Criteria)0);
    // (void)printf("CV_Term_Criteria__create: malloc_bytes=%d\n",
    //   malloc_bytes);
    CV_Term_Criteria term_criteria = (CV_Term_Criteria)malloc(malloc_bytes);

    term_criteria->type = type;
    term_criteria->max_iter = maximum_iterations;
    term_criteria->epsilon = epsilon;
    return term_criteria;
}

