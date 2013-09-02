/*
 * Header for OpenCV.
 *
 * Copyright (c) 2010 by Wayne C. Gramlich
 * All rights reserved.
 */

#if !defined(CV_C_H_INCLUDED)
#define CV_C_H_INCLUDED 1

#include "opencv/cv.h"
#include "opencv/highgui.h"

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

extern CvContour CV_Contour__Initial;
extern IplImage CV_Image__Initial;
/* extern CvCapture CV_Capture__Initial; */
extern CvSize CV_Size__Initial;
extern CvMat CV_Matrix__Initial;
extern CvMemStorage CV_Memory_Storage__Initial;
extern CvPoint CV_Point__Initial;
extern CvPoint2D32f CV_Point2D32F__Initial;
extern CvPoint2D32f CV_Point2D32F_Vector__Initial;
extern CvScalar CV_Scalar__Initial;
extern CvSlice CV_Slice__Initial;
extern CvTermCriteria CV_Term_Criteria__Initial;

extern CV_Image CV__tga_read(CV_Image image, String file_name);
extern void CV__tga_write(CV_Image image, String file_name);

#endif // !defined(CV_C_H_INCLUDED)
