// Copyright (c) 2010, 2013 by Wayne C. Gramlich.  All rights reserved.

#if !defined(HIGH_GUI2_C_H_INCLUDED)
#define HIGH_GUI2_C_H_INCLUDED 1

#include <opencv2/highgui/highgui_c.h>

#include "Double.h"
#include "Integer.h"

typedef CvCapture *CV_Capture;
typedef CvVideoWriter *CV_Video_Writer;

extern CvCapture CV_Capture__Initial;
extern CvVideoWriter CV_Video_Writer__Initial;

extern CV_Capture CV__create_file_capture(String capture_file_name);
extern CV_Video_Writer CV__create_video_writer(String out_file_name,
  Integer four_cc, Double fps, CV_Size size, Integer is_color);
extern void CV__destroy_window(String window_name);
extern CV_Image CV__load_image(String image_file_name, Integer is_color);
extern Integer CV__named_window(String window_name, Integer flags);
extern void CV_Capture__release_capture(CV_Capture capture);
extern void CV_Video_Writer__release_video_writer( CV_Video_Writer writer);
extern void CV__show_image(String window_name, CV_Image image);
extern Integer CV_Video_Writer__write_frame(
  CV_Video_Writer writer, CV_Image frame);
extern Integer CV__wait_key(Integer delay);

#endif // !defined(HIGH_GUI2_C_H_INCLUDED)
