// Copyright (c) 2010, 2013 by Wayne C. Gramlich.  All rights reserved.

//#include "opencv/cv.hpp"

#include <opencv2/highgui/highgui_c.h>

#include "CV.hpp"
#include "Double.hpp"
#include "High_GUI2.hpp"

int CV_Capture__property_pos_msec = CV_CAP_PROP_POS_MSEC;
int CV_Capture__property_frames = CV_CAP_PROP_POS_FRAMES;
int CV_Capture__property_avi_ratio = CV_CAP_PROP_POS_AVI_RATIO;
int CV_Capture__property_frame_width = CV_CAP_PROP_FRAME_WIDTH;
int CV_Capture__property_frame_height = CV_CAP_PROP_FRAME_HEIGHT;
int CV_Capture__property_fps = CV_CAP_PROP_FPS;
int CV_Capture__property_fourcc = CV_CAP_PROP_FOURCC;
int CV_Capture__property_frame_count = CV_CAP_PROP_FRAME_COUNT;
int CV_Capture__property_format = CV_CAP_PROP_FORMAT;
int CV_Capture__property_mode = CV_CAP_PROP_MODE;
int CV_Capture__property_brightness = CV_CAP_PROP_BRIGHTNESS;
int CV_Capture__property_contrast = CV_CAP_PROP_CONTRAST;
int CV_Capture__property_saturation = CV_CAP_PROP_SATURATION;
int CV_Capture__property_hue = CV_CAP_PROP_HUE;
int CV_Capture__property_gain = CV_CAP_PROP_GAIN;
int CV_Capture__property_convert_rgb = CV_CAP_PROP_CONVERT_RGB;


int CV__capture_any = CV_CAP_ANY;
int CV__capture_mil = CV_CAP_MIL;
int CV__capture_vfw = CV_CAP_VFW;
int CV__capture_v4l = CV_CAP_V4L;
int CV__capture_v4l2 = CV_CAP_V4L2;
int CV__capture_fireware = CV_CAP_FIREWARE;	/* They meant FIREWIRE */
int CV__capture_firewire = CV_CAP_FIREWARE;
int CV__capture_ieee1394 = CV_CAP_IEEE1394;
int CV__capture_dc1394 = CV_CAP_DC1394;
int CV__capture_cmu1394 = CV_CAP_CMU1394;
int CV__capture_stereo = CV_CAP_STEREO;
int CV__capture_tyzx = CV_CAP_TYZX;
int CV__tyzx_left = CV_TYZX_LEFT;
int CV__tyzx_right = CV_TYZX_RIGHT;
int CV__tyzx_color = CV_TYZX_COLOR;
int CV__capture_tyzx_z = CV_TYZX_Z;
int CV__capture_qt = CV_CAP_QT;

CV_Image CV_Capture__query_frame(CV_Capture capture) {
    return cvQueryFrame(capture);
}

CV_Capture CV_Capture__create_camera(int camera_number) {
    return cvCreateCameraCapture(camera_number);
}

CV_Capture CV_Capture__create_file(String capture_file_name) {
    return cvCreateFileCapture(capture_file_name);
}

int CV_Capture__set_property(
  CV_Capture capture, int property_id, Double value) {
    return cvSetCaptureProperty(capture, property_id, value);
}

CV_Video_Writer
CV__create_video_writer(
  String out_file_name,
  int four_cc,
  Double fps,
  CV_Size size,
  int is_color)
{
    return cvCreateVideoWriter(out_file_name, four_cc, fps, *size, is_color);
}

void
CV__destroy_window(
  String window_name)
{
    cvDestroyWindow(window_name);
}

CV_Image
CV__load_image(
  String image_file_name,
  int is_color)
{
    return cvLoadImage(image_file_name, is_color);
}

int
CV__named_window(
  String window_name,
  int flags)
{
    return cvNamedWindow(window_name, flags);
}

void CV_Capture__release(CV_Capture capture) {
    cvReleaseCapture(&capture);
}

void
CV_Video_Writer__release_video_writer(
  CV_Video_Writer writer)
{
    cvReleaseVideoWriter(&writer);
}

void CV_Image__show(CV_Image image, String window_name) {
    cvShowImage(window_name, image);
}

int
CV_Video_Writer__write_frame(
  CV_Video_Writer writer,
  CV_Image frame)
{
    return cvWriteFrame(writer, frame);
}

// This routine will wait for a key event or delay for {delay} milliseconds.
// If {delay} is 0, the delay is infinite.

int CV__wait_key(int delay) {
    return cvWaitKey(delay);
}



