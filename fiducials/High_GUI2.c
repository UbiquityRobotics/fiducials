// Copyright (c) 2010, 2013 by Wayne C. Gramlich.  All rights reserved.

//#include "opencv/cv.h"

#include <opencv2/highgui/highgui_c.h>

#include "CV.h"
#include "Double.h"
#include "High_GUI2.h"
#include "Integer.h"

Integer CV_Capture__property_pos_msec = CV_CAP_PROP_POS_MSEC;
Integer CV_Capture__property_frames = CV_CAP_PROP_POS_FRAMES;
Integer CV_Capture__property_avi_ratio = CV_CAP_PROP_POS_AVI_RATIO;
Integer CV_Capture__property_frame_width = CV_CAP_PROP_FRAME_WIDTH;
Integer CV_Capture__property_frame_height = CV_CAP_PROP_FRAME_HEIGHT;
Integer CV_Capture__property_fps = CV_CAP_PROP_FPS;
Integer CV_Capture__property_fourcc = CV_CAP_PROP_FOURCC;
Integer CV_Capture__property_frame_count = CV_CAP_PROP_FRAME_COUNT;
Integer CV_Capture__property_format = CV_CAP_PROP_FORMAT;
Integer CV_Capture__property_mode = CV_CAP_PROP_MODE;
Integer CV_Capture__property_brightness = CV_CAP_PROP_BRIGHTNESS;
Integer CV_Capture__property_contrast = CV_CAP_PROP_CONTRAST;
Integer CV_Capture__property_saturation = CV_CAP_PROP_SATURATION;
Integer CV_Capture__property_hue = CV_CAP_PROP_HUE;
Integer CV_Capture__property_gain = CV_CAP_PROP_GAIN;
Integer CV_Capture__property_convert_rgb = CV_CAP_PROP_CONVERT_RGB;


Integer CV__capture_any = CV_CAP_ANY;
Integer CV__capture_mil = CV_CAP_MIL;
Integer CV__capture_vfw = CV_CAP_VFW;
Integer CV__capture_v4l = CV_CAP_V4L;
Integer CV__capture_v4l2 = CV_CAP_V4L2;
Integer CV__capture_fireware = CV_CAP_FIREWARE;	/* They meant FIREWIRE */
Integer CV__capture_firewire = CV_CAP_FIREWARE;
Integer CV__capture_ieee1394 = CV_CAP_IEEE1394;
Integer CV__capture_dc1394 = CV_CAP_DC1394;
Integer CV__capture_cmu1394 = CV_CAP_CMU1394;
Integer CV__capture_stereo = CV_CAP_STEREO;
Integer CV__capture_tyzx = CV_CAP_TYZX;
Integer CV__tyzx_left = CV_TYZX_LEFT;
Integer CV__tyzx_right = CV_TYZX_RIGHT;
Integer CV__tyzx_color = CV_TYZX_COLOR;
Integer CV__capture_tyzx_z = CV_TYZX_Z;
Integer CV__capture_qt = CV_CAP_QT;

CV_Image CV_Capture__query_frame(CV_Capture capture) {
    return cvQueryFrame(capture);
}

CV_Capture CV_Capture__create_camera(Integer camera_number) {
    return cvCreateCameraCapture(camera_number);
}

CV_Capture CV_Capture__create_file(String capture_file_name) {
    return cvCreateFileCapture(capture_file_name);
}

Integer CV_Capture__set_property(
  CV_Capture capture, Integer property_id, Double value) {
    return cvSetCaptureProperty(capture, property_id, value);
}

CV_Video_Writer
CV__create_video_writer(
  String out_file_name,
  Integer four_cc,
  Double fps,
  CV_Size size,
  Integer is_color)
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
  Integer is_color)
{
    return cvLoadImage(image_file_name, is_color);
}

Integer
CV__named_window(
  String window_name,
  Integer flags)
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

Integer
CV_Video_Writer__write_frame(
  CV_Video_Writer writer,
  CV_Image frame)
{
    return cvWriteFrame(writer, frame);
}

// This routine will wait for a key event or delay for {delay} milliseconds.
// If {delay} is 0, the delay is infinite.

Integer CV__wait_key(Integer delay) {
    return cvWaitKey(delay);
}



