#ifndef IMAGE_SMOOTH_H
#define IMAGE_SMOOTH_H

/// Given an image of size widthxheight in srcImg, smooths the image using a
/// gaussian filter (cvSmooth) and copies the smoothed image to smoothImg If
/// sigma=1.0, then calls cvSmooth(srcImg, smoothedImg, CV_GAUSSIAN, 5, 5); This
/// is the default. If sigma>1.0, then calls cvSmooth(srcImg, smoothedImg,
/// CV_GAUSSIAN, 0, 0, sigma);
///
void SmoothImage(unsigned char *srcImg, unsigned char *smoothImg, int width,
                 int height, double sigma = 1.0);

#endif