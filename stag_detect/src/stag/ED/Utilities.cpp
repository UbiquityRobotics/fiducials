// #include <windows.h>
#include <stdio.h>
#include <math.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

// Burak - commented the line below
//#include "ImageVideoLib.h"
#include "stag/ED/Utilities.h"
// #include "Timer.h"

// Burak - suppresses _CRT_SECURE_NO_DEPRECATE warnings
#pragma warning(disable : 4996)

/*
RGB 2 XYZ

var_R = ( R / 255 )        //R from 0 to 255
var_G = ( G / 255 )        //G from 0 to 255
var_B = ( B / 255 )        //B from 0 to 255

if ( var_R > 0.04045 ) var_R = ( ( var_R + 0.055 ) / 1.055 ) ^ 2.4
else                   var_R = var_R / 12.92
if ( var_G > 0.04045 ) var_G = ( ( var_G + 0.055 ) / 1.055 ) ^ 2.4
else                   var_G = var_G / 12.92
if ( var_B > 0.04045 ) var_B = ( ( var_B + 0.055 ) / 1.055 ) ^ 2.4
else                   var_B = var_B / 12.92

var_R = var_R * 100
var_G = var_G * 100
var_B = var_B * 100

//Observer. = 2�, Illuminant = D65
X = var_R * 0.4124 + var_G * 0.3576 + var_B * 0.1805
Y = var_R * 0.2126 + var_G * 0.7152 + var_B * 0.0722
Z = var_R * 0.0193 + var_G * 0.1192 + var_B * 0.9505

*/

/*
XYZ 2 Lab

var_X = X / ref_X          //ref_X =  95.047   Observer= 2�, Illuminant= D65
var_Y = Y / ref_Y          //ref_Y = 100.000
var_Z = Z / ref_Z          //ref_Z = 108.883

if ( var_X > 0.008856 ) var_X = var_X ^ ( 1/3 )
else                    var_X = ( 7.787 * var_X ) + ( 16 / 116 )
if ( var_Y > 0.008856 ) var_Y = var_Y ^ ( 1/3 )
else                    var_Y = ( 7.787 * var_Y ) + ( 16 / 116 )
if ( var_Z > 0.008856 ) var_Z = var_Z ^ ( 1/3 )
else                    var_Z = ( 7.787 * var_Z ) + ( 16 / 116 )

CIE-L* = ( 116 * var_Y ) - 16
CIE-a* = 500 * ( var_X - var_Y )
CIE-b* = 200 * ( var_Y - var_Z )

*/
void MyRGB2Lab(unsigned char *redImg, unsigned char *greenImg,
               unsigned char *blueImg, unsigned char *LImg, unsigned char *aImg,
               unsigned char *bImg, int width, int height) {
  // First RGB 2 XYZ
  double red, green, blue;
  double x, y, z;

  double *L = new double[width * height];
  double *a = new double[width * height];
  double *b = new double[width * height];

  for (int i = 0; i < width * height; i++) {
    red = redImg[i] / 255.0;
    green = greenImg[i] / 255.0;
    blue = blueImg[i] / 255.0;

#if 1
    if (red > 0.04045)
      red = pow(((red + 0.055) / 1.055), 2.4);
    else
      red = red / 12.92;

    if (green > 0.04045)
      green = pow(((green + 0.055) / 1.055), 2.4);
    else
      green = green / 12.92;

    if (blue > 0.04045)
      blue = pow(((blue + 0.055) / 1.055), 2.4);
    else
      blue = blue / 12.92;
#endif

    red = red * 100;
    green = green * 100;
    blue = blue * 100;

    // Observer. = 2�, Illuminant = D65
    x = red * 0.4124564 + green * 0.3575761 + blue * 0.1804375;
    y = red * 0.2126729 + green * 0.7151522 + blue * 0.0721750;
    z = red * 0.0193339 + green * 0.1191920 + blue * 0.9503041;

    // Now xyz 2 Lab
    double refX = 95.047;
    double refY = 100.000;
    double refZ = 108.883;

    x = x / refX;  // ref_X =  95.047   Observer= 2�, Illuminant= D65
    y = y / refY;  // ref_Y = 100.000
    z = z / refZ;  // ref_Z = 108.883

    if (x > 0.008856)
      x = pow(x, 1.0 / 3.0);
    else
      x = (7.787 * x) + (16.0 / 116.0);

    if (y > 0.008856)
      y = pow(y, 1.0 / 3.0);
    else
      y = (7.787 * y) + (16.0 / 116.0);

    if (z > 0.008856)
      z = pow(z, 1.0 / 3.0);
    else
      z = (7.787 * z) + (16.0 / 116.0);

    L[i] = (116.0 * y) - 16;
    a[i] = 500 * (x / y);  // This is wrong but gives better results. Why?
    //    a[i] = 500*(x-y); // This is the correct formula
    b[i] = 200 * (y - z);
  }  // end-for

  // Scale L to [0-255]
  double min = 1e10;
  double max = -1e10;
  for (int i = 0; i < width * height; i++) {
    if (L[i] < min)
      min = L[i];
    else if (L[i] > max)
      max = L[i];
  }  // end-for

  double scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    LImg[i] = (unsigned char)((L[i] - min) * scale);
  }

  //  printf("L: min: %4.2lf, max: %4.2lf\n", min, max);

  // Scale a to [0-255]
  min = 1e10;
  max = -1e10;
  for (int i = 0; i < width * height; i++) {
    if (a[i] < min)
      min = a[i];
    else if (a[i] > max)
      max = a[i];
  }  // end-for

  scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    aImg[i] = (unsigned char)((a[i] - min) * scale);
  }

  //  printf("a: min: %4.2lf, max: %4.2lf\n", min, max);

  // Scale b to [0-255]
  min = 1e10;
  max = -1e10;
  for (int i = 0; i < width * height; i++) {
    if (b[i] < min)
      min = b[i];
    else if (b[i] > max)
      max = b[i];
  }  // end-for

  scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    bImg[i] = (unsigned char)((b[i] - min) * scale);
  }

  //  printf("b: min: %4.2lf, max: %4.2lf\n", min, max);

  delete L;
  delete a;
  delete b;
}  // end-MyRGB2Lab

///------------------------------------------------------------------------------------------
/// Fast version of MyRGB2Lab conversion
///
void MyRGB2LabFast(unsigned char *redImg, unsigned char *greenImg,
                   unsigned char *blueImg, unsigned char *LImg,
                   unsigned char *aImg, unsigned char *bImg, int width,
                   int height) {
  // First RGB 2 XYZ
  double red, green, blue;
  double x, y, z;

  // Timer timer;

  // timer.Start();

  for (int i = 0; i < width * height; i++) {
    red = redImg[i] / 255.0;
    green = greenImg[i] / 255.0;
    blue = blueImg[i] / 255.0;

#if 1
    if (red > 0.04045)
      red = pow(((red + 0.055) / 1.055), 2.4);
    else
      red = red / 12.92;

    if (green > 0.04045)
      green = pow(((green + 0.055) / 1.055), 2.4);
    else
      green = green / 12.92;

    if (blue > 0.04045)
      blue = pow(((blue + 0.055) / 1.055), 2.4);
    else
      blue = blue / 12.92;
#endif

    red = red * 100;
    green = green * 100;
    blue = blue * 100;

    // Observer. = 2�, Illuminant = D65
    x = red * 0.4124564 + green * 0.3575761 + blue * 0.1804375;
    y = red * 0.2126729 + green * 0.7151522 + blue * 0.0721750;
    z = red * 0.0193339 + green * 0.1191920 + blue * 0.9503041;

    // Now xyz 2 Lab
    double refX = 95.047;
    double refY = 100.000;
    double refZ = 108.883;

    x = x / refX;  // ref_X =  95.047   Observer= 2�, Illuminant= D65
    y = y / refY;  // ref_Y = 100.000
    z = z / refZ;  // ref_Z = 108.883

    if (x > 0.008856)
      x = pow(x, 1.0 / 3.0);
    else
      x = (7.787 * x) + (16.0 / 116.0);

    if (y > 0.008856)
      y = pow(y, 1.0 / 3.0);
    else
      y = (7.787 * y) + (16.0 / 116.0);

    if (z > 0.008856)
      z = pow(z, 1.0 / 3.0);
    else
      z = (7.787 * z) + (16.0 / 116.0);

    LImg[i] = (unsigned char)(2.55 * ((116.0 * y) - 16));
    aImg[i] =
        (unsigned char)(500 * (x - y) + 128);  // This is the correct formula
    bImg[i] = (unsigned char)(200 * (y - z) + 128);
  }  // end-for

  // timer.Stop();
  // printf("Loop takes: %4.2lf\n", timer.ElapsedTime());
}  // end-MyRGB2LabFast

///-----------------------------------------------------------------------------------
/// Standard Std2Lab conversion: No scaling after conversion
///
void StdRGB2Lab(unsigned char *redImg, unsigned char *greenImg,
                unsigned char *blueImg, unsigned char *LImg,
                unsigned char *aImg, unsigned char *bImg, int width,
                int height) {
  // First RGB 2 XYZ
  double red, green, blue;
  double x, y, z;

  double *L = new double[width * height];
  double *a = new double[width * height];
  double *b = new double[width * height];

  for (int i = 0; i < width * height; i++) {
    red = redImg[i] / 255.0;
    green = greenImg[i] / 255.0;
    blue = blueImg[i] / 255.0;

    if (red > 0.04045)
      red = pow(((red + 0.055) / 1.055), 2.4);
    else
      red = red / 12.92;

    if (green > 0.04045)
      green = pow(((green + 0.055) / 1.055), 2.4);
    else
      green = green / 12.92;

    if (blue > 0.04045)
      blue = pow(((blue + 0.055) / 1.055), 2.4);
    else
      blue = blue / 12.92;

    red = red * 100;
    green = green * 100;
    blue = blue * 100;

    // Observer. = 2�, Illuminant = D65
    x = red * 0.4124564 + green * 0.3575761 + blue * 0.1804375;
    y = red * 0.2126729 + green * 0.7151522 + blue * 0.0721750;
    z = red * 0.0193339 + green * 0.1191920 + blue * 0.9503041;

    // Now xyz 2 Lab
    double refX = 95.047;
    double refY = 100.000;
    double refZ = 108.883;

    x = x / refX;  // ref_X =  95.047   Observer= 2�, Illuminant= D65
    y = y / refY;  // ref_Y = 100.000
    z = z / refZ;  // ref_Z = 108.883

    if (x > 0.008856)
      x = pow(x, 1.0 / 3.0);
    else
      x = (7.787 * x) + (16.0 / 116.0);

    if (y > 0.008856)
      y = pow(y, 1.0 / 3.0);
    else
      y = (7.787 * y) + (16.0 / 116.0);

    if (z > 0.008856)
      z = pow(z, 1.0 / 3.0);
    else
      z = (7.787 * z) + (16.0 / 116.0);

    L[i] = (116.0 * y) - 16;
    a[i] = 500 * (x - y);  // This is the correct formula
    b[i] = 200 * (y - z);
  }  // end-for

  // Scale L to [0-255]
  double min = 1e10;
  double max = -1e10;
  for (int i = 0; i < width * height; i++) {
    if (L[i] < min)
      min = L[i];
    else if (L[i] > max)
      max = L[i];
  }  // end-for

  double scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    LImg[i] = (unsigned char)((L[i] - min) * scale);
  }

  // Scale a to [0-255]
  min = 1e10;
  max = -1e10;
  for (int i = 0; i < width * height; i++) {
    if (a[i] < min)
      min = a[i];
    else if (a[i] > max)
      max = a[i];
  }  // end-for

  scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    aImg[i] = (unsigned char)((a[i] - min) * scale);
  }

  // Scale b to [0-255]
  min = 1e10;
  max = -1e10;
  for (int i = 0; i < width * height; i++) {
    if (b[i] < min)
      min = b[i];
    else if (b[i] > max)
      max = b[i];
  }  // end-for

  scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    bImg[i] = (unsigned char)((b[i] - min) * scale);
  }

  delete L;
  delete a;
  delete b;
}  // end-StdRGB2Lab

///-------------------------------------------------------------------------------------
/// My formulation on an IplImage
///
void RGB2Lab(IplImage *rgbImg, IplImage *labImg) {
  // First RGB 2 XYZ
  double red, green, blue;
  double x, y, z;

  int width = rgbImg->width;
  int height = rgbImg->height;

  for (int i = 0; i < height; i++) {
    unsigned char *rgbLine =
        (unsigned char *)(rgbImg->imageData + i * rgbImg->widthStep);
    unsigned char *labLine =
        (unsigned char *)(labImg->imageData + i * labImg->widthStep);

    for (int j = 0; j < width; j++) {
      red = rgbLine[2] / 255.0;
      green = rgbLine[1] / 255.0;
      blue = rgbLine[0] / 255.0;

      if (red > 0.04045)
        red = pow(((red + 0.055) / 1.055), 2.4);
      else
        red = red / 12.92;

      if (green > 0.04045)
        green = pow(((green + 0.055) / 1.055), 2.4);
      else
        green = green / 12.92;

      if (blue > 0.04045)
        blue = pow(((blue + 0.055) / 1.055), 2.4);
      else
        blue = blue / 12.92;

      red = red * 100;
      green = green * 100;
      blue = blue * 100;

      // Observer. = 2�, Illuminant = D65
      x = red * 0.4124 + green * 0.3576 + blue * 0.1805;
      y = red * 0.2126 + green * 0.7152 + blue * 0.0722;
      z = red * 0.0193 + green * 0.1192 + blue * 0.9505;

      // Now xyz 2 Lab
      double refX = 95.047;
      double refY = 100.000;
      double refZ = 108.883;

      x = x / refX;  // ref_X =  95.047   Observer= 2�, Illuminant= D65
      y = y / refY;  // ref_Y = 100.000
      z = z / refZ;  // ref_Z = 108.883

      if (x > 0.008856)
        x = pow(x, 1.0 / 3.0);
      else
        x = (7.787 * x) + (16.0 / 116.0);

      if (y > 0.008856)
        y = pow(y, 1.0 / 3.0);
      else
        y = (7.787 * y) + (16.0 / 116.0);

      if (z > 0.008856)
        z = pow(z, 1.0 / 3.0);
      else
        z = (7.787 * z) + (16.0 / 116.0);

      labLine[0] = (unsigned char)(2.55 * ((116.0 * y) - 16));
      labLine[1] = (unsigned char)(128 + 500 * (x - y));
      labLine[2] = (unsigned char)(128 + 200 * (y - z));

      rgbLine += 3;
      labLine += 3;
    }  // end-for
  }    // end-for
}  // end-RGB2Lab

///-------------------------------------------------------------------------------------
/// Another conversion function based on OpenCV's formulation
///
void RGB2Lab2(unsigned char *redImg, unsigned char *greenImg,
              unsigned char *blueImg, unsigned char *LImg, unsigned char *aImg,
              unsigned char *bImg, int width, int height) {
  // First RGB 2 XYZ
  double X, Y, Z, fX, fY, fZ;
  double L, a, b;

#define BLACK 20.0
#define YELLOW 70.0

  for (int i = 0; i < width * height; i++) {
    // OpenCV RGB2Lab formulation
    X = 0.412453 * redImg[i] + 0.357580 * greenImg[i] + 0.180423 * blueImg[i];
    Y = 0.212671 * redImg[i] + 0.715160 * greenImg[i] + 0.072169 * blueImg[i];
    Z = 0.019334 * redImg[i] + 0.119193 * greenImg[i] + 0.950227 * blueImg[i];

    X /= (255.0 * 0.950456);
    Y /= 255.0;
    Z /= (255.0 * 1.088754);

    if (Y > 0.008856) {
      fY = pow(Y, 1.0 / 3.0);
      L = 116.0 * fY - 16.0;

    } else {
      fY = 7.787 * Y + 16.0 / 116.0;
      L = 903.3 * Y;
    }  // end-else

    if (X > 0.008856)
      fX = pow(X, 1.0 / 3.0);
    else
      fX = 7.787 * X + 16.0 / 116.0;

    if (Z > 0.008856)
      fZ = pow(Z, 1.0 / 3.0);
    else
      fZ = 7.787 * Z + 16.0 / 116.0;

    a = 500.0 * (fX - fY);
    b = 200.0 * (fY - fZ);

    if (L < BLACK) {
      a *= exp((L - BLACK) / (BLACK / 4));
      b *= exp((L - BLACK) / (BLACK / 4));
      L = BLACK;
    }  // end-if

    if (b > YELLOW) b = YELLOW;

    if (2.55 * L > 255) printf("L-- overflow\n");
    if (a + 128 > 255) printf("a-- overflow\n");
    if (b + 128 > 255) printf("b-- overflow\n");

    LImg[i] = (unsigned char)L;  // Is this correct?
                                 //    LImg[i] = (unsigned char)(2.55*L);
    aImg[i] = (unsigned char)(a + 128);
    bImg[i] = (unsigned char)(b + 128);
  }  // end-for

#if 0
  // Scale L
  unsigned char min=255;
  unsigned char max=0;
  for (int i=0; i<width*height; i++){
    if      (LImg[i]<min) min = LImg[i];
    else if (LImg[i]>max) max = LImg[i];
  } //end-for

  double scale = 255.0/(max-min);  
  for (int i=0; i<width*height; i++){LImg[i] = (unsigned char)((LImg[i]-min)*scale);}
//  printf("min: %4.2lf, max: %4.2lf\n", min/2.55, max/2.55);

  // Scale a
  min=255;
  max=0;
  for (int i=0; i<width*height; i++){
    if      (aImg[i]<min) min = aImg[i];
    else if (aImg[i]>max) max = aImg[i];
  } //end-for

  scale = 255.0/(max-min);  
  for (int i=0; i<width*height; i++){aImg[i] = (unsigned char)((aImg[i]-min)*scale);}
//  printf("min: %d, max: %d\n", min-128, max-128);

  // Scale b
  min=255;
  max=0;
  for (int i=0; i<width*height; i++){
    if      (bImg[i]<min) min = bImg[i];
    else if (bImg[i]>max) max = bImg[i];
  } //end-for

  scale = 255.0/(max-min);  
  for (int i=0; i<width*height; i++){bImg[i] = (unsigned char)((bImg[i]-min)*scale);}
//  printf("min: %d, max: %d\n", min-128, max-128);
#endif
}  // end-RGB2Lab2

///------------------------------------------------------------------------------------
/// Standard way of converting RGB-->Lab: No scaling of Lab after conversion
///
#define BLACK 20.0
#define YELLOW 70.0

void RGB2LabOne(float R, float G, float B, float *L, float *a, float *b) {
  float X, Y, Z, fX, fY, fZ;

  // Burak - added explicit typecasts
  X = (float)(0.412453 * R + 0.357580 * G + 0.180423 * B);
  Y = (float)(0.212671 * R + 0.715160 * G + 0.072169 * B);
  Z = (float)(0.019334 * R + 0.119193 * G + 0.950227 * B);

  X /= (float)(255 * 0.950456);
  Y /= 255;
  Z /= (float)(255 * 1.088754);

  if (Y > 0.008856) {
    fY = pow(Y, 1.0f / 3.0f);
    *L = (float)(116.0 * fY - 16.0);
  } else {
    fY = (float)(7.787 * Y + 16.0 / 116.0);
    *L = (float)(903.3 * Y);
  }

  if (X > 0.008856)
    fX = pow(X, 1.0f / 3.0f);
  else
    fX = (float)(7.787 * X + 16.0 / 116.0);

  if (Z > 0.008856)
    fZ = pow(Z, 1.0f / 3.0f);
  else
    fZ = (float)(7.787 * Z + 16.0 / 116.0);

  *a = (float)(500.0 * (fX - fY));
  *b = (float)(200.0 * (fY - fZ));

  if (*L < BLACK) {
    *a *= (float)(exp((*L - BLACK) / (BLACK / 4)));
    *b *= (float)(exp((*L - BLACK) / (BLACK / 4)));
    *L = BLACK;
  }
  if (*b > YELLOW) *b = YELLOW;

  /*printf("RGB=(%d,%d,%d) ==> Lab(%d,%d,%d)\n",(int)R,(int)G,(int)B,
    (int)*L,(int)*a,(int)*b); */
}
#if 0
void StdRGB2Lab(unsigned char *redImg, unsigned char *greenImg, unsigned char *blueImg,
                  unsigned char *LImg, unsigned char *aImg, unsigned char *bImg,
                  int width, int height){

  for (int i=0; i<height; i++){
    for (int j=0; j<width; j++){
      float R = redImg[i*width+j];
      float G = greenImg[i*width+j];
      float B = blueImg[i*width+j];

      float L, a, b;
      RGB2LabOne(R, G, B, &L, &a, &b);

      LImg[i*width+j] = (unsigned char)L;
      aImg[i*width+j] = (unsigned char)(a + 128);
      bImg[i*width+j] = (unsigned char)(b + 128);
    } //end-for 
  } //end-for
} //end-StdRGB2Lab
#endif

void RGB2LabOpenCV(unsigned char *redImg, unsigned char *greenImg,
                   unsigned char *blueImg, unsigned char *LImg,
                   unsigned char *aImg, unsigned char *bImg, int width,
                   int height) {
  // Convert BGR to Lab using OpenCV
  IplImage *iplImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
  IplImage *iplLabImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

  int index = 0;
  for (int i = 0; i < height; i++) {
    unsigned char *p =
        (unsigned char *)(iplImg->imageData + i * iplImg->widthStep);

    for (int j = 0; j < width; j++, p += 3) {
      p[0] = blueImg[index];
      p[1] = greenImg[index];
      p[2] = redImg[index];
      index++;
    }  // end-for
  }    // end-for

  // Convert from BGR2Lab
  cvCvtColor(iplImg, iplLabImg, CV_BGR2Lab);

  index = 0;
  for (int i = 0; i < height; i++) {
    unsigned char *p =
        (unsigned char *)(iplLabImg->imageData + i * iplLabImg->widthStep);

    for (int j = 0; j < width; j++, p += 3) {
      LImg[index] = p[0];
      aImg[index] = p[1];
      bImg[index] = p[2];
      index++;
    }  // end-for
  }    // end-for

  cvReleaseImage(&iplImg);
  cvReleaseImage(&iplLabImg);

#if 0
  // Scale L
  unsigned char min=255;
  unsigned char max=0;
  for (int i=0; i<width*height; i++){
    if      (LImg[i]<min) min = LImg[i];
    else if (LImg[i]>max) max = LImg[i];
  } //end-for
  
//  printf("L: min: %d, max: %d\n", (int)(min/2.55), (int)(max/2.55));     

  double scale = 255.0/(max-min);  
  for (int i=0; i<width*height; i++){LImg[i] = (unsigned char)((LImg[i]-min)*scale);}

  // Scale a
  min=255;
  max=0;
  for (int i=0; i<width*height; i++){
    if      (aImg[i]<min) min = aImg[i];
    else if (aImg[i]>max) max = aImg[i];
  } //end-for

//  printf("a: min: %d, max: %d\n", min, max);

  scale = 255.0/(max-min);  
  for (int i=0; i<width*height; i++){aImg[i] = (unsigned char)((aImg[i]-min)*scale);}

  // Scale b
  min=255;
  max=0;
  for (int i=0; i<width*height; i++){
    if      (bImg[i]<min) min = bImg[i];
    else if (bImg[i]>max) max = bImg[i];
  } //end-for

//  printf("b: min: %d, max: %d\n", min, max);

  scale = 255.0/(max-min);  
  for (int i=0; i<width*height; i++){bImg[i] = (unsigned char)((bImg[i]-min)*scale);}
#endif
}  // end-RGB2LabOpenCV

/*******************************************************************************
var_U = ( 4 * X ) / ( X + ( 15 * Y ) + ( 3 * Z ) )
var_V = ( 9 * Y ) / ( X + ( 15 * Y ) + ( 3 * Z ) )

var_Y = Y / 100
if ( var_Y > 0.008856 ) var_Y = var_Y ^ ( 1/3 )
else                    var_Y = ( 7.787 * var_Y ) + ( 16 / 116 )

ref_X =  95.047        //Observer= 2�, Illuminant= D65
ref_Y = 100.000
ref_Z = 108.883

ref_U = ( 4 * ref_X ) / ( ref_X + ( 15 * ref_Y ) + ( 3 * ref_Z ) )
ref_V = ( 9 * ref_Y ) / ( ref_X + ( 15 * ref_Y ) + ( 3 * ref_Z ) )

CIE-L* = ( 116 * var_Y ) - 16
CIE-u* = 13 * CIE-L* * ( var_U - ref_U )
CIE-v* = 13 * CIE-L* * ( var_V - ref_V )
*/
void RGB2Luv(unsigned char *redImg, unsigned char *greenImg,
             unsigned char *blueImg, unsigned char *LImg, unsigned char *uImg,
             unsigned char *vImg, int width, int height) {
  // First RGB 2 XYZ
  double red, green, blue;

  double *L = new double[width * height];
  double *u = new double[width * height];
  double *v = new double[width * height];

  for (int i = 0; i < width * height; i++) {
    red = redImg[i] / 255.0;
    green = greenImg[i] / 255.0;
    blue = blueImg[i] / 255.0;

    if (red > 0.04045)
      red = pow(((red + 0.055) / 1.055), 2.4);
    else
      red = red / 12.92;

    if (green > 0.04045)
      green = pow(((green + 0.055) / 1.055), 2.4);
    else
      green = green / 12.92;

    if (blue > 0.04045)
      blue = pow(((blue + 0.055) / 1.055), 2.4);
    else
      blue = blue / 12.92;

    red = red * 100;
    green = green * 100;
    blue = blue * 100;

    // Observer. = 2�, Illuminant = D65
    double X = red * 0.4124 + green * 0.3576 + blue * 0.1805;
    double Y = red * 0.2126 + green * 0.7152 + blue * 0.0722;
    double Z = red * 0.0193 + green * 0.1192 + blue * 0.9505;

    // Now xyz 2 Luv
    double var_U = (4 * X) / (X + (15 * Y) + (3 * Z));
    double var_V = (9 * Y) / (X + (15 * Y) + (3 * Z));

    double var_Y = Y / 100.0;
    if (var_Y > 0.008856)
      var_Y = pow(var_Y, 1.0 / 3);
    else
      var_Y = (7.787 * var_Y) + (16.0 / 116.0);

    double ref_X = 95.047;  // Observer= 2�, Illuminant= D65
    double ref_Y = 100.000;
    double ref_Z = 108.883;

    double ref_U = (4 * ref_X) / (ref_X + (15 * ref_Y) + (3 * ref_Z));
    double ref_V = (9 * ref_Y) / (ref_X + (15 * ref_Y) + (3 * ref_Z));

    L[i] = (116 * var_Y) - 16;
    u[i] = 13 * L[i] * (var_U - ref_U);
    v[i] = 13 * L[i] * (var_V - ref_V);
  }  // end-for

  // Scale L to [0-255]
  double min = 1e10;
  double max = -1e10;
  for (int i = 0; i < width * height; i++) {
    if (L[i] < min)
      min = L[i];
    else if (L[i] > max)
      max = L[i];
  }  // end-for

  double scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    LImg[i] = (unsigned char)((L[i] - min) * scale);
  }

  // Scale u to [0-255]
  min = 1e10;
  max = -1e10;
  for (int i = 0; i < width * height; i++) {
    if (u[i] < min)
      min = u[i];
    else if (u[i] > max)
      max = u[i];
  }  // end-for

  scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    uImg[i] = (unsigned char)((u[i] - min) * scale);
  }

  // Scale v to [0-255]
  min = 1e10;
  max = -1e10;
  for (int i = 0; i < width * height; i++) {
    if (v[i] < min)
      min = v[i];
    else if (v[i] > max)
      max = v[i];
  }  // end-for

  scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    vImg[i] = (unsigned char)((v[i] - min) * scale);
  }

  delete L;
  delete u;
  delete v;
}  // end-RGB2Luv

/*------------------ RGB 2 HSL ----------------------------------
var_R = ( R / 255 )                     //RGB from 0 to 255
var_G = ( G / 255 )
var_B = ( B / 255 )

var_Min = min( var_R, var_G, var_B )    //Min. value of RGB
var_Max = max( var_R, var_G, var_B )    //Max. value of RGB
del_Max = var_Max - var_Min             //Delta RGB value

L = ( var_Max + var_Min ) / 2

if ( del_Max == 0 )                     //This is a gray, no chroma...
{
   H = 0                                //HSL results from 0 to 1
   S = 0
}
else                                    //Chromatic data...
{
   if ( L < 0.5 ) S = del_Max / ( var_Max + var_Min )
   else           S = del_Max / ( 2 - var_Max - var_Min )

   del_R = ( ( ( var_Max - var_R ) / 6 ) + ( del_Max / 2 ) ) / del_Max
   del_G = ( ( ( var_Max - var_G ) / 6 ) + ( del_Max / 2 ) ) / del_Max
   del_B = ( ( ( var_Max - var_B ) / 6 ) + ( del_Max / 2 ) ) / del_Max

   if      ( var_R == var_Max ) H = del_B - del_G
   else if ( var_G == var_Max ) H = ( 1 / 3 ) + del_R - del_B
   else if ( var_B == var_Max ) H = ( 2 / 3 ) + del_G - del_R

   if ( H < 0 ) H += 1
   if ( H > 1 ) H -= 1
}
*/
void RGB2HSL(unsigned char *redImg, unsigned char *greenImg,
             unsigned char *blueImg, unsigned char *HImg, unsigned char *SImg,
             unsigned char *LImg, int width, int height) {
  // First RGB 2 XYZ
  double red, green, blue;

  double *H = new double[width * height];
  double *S = new double[width * height];
  double *L = new double[width * height];

  double min, max;

  for (int i = 0; i < width * height; i++) {
    red = redImg[i] / 255.0;
    green = greenImg[i] / 255.0;
    blue = blueImg[i] / 255.0;

    min = red;
    if (green < min) min = green;
    if (blue < min) min = blue;

    max = red;
    if (green > max) max = green;
    if (blue > max) max = blue;

    double deltaMax = max - min;
    L[i] = (max + min) / 2.0;  // delta RGB value

    if (deltaMax < 1e-10) {  // This is a gray, no chroma...
      H[i] = 0;
      S[i] = 0;  // HSL results from 0 to 1

    } else {
      // Chromatic data...
      if (L[i] < 0.5)
        S[i] = deltaMax / (max + min);
      else
        S[i] = deltaMax / (2 - max - min);

      double deltaR = (((max - red) / 6) + (deltaMax / 2)) / deltaMax;
      double deltaG = (((max - green) / 6) + (deltaMax / 2)) / deltaMax;
      double deltaB = (((max - blue) / 6) + (deltaMax / 2)) / deltaMax;

      if (fabs(red - max) <= 1e-10)
        H[i] = deltaB - deltaG;
      else if (fabs(green - max) <= 1e-10)
        H[i] = (1.0 / 3.0) + deltaR - deltaB;
      else
        H[i] = (2.0 / 3.0) + deltaG - deltaR;

      if (H[i] < 0) H[i] += 1.0;
      if (H[i] > 1) H[i] -= 1.0;
    }  // end-else
  }    // end-for

  // Scale H to [0-255]
  min = 1e10;
  max = -1e10;
  for (int i = 0; i < width * height; i++) {
    if (H[i] < min)
      min = H[i];
    else if (H[i] > max)
      max = H[i];
  }  // end-for

  double scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    HImg[i] = (unsigned char)((H[i] - min) * scale);
  }

  // Scale S to [0-255]
  min = 1e10;
  max = -1e10;
  for (int i = 0; i < width * height; i++) {
    if (S[i] < min)
      min = S[i];
    else if (S[i] > max)
      max = S[i];
  }  // end-for

  scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    SImg[i] = (unsigned char)((S[i] - min) * scale);
  }

  // Scale L to [0-255]
  min = 1e10;
  max = -1e10;
  for (int i = 0; i < width * height; i++) {
    if (L[i] < min)
      min = L[i];
    else if (L[i] > max)
      max = L[i];
  }  // end-for

  scale = 255.0 / (max - min);
  for (int i = 0; i < width * height; i++) {
    LImg[i] = (unsigned char)((L[i] - min) * scale);
  }

  delete H;
  delete S;
  delete L;
}  // end-RGB2HSL

void RGB2YUV(unsigned char *redImg, unsigned char *greenImg,
             unsigned char *blueImg, unsigned char *YImg, unsigned char *UImg,
             unsigned char *VImg, int width, int height) {
  for (int i = 0; i < width * height; i++) {
    YImg[i] = (unsigned char)(0.299 * redImg[i] + 0.587 * greenImg[i] +
                              0.114 * blueImg[i]);
    UImg[i] = (unsigned char)(-0.14713 * redImg[i] - 0.28886 * greenImg[i] +
                              0.436 * blueImg[i]);
    VImg[i] = (unsigned char)(0.615 * redImg[i] - 0.51499 * greenImg[i] -
                              0.10001 * blueImg[i]);
  }  // end-for
}  // end-RGB2YUV

///-------------------------------------------------------------------------------------------
/// Dumps the gradient image: Scaled to [0-255]
///
void DumpGradImage(char *file, short *gradImg, int width, int height) {
  unsigned char *out = new unsigned char[width * height];

  int max = 0;
  for (int i = 0; i < width * height; i++) {
    if (gradImg[i] > max) max = gradImg[i];
  }  // end-for

  double scale = 255.0 / max;

  for (int i = 0; i < width * height; i++) {
    out[i] = (unsigned char)(scale * gradImg[i]);
  }  // end-for

  // Burak - commented the line below, it was making a fuss
  // SaveImage(file, (char *)out, width, height, 8);
  delete out;
}  // end-DumpGradImg

///-------------------------------------------------------------------------------------------
/// Dumps the gradient image
///
void DumpGradImage(char *file, short *gradImg, int width, int height,
                   int thresh) {
  unsigned char *out = new unsigned char[width * height];

  for (int i = 0; i < width * height; i++) {
    if (gradImg[i] >= thresh)
      out[i] = 255;
    else
      out[i] = 0;
  }  // end-for

  // Burak - commented the line below, it was making a fuss
  // SaveImage(file, (char *)out, width, height, 8);
  delete out;
}  // end-DumpGradImg

///-------------------------------------------------------------------------------------------
/// Dumps the edge segments to a file
///
void DumpEdgeSegments(char *file, EdgeMap *map) {
  int width = map->width;
  int height = map->height;

  unsigned char *edgeImg = new unsigned char[width * height];
  memset(edgeImg, 0, width * height);

  for (int i = 0; i < map->noSegments; i++) {
    for (int j = 0; j < map->segments[i].noPixels; j++) {
      int r = map->segments[i].pixels[j].r;
      int c = map->segments[i].pixels[j].c;

      edgeImg[r * width + c] = 255;
    }  // end-for
  }    // end-for

  // Burak - commented the line below, it was making a fuss
  // SaveImage(file, (char *)edgeImg, width, height, 8);
  delete edgeImg;
}  // end-DumpEdgeSegments

///-------------------------------------------------------------------------------------------
/// ColorEdgseSegments
///
void ColorEdgeSegments(EdgeMap *map, unsigned char *colorImg,
                       unsigned char *srcImg) {
  int width = map->width;
  int height = map->height;

  if (srcImg == NULL) memset(colorImg, 0, width * height * 3);
  //    memset(colorImg, 255, width*height*3);

  else {
    for (int i = 0; i < width * height; i++) {
      colorImg[i * 3] = srcImg[i];
      colorImg[i * 3 + 1] = srcImg[i];
      colorImg[i * 3 + 2] = srcImg[i];
    }  // end-for
  }    // end-for

  ColorGenerator cg;
  int red, blue, green;

  for (int i = 0; i < map->noSegments; i++) {
    cg.getNextColor(&red, &green, &blue);

    if (red == 255 && green == 0 && blue == 0)
      cg.getNextColor(&red, &green, &blue);

    for (int j = 0; j < map->segments[i].noPixels; j++) {
      int r = map->segments[i].pixels[j].r;
      int c = map->segments[i].pixels[j].c;

      if (r < 0 || r >= height) continue;
      if (c < 0 || c >= width) continue;

      colorImg[(r * width + c) * 3] = blue;
      colorImg[(r * width + c) * 3 + 1] = green;
      colorImg[(r * width + c) * 3 + 2] = red;
    }  // end-for
  }    // end-for
}  // end-ColorEdgeSegments

///-------------------------------------------------------------------------------------------
/// ColorEdgseSegments
///
void ColorEdgeSegments(char *file, EdgeMap *map, unsigned char *srcImg) {
  int width = map->width;
  int height = map->height;

  unsigned char *colorImg = new unsigned char[width * height * 3];

  ColorEdgeSegments(map, colorImg, srcImg);

  // Burak - commented the line below, it was making a fuss
  // SaveImage(file, (char *)colorImg, width, height, 24);
  delete colorImg;
}  // end-ColorEdgeSegments

///-------------------------------------------------------------------------------------------
/// ShowJointPoints
///
void ShowJointPoints(char *file, EdgeMap *map, unsigned char *jointPoints,
                     unsigned char *srcImg) {
  int width = map->width;
  int height = map->height;

  unsigned char *colorImg = new unsigned char[width * height * 3];

  if (srcImg == NULL)
    memset(colorImg, 0, width * height * 3);

  else {
    for (int i = 0; i < width * height; i++) {
      colorImg[i * 3] = srcImg[i];
      colorImg[i * 3 + 1] = srcImg[i];
      colorImg[i * 3 + 2] = srcImg[i];
    }  // end-for
  }    // end-for

  ColorGenerator cg;
  int red, blue, green;

  for (int i = 0; i < map->noSegments; i++) {
    cg.getNextColor(&red, &green, &blue);
    if (red == 255 && green == 0 && blue == 0)
      cg.getNextColor(&red, &green, &blue);

    for (int j = 0; j < map->segments[i].noPixels; j++) {
      int r = map->segments[i].pixels[j].r;
      int c = map->segments[i].pixels[j].c;

      colorImg[(r * width + c) * 3] = blue;
      colorImg[(r * width + c) * 3 + 1] = green;
      colorImg[(r * width + c) * 3 + 2] = red;
    }  // end-for
  }    // end-for

  for (int i = 0; i < width * height; i++) {
    if (jointPoints[i] == 0) continue;

    colorImg[i * 3] = 0;
    colorImg[i * 3 + 1] = 0;
    colorImg[i * 3 + 2] = 255;
  }  // end-for

  // Burak - commented the line below, it was making a fuss
  // SaveImage(file, (char *)colorImg, width, height, 24);
  delete colorImg;
}  // end-ShowJointPoints

///-------------------------------------------------------------------
/// Get all files in a directory in an array
///
int GetFilenamesInDirectory(char *dirname, DirectoryEntry *items) {
  /*
  BOOL            fFinished;
  HANDLE          hList;
  TCHAR           szDir[MAX_PATH+1];
  WIN32_FIND_DATA FileData;
  char            filename[100];

  // Get the proper directory path
  //Burak - added (char*) typecast below line
  sprintf((char*)szDir, "%s\\*", dirname);

  // Get the first file
  hList = FindFirstFile(szDir, &FileData);
  if (hList == INVALID_HANDLE_VALUE) return 0;

  int noItems = 0;

  // Traverse through the directory structure
  fFinished = FALSE;
  while (!fFinished){
    SYSTEMTIME sysTime;
    FileTimeToSystemTime(&FileData.ftCreationTime, &sysTime);

    if ((FileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == false){
      // Only take files
      sprintf(filename, "%s%s", dirname, FileData.cFileName);

      // Insert into items in sorted order
      int index = noItems-1;
      while (index >= 0){
        if (strcmp(items[index].filename, filename) < 0) break;
        items[index+1] = items[index];
        index--;
      } //end-while
    

      index++;
      strcpy(items[index].filename, filename);
      noItems++;
    } //end-if

    if (!FindNextFile(hList, &FileData)){
        if (GetLastError() == ERROR_NO_MORE_FILES){
            fFinished = TRUE;
        } //end-if
    } //end-if
  } //end-while

  FindClose(hList);

  return noItems;
  */
  return 0;
}  // end-GetFilenamesInDirectory

///-----------------------------------------------------------------------------
/// Scales a given image
///
typedef struct image_double_s {
  double *data;
  unsigned int xsize, ysize;
} * image_double;

typedef struct ntuple_list_s {
  unsigned int size;
  unsigned int max_size;
  unsigned int dim;
  double *values;
} * ntuple_list;

// static void error(char *msg) {
static void error(std::string msg) {
  fprintf(stderr, "gaussian_sampler error: %s\n", msg.c_str());
  exit(EXIT_FAILURE);
}

/*----------------------------------------------------------------------------*/
/** Free memory used in image_double 'i'.
 */
void free_image_double(image_double i) {
  if (i == NULL || i->data == NULL)
    error("free_image_double: invalid input image.");
  free((void *)i->data);
  free((void *)i);
}

/*----------------------------------------------------------------------------*/
/** Create a new image_double of size 'xsize' times 'ysize'.
 */
image_double new_image_double(unsigned int xsize, unsigned int ysize) {
  image_double image;

  /* check parameters */
  if (xsize == 0 || ysize == 0) error("new_image_double: invalid image size.");

  /* get memory */
  image = (image_double)malloc(sizeof(struct image_double_s));
  if (image == NULL) error("not enough memory.");
  image->data = (double *)calloc((size_t)(xsize * ysize), sizeof(double));
  if (image->data == NULL) error("not enough memory.");

  /* set image size */
  image->xsize = xsize;
  image->ysize = ysize;

  return image;
}

/*----------------------------------------------------------------------------*/
/** Free memory used in n-tuple 'in'.
 */
void free_ntuple_list(ntuple_list in) {
  if (in == NULL || in->values == NULL)
    error("free_ntuple_list: invalid n-tuple input.");
  free((void *)in->values);
  free((void *)in);
}

/*----------------------------------------------------------------------------*/
/** Create an n-tuple list and allocate memory for one element.
    @param dim the dimension (n) of the n-tuple.
 */
ntuple_list new_ntuple_list(unsigned int dim) {
  ntuple_list n_tuple;

  /* check parameters */
  if (dim == 0) error("new_ntuple_list: 'dim' must be positive.");

  /* get memory for list structure */
  n_tuple = (ntuple_list)malloc(sizeof(struct ntuple_list_s));
  if (n_tuple == NULL) error("not enough memory.");

  /* initialize list */
  n_tuple->size = 0;
  n_tuple->max_size = 1;
  n_tuple->dim = dim;

  /* get memory for tuples */
  n_tuple->values = (double *)malloc(dim * n_tuple->max_size * sizeof(double));
  if (n_tuple->values == NULL) error("not enough memory.");

  return n_tuple;
}

/*----------------------------------------------------------------------------*/
/** Enlarge the allocated memory of an n-tuple list.
 */
static void enlarge_ntuple_list(ntuple_list n_tuple) {
  /* check parameters */
  if (n_tuple == NULL || n_tuple->values == NULL || n_tuple->max_size == 0)
    error("enlarge_ntuple_list: invalid n-tuple.");

  /* duplicate number of tuples */
  n_tuple->max_size *= 2;

  /* realloc memory */
  n_tuple->values =
      (double *)realloc((void *)n_tuple->values,
                        n_tuple->dim * n_tuple->max_size * sizeof(double));
  if (n_tuple->values == NULL) error("not enough memory.");
}

static void gaussian_kernel(ntuple_list kernel, double sigma, double mean) {
  double sum = 0.0;
  double val;
  unsigned int i;

  /* check parameters */
  if (kernel == NULL || kernel->values == NULL)
    error("gaussian_kernel: invalid n-tuple 'kernel'.");
  if (sigma <= 0.0) error("gaussian_kernel: 'sigma' must be positive.");

  /* compute Gaussian kernel */
  if (kernel->max_size < 1) enlarge_ntuple_list(kernel);
  kernel->size = 1;
  for (i = 0; i < kernel->dim; i++) {
    val = ((double)i - mean) / sigma;
    kernel->values[i] = exp(-0.5 * val * val);
    sum += kernel->values[i];
  }

  /* normalization */
  if (sum >= 0.0)
    for (i = 0; i < kernel->dim; i++) kernel->values[i] /= sum;
}

static image_double gaussian_sampler(image_double in, double scale,
                                     double sigma_scale) {
  image_double aux, out;
  ntuple_list kernel;
  unsigned int N, M, h, n, x, y, i;
  int xc, yc, j, double_x_size, double_y_size;
  double sigma, xx, yy, sum, prec;

  /* check parameters */
  if (in == NULL || in->data == NULL || in->xsize == 0 || in->ysize == 0)
    error("gaussian_sampler: invalid image.");
  if (scale <= 0.0) error("gaussian_sampler: 'scale' must be positive.");
  if (sigma_scale <= 0.0)
    error("gaussian_sampler: 'sigma_scale' must be positive.");

  /* get memory for images */
  if (in->xsize * scale > (double)UINT_MAX ||
      in->ysize * scale > (double)UINT_MAX)
    error("gaussian_sampler: the output image size exceeds the handled size.");
  N = (unsigned int)floor(in->xsize * scale);
  M = (unsigned int)floor(in->ysize * scale);
  aux = new_image_double(N, in->ysize);
  out = new_image_double(N, M);

  /* sigma, kernel size and memory for the kernel */
  sigma = scale < 1.0 ? sigma_scale / scale : sigma_scale;
  /*
     The size of the kernel is selected to guarantee that the
     the first discarded term is at least 10^prec times smaller
     than the central value. For that, h should be larger than x, with
       e^(-x^2/2sigma^2) = 1/10^prec.
     Then,
       x = sigma * sqrt( 2 * prec * ln(10) ).
   */
  prec = 3.0;
  h = (unsigned int)ceil(sigma * sqrt(2.0 * prec * log(10.0)));
  n = 1 + 2 * h; /* kernel size */
  kernel = new_ntuple_list(n);

  /* auxiliary double image size variables */
  double_x_size = (int)(2 * in->xsize);
  double_y_size = (int)(2 * in->ysize);

  /* First subsampling: x axis */
  for (x = 0; x < aux->xsize; x++) {
    /*
       x   is the coordinate in the new image.
       xx  is the corresponding x-value in the original size image.
       xc  is the integer value, the pixel coordinate of xx.
     */
    xx = (double)x / scale;
    /* coordinate (0.0,0.0) is in the center of pixel (0,0),
       so the pixel with xc=0 get the values of xx from -0.5 to 0.5 */
    xc = (int)floor(xx + 0.5);
    gaussian_kernel(kernel, sigma, (double)h + xx - (double)xc);
    /* the kernel must be computed for each x because the fine
       offset xx-xc is different in each case */

    for (y = 0; y < aux->ysize; y++) {
      sum = 0.0;
      for (i = 0; i < kernel->dim; i++) {
        j = xc - h + i;

        /* symmetry boundary condition */
        while (j < 0) j += double_x_size;
        while (j >= double_x_size) j -= double_x_size;
        if (j >= (int)in->xsize) j = double_x_size - 1 - j;

        sum += in->data[j + y * in->xsize] * kernel->values[i];
      }
      aux->data[x + y * aux->xsize] = sum;
    }
  }

  /* Second subsampling: y axis */
  for (y = 0; y < out->ysize; y++) {
    /*
       y   is the coordinate in the new image.
       yy  is the corresponding x-value in the original size image.
       yc  is the integer value, the pixel coordinate of xx.
     */
    yy = (double)y / scale;
    /* coordinate (0.0,0.0) is in the center of pixel (0,0),
       so the pixel with yc=0 get the values of yy from -0.5 to 0.5 */
    yc = (int)floor(yy + 0.5);
    gaussian_kernel(kernel, sigma, (double)h + yy - (double)yc);
    /* the kernel must be computed for each y because the fine
       offset yy-yc is different in each case */

    for (x = 0; x < out->xsize; x++) {
      sum = 0.0;
      for (i = 0; i < kernel->dim; i++) {
        j = yc - h + i;

        /* symmetry boundary condition */
        while (j < 0) j += double_y_size;
        while (j >= double_y_size) j -= double_y_size;
        if (j >= (int)in->ysize) j = double_y_size - 1 - j;

        sum += aux->data[x + j * aux->xsize] * kernel->values[i];
      }
      out->data[x + y * out->xsize] = sum;
    }
  }

  /* free memory */
  free_ntuple_list(kernel);
  free_image_double(aux);

  return out;
}

unsigned char *ScaleImage(unsigned char *srcImg, int width, int height,
                          double scale, int *pw, int *ph) {
  image_double doubleImg = new_image_double(width, height);

  for (int i = 0; i < width * height; i++)
    doubleImg->data[i] = (double)srcImg[i];

  image_double scaledImg = gaussian_sampler(doubleImg, scale, 0.6);
  int w = scaledImg->xsize;
  int h = scaledImg->ysize;

  unsigned char *outImg = new unsigned char[w * h];
  for (int i = 0; i < w * h; i++) {
    outImg[i] = (unsigned char)(scaledImg->data[i]);
  }  // end-for

  free_image_double(doubleImg);
  free_image_double(scaledImg);

  *pw = w;
  *ph = h;

  return outImg;
}  // end-ScaleImage

#if 0
///---------------------------------------------------------------------------------------------------
/// Std way of converting rgb-->Lab (whatever this means?)
///
void StdRGB2LabOne(unsigned char r, unsigned char g, unsigned char bl, double *L, double *a, double *b){
  double red, green, blue;
  double x, y, z;

  red = r/255.0;
  green = g/255.0;
  blue = bl/255.0;

  if (red>0.04045) red = pow(((red+0.055)/1.055), 2.4);
  else             red = red/12.92;

  if (green>0.04045) green = pow(((green+0.055)/1.055), 2.4);
  else               green = green/12.92;

  if (blue>0.04045) blue = pow(((blue+0.055)/1.055), 2.4);
  else              blue = blue/12.92;

  red = red*100;
  green = green*100;
  blue = blue*100;

  //Observer. = 2�, Illuminant = D65
  x = red*0.4124564 + green*0.3575761 + blue*0.1804375;
  y = red*0.2126729 + green*0.7151522 + blue*0.0721750;
  z = red*0.0193339 + green*0.1191920 + blue*0.9503041;

  // Now xyz 2 Lab
  double refX = 95.047;
  double refY = 100.0;
  double refZ = 108.883;

  x = x / refX;          //ref_X =  95.047   Observer= 2�, Illuminant= D65
  y = y / refY;          //ref_Y = 100.000
  z = z / refZ;          //ref_Z = 108.883

  if   (x>0.008856) x = pow(x, 1.0/3.0);
  else x = (7.787*x) + (16.0/116.0);

  if   (y>0.008856) y = pow(y, 1.0/3.0);
  else y = (7.787*y) + (16.0/116.0);

  if   (z>0.008856) z = pow(z, 1.0/3.0);
  else z = (7.787*z) + (16.0/116.0);

  *L = (116.0*y)-16;
  *a = 500*(x-y);
  *b = 200*(y-z);
} //end-StdRGB2LabOne

#else
///---------------------------------------------------------------------------------------------------
/// Std way of converting rgb-->Lab (whatever this means?)
///
void StdRGB2LabOne(unsigned char r, unsigned char g, unsigned char bl,
                   double *L, double *a, double *b) {
  double red, green, blue;
  double x, y, z;

  red = r / 255.0;
  green = g / 255.0;
  blue = bl / 255.0;

  red = red * 100;
  green = green * 100;
  blue = blue * 100;

  // Observer. = 2�, Illuminant = D65
  x = red * 0.4124564 + green * 0.3575761 + blue * 0.1804375;
  y = red * 0.2126729 + green * 0.7151522 + blue * 0.0721750;
  z = red * 0.0193339 + green * 0.1191920 + blue * 0.9503041;

  // Now xyz 2 Lab
  double refX = 95.047;
  double refY = 100.0;
  double refZ = 108.883;

  x = x / refX;  // ref_X =  95.047   Observer= 2�, Illuminant= D65
  y = y / refY;  // ref_Y = 100.000
  z = z / refZ;  // ref_Z = 108.883

  if (x > 0.008856)
    x = pow(x, 1.0 / 3.0);
  else
    x = (7.787 * x) + (16.0 / 116.0);

  if (y > 0.008856)
    y = pow(y, 1.0 / 3.0);
  else
    y = (7.787 * y) + (16.0 / 116.0);

  if (z > 0.008856)
    z = pow(z, 1.0 / 3.0);
  else
    z = (7.787 * z) + (16.0 / 116.0);

  *L = (116.0 * y) - 16;
  *a = 500 * (x - y);
  *b = 200 * (y - z);
}  // end-StdRGB2LabOne
#endif

// Burak - restores _CRT_SECURE_NO_DEPRECATE warnings
#pragma warning(default : 4996)