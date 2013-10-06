
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


/* 
 * dump a matrix to stdout
 */
void dumpMat(CvMat* M)
{
    for (int i=0; i<M->rows; i++) {
      for (int j=0; j<M->cols; j++)
          printf("%8.3lf ", cvGetReal2D(M, i, j));
      printf("\n");
    }
}


/* 
 * Read the calibration file and generate the undistortion maps
 * in:
 *   calibFile   -  camera calibration file
 *   w h         -  width and height of images to undistort
 * out:
 *   mapx, mapy, - undistortion maps
 */
int setupUndistortion(char *calibFile, int w, int h, IplImage** mapx, IplImage** mapy)
{
    double fcx, fcy, ccx, ccy;
    double kc[4];
  
    FILE *fp = fopen(calibFile, "r");
    if (fp == NULL) {
        fprintf(stderr, "Could not open \"%s\"\n", calibFile);
        return -1;
    }

    /*
     *  format is fc - focal length, cc, principal point, kc distortion vector
     */
    int x = fscanf(fp, "fc %lf %lf cc %lf %lf kc %lf %lf %lf %lf %lf", 
       &fcx, &fcy, &ccx, &ccy, &kc[0], &kc[1], &kc[2], &kc[3]);
    if (x != 8) {
        fprintf(stderr, "Expected 8 parameters got %d\n", x);
        return -1;
    }
    
    double intvec[9] = {
        fcx,   0, ccx,
          0, fcy, ccy,
          0,   0,   1
    }; 
    CvMat intrinsic = cvMat(3, 3, CV_64FC1, intvec);
    printf("intrinsic matrix\n");
    dumpMat(&intrinsic);

    CvMat distortion = cvMat(1, 4, CV_64FC1, kc);
    printf("distortion matrix\n");
    dumpMat(&distortion);

    *mapx = cvCreateImage(cvSize(w, h), IPL_DEPTH_32F, 1);
    *mapy = cvCreateImage(cvSize(w, h), IPL_DEPTH_32F, 1);

    cvInitUndistortMap(&intrinsic, &distortion, *mapx, *mapy);
    return 0;
}

int main(int argc, char *argv[])
{
    IplImage *mapx = NULL;
    IplImage *mapy = NULL;

    IplImage *image = NULL;
    IplImage *imageUndis = NULL;

    if (argc < 3) {
       fprintf(stderr, "Usage %s calibrationFile images ...\n", argv[0]);
       exit(1);
    } 
   
    cvNamedWindow("Original", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Undistorted", CV_WINDOW_AUTOSIZE);

    for (int i=2; i<argc; i++) {
        printf("image %s\n", argv[i]);
        image = cvLoadImage(argv[i], CV_LOAD_IMAGE_GRAYSCALE);
        if (image == NULL) {
            fprintf(stderr, "Could not load image \"%s\"\n", argv[i]);
            continue;
        }
        int w = image->width;
        int h = image->height;

        if (mapx == NULL) {
            int rc = setupUndistortion(argv[1], w, h, &mapx, &mapy);
            if (rc != 0)
                exit(1);
        }
 
        fprintf(stderr, "Maps initialized\n");
        if (imageUndis == NULL) 
            imageUndis = cvCreateImage(cvSize(w, h), image->depth, image->nChannels);
        cvRemap(image, imageUndis, mapx, mapy, CV_INTER_NN|CV_WARP_FILL_OUTLIERS, cvScalarAll(0));


        cvShowImage("Original", image);
        cvShowImage("Undistorted", imageUndis);
        //cvSaveImage("undisorted.jpg", imageUndis);
        cvWaitKey(0);
    }
}
