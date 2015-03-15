#include <cstdio>
#include <opencv2/core/core.hpp>
#include "RPP.h"

using namespace cv;

inline double SQ(double x)
{
    return x*x;
}

int main(int argc, char **argv)
{
    // Data from the original Matlab code, we'll use it for ground truth. Verify the results with the Matlab output.
    // First row is x
    // Second row is y
    double model_data[20] = {0.0685, 0.6383, 0.4558, 0.7411, -0.7219, 0.7081, 0.7061, 0.2887, -0.9521, -0.2553,
                             0.4636, 0.0159, -0.1010, 0.2817, 0.6638, 0.1582, 0.3925, -0.7954, 0.6965, -0.7795};


    double iprts_data[20] = {-0.0168, 0.0377, 0.0277, 0.0373, -0.0824, 0.0386, 0.0317, 0.0360, -0.1015, -0.0080,
                              0.0866, 0.1179, 0.1233, 0.1035,  0.0667, 0.1102, 0.0969, 0.1660,  0.0622, 0.1608};


    // Results from Matlab/Octave
    // R is rotation and t is translation, you should get VERY similar results, or even numerically exact
    /*
      R =

     0.85763  -0.31179   0.40898
     0.16047  -0.59331  -0.78882
     0.48859   0.74214  -0.45881

    t =
     -0.10825
      1.26601
     11.19855
    */

    Mat model = Mat::zeros(3, 10, CV_64F); // 3D points, z is zero
    Mat iprts = Mat::ones(3, 10, CV_64F); // 2D points, homogenous points
    Mat rotation;
    Mat translation;
    int iterations;
    double obj_err;
    double img_err;

    for(int i=0; i < 10; i++) {
        model.at<double>(0,i) = model_data[i];
        model.at<double>(1,i) = model_data[i+10];

        iprts.at<double>(0,i) = iprts_data[i];
        iprts.at<double>(1,i) = iprts_data[i+10];
    }

    if(!RPP::Rpp(model, iprts, rotation, translation, iterations, obj_err, img_err)) {
        fprintf(stderr, "Error with RPP\n");
        return 1;
    }

	printf("Input normalised image points (using 3x3 camera intrinsic matrix):\n");
	for(int i=0; i < 10; i++) {
		printf("%d: (%g, %g)\n", i, iprts_data[i*2], iprts_data[i*2+1]);
	}

	printf("\n");
	printf("Input model points (fixed 3D points chosen by the user, z is fixed to 0.0):\n");
	for(int i=0; i < 10; i++) {
		printf("%d: (%g, %g, 0.0)\n", i, model_data[i*2], model_data[i*2+1]);
	}

	printf("\n");
	printf("Pose found by RPP\n");

	printf("\n");
    printf("Rotation matrix\n");
    RPP::Print(rotation);

    printf("Translation matrix\n");
    RPP::Print(translation);

    printf("Number of iterations: %d\n", iterations);
    printf("Object error: %f\n", obj_err);
    printf("Image error: %f\n", img_err);

    // Compare against ground truth
    double rot_err = 0.0;
    double tran_err = 0.0;

    // Using fabs instead of sum square, in case the signs are opposite
    rot_err += SQ(rotation.at<double>(0,0) -  0.85763);
    rot_err += SQ(rotation.at<double>(0,1) - -0.31179);
    rot_err += SQ(rotation.at<double>(0,2) - 0.40898);
    rot_err += SQ(rotation.at<double>(1,0) - 0.16047);
    rot_err += SQ(rotation.at<double>(1,1) - -0.59331);
    rot_err += SQ(rotation.at<double>(1,2) - -0.78882);
    rot_err += SQ(rotation.at<double>(2,0) - 0.48859);
    rot_err += SQ(rotation.at<double>(2,1) - 0.74214);
    rot_err += SQ(rotation.at<double>(2,2) - -0.45881);

    tran_err += SQ(translation.at<double>(0) - -0.10825);
    tran_err += SQ(translation.at<double>(1) - 1.26601);
    tran_err += SQ(translation.at<double>(2) - 11.19855);

    printf("\n");

    if(rot_err < 1e-4) {
        printf("Rotation results look correct! Square error = %g\n", rot_err);
    }
    else {
        printf("Rotation results does not look correct :( Square error = %g\n", rot_err);
    }

    if(tran_err < 1e-4) {
        printf("Translation results look correct! Square error = %g\n", tran_err);
    }
    else {
        printf("Translation results does not look correct :( Square error = %g\n", tran_err);
    }
}
