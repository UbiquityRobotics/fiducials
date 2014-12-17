/*
This is a C++ port using OpenCV of the Robust Pose Estimation from a Planar Target algorithm by Gerald Schweighofer and Axel Pinz.

It is a line by line port of the Matlab code at

http://www.emt.tugraz.at/~pinz/code/

I have no idea what their license is, if any. I'll make my code BSD for now unless I later find they have a more restrictive one.
*/

/*
Copyright 2011 Nghia Ho. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY NGHIA HO ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL NGHIA HO OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Nghia Ho.
*/

#ifndef __RPP_H__
#define __RPP_H__

#include <opencv2/core/core.hpp>
#include <vector>
#include "Rpoly.h"

namespace RPP
{

class Quaternion
{
public:

    cv::Vec3d vector;
    double scalar;

    Quaternion(){}
    Quaternion(const cv::Vec3d &v, double s) {
        vector = v;
        scalar = s;
    }
};

class Solution
{
public:
    Solution()
    {
        R = cv::Mat::zeros(3,3,CV_64F);
        t = cv::Mat::zeros(3,1,CV_64F);
    }

    Solution(const Solution &s) {
        R = s.R.clone();
        t = s.t.clone();
        E = s.E;
        bl = s.bl;
        at = s.at;
        obj_err = s.obj_err;
        img_err = s.img_err;
    }


    cv::Mat R;
    cv::Mat t;
    double E;
    double bl;
    double at;
    double obj_err;
    double img_err;
};

bool Rpp(const cv::Mat &model_3D, const cv::Mat &iprts,
            cv::Mat &Rlu, cv::Mat &tlu, int &it1, double &obj_err1, double &img_err1);

void ObjPose(const cv::Mat P, cv::Mat Qp, cv::Mat initR,
             cv::Mat &R, cv::Mat &t, int &it, double &obj_err, double &img_err);

void AbsKernel(cv::Mat P, cv::Mat Q, const std::vector <cv::Mat> &F, const cv::Mat &G,
               cv::Mat &R, cv::Mat &t, cv::Mat &Qout, double &err2);

cv::Mat EstimateT(const cv::Mat &R, const cv::Mat &G, const std::vector <cv::Mat> &F, const cv::Mat &P);
cv::Mat NormRv(const cv::Mat &R);
cv::Mat NormRv(const cv::Vec3d &V);

Quaternion Quaternion_byAngleAndVector(double q_angle, const cv::Vec3d &q_vector);
cv::Mat quat2mat(const Quaternion &Q);
cv::Mat GetRotationbyVector(const cv::Vec3d &v1, const cv::Vec3d &v2);

bool Get2ndPose_Exact(const cv::Mat &v, const cv::Mat &P, const cv::Mat &R, const cv::Mat &t, std::vector <Solution> &ret);

bool GetRfor2ndPose_V_Exact(const cv::Mat &v, const cv::Mat &P, const cv::Mat &R, const cv::Mat &t, std::vector <Solution> &ret);

void GetRotationY_wrtT(const cv::Mat &v, const cv::Mat &P, const cv::Mat &t, const cv::Mat &Rz,
                        std::vector <double> &al, cv::Mat &tnew, std::vector <double> &at);


bool DecomposeR(const cv::Mat &R, cv::Mat &Rz2, cv::Mat &ret);

cv::Mat RpyMat(const cv::Vec3d &angs);
bool RpyAng(const cv::Mat &R, cv::Vec3d &ret); // returns roll,pitch,yaw
cv::Mat Mean(const cv::Mat &m);
cv::Mat Sum(const cv::Mat &m, int dim=1);
cv::Mat Mul(const cv::Mat &a, const cv::Mat &b);
cv::Mat Sq(const cv::Mat &m);
cv::Mat Xform(const cv::Mat &P, const cv::Mat &R, const cv::Mat &t);
double Norm(const cv::Mat &m);

void Print(const cv::Mat &m);
void Print(const Quaternion &q);

// Makes homogenous points
cv::Mat Point2Mat(const std::vector <cv::Point3d> &pts);
cv::Mat Point2Mat(const std::vector <cv::Point2d> &pts);
cv::Mat Vec2Mat(const cv::Vec3d &v);

bool RpyAng_X(const cv::Mat &R, cv::Vec3d &ret);


inline Quaternion Quaternion_byVectorAndScalar(const cv::Vec3d &vector, double scalar)
{
    return Quaternion(vector, scalar);
}

inline Quaternion Quaternion_multiplyByScalar(const Quaternion &q, double scalar)
{
    Quaternion ret;

    ret.vector[0] = q.vector[0]*scalar;
    ret.vector[1] = q.vector[1]*scalar;
    ret.vector[2] = q.vector[2]*scalar;

    ret.scalar = q.scalar*scalar;

    return ret;
}

inline double Quaternion_Norm(const Quaternion &Q)
{
    return sqrt(Q.vector[0]*Q.vector[0] + Q.vector[1]*Q.vector[1] + Q.vector[2]*Q.vector[2] + Q.scalar*Q.scalar);
}

inline int sign(double x)
{
    if(x < 0)
        return -1;

    if(x > 0)
        return 1;

    return 0;
}

} // End namespace
#endif
