/*
 * Copyright (c) 2017, Ubiquity Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#ifndef ESTIMATE_H
#define ESTIMATE_H

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/calib3d.hpp>

#include <fiducial_msgs/Fiducial.h>
#include <fiducial_msgs/FiducialArray.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <fiducial_msgs/FiducialTransformArray.h>

#include <map>
#include <list>
#include <string>

#include <sensor_msgs/CameraInfo.h>

#include "fiducial_slam/map.h"

using namespace std;

class Estimator {
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;

    Map &map;

    bool haveCaminfo;

    double fiducialLen;
    double errorThreshold;

    int frameNum;
    string frameId;

    std::map<int, cv::Vec3d> rvecHistory;
    std::map<int, cv::Vec3d> tvecHistory;

    void estimatePose(int fid, const vector<Point3d> &worldPoints,
                      const vector<Point2d> &imagePoints,
                      Observation &obs, fiducial_msgs::FiducialTransform &ft,
                      const ros::Time& stamp, const string& frame);

    double getReprojectionError(const vector<Point3d> &objectPoints,
                                const vector<Point2d> &imagePoints,
                                const Vec3d &rvec, const Vec3d &tvec);

  public:
    Estimator(Map &fiducialMap);

    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

    void estimatePoses(const fiducial_msgs::FiducialArray::ConstPtr& msg,
                       vector<Observation> &observations,
                       fiducial_msgs::FiducialTransformArray &outMsg);

    void setFiducialLen(double fiducialLen) { this->fiducialLen = fiducialLen; };
    void setErrorThreshold(double errorThreshold) { this->errorThreshold = errorThreshold; };
};

#endif
