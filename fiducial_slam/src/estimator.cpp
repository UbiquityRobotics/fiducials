
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

#include "fiducial_slam/map.h"
#include "fiducial_slam/estimator.h"

/**
  * @brief Return object points for the system centered in a single marker, given the marker length
  */
static void getSingleMarkerObjectPoints(double markerLength, vector<Point3d>& objPoints) {

    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.push_back(Vec3d(-markerLength / 2.f, markerLength / 2.f, 0.f));
    objPoints.push_back(Vec3d( markerLength / 2.f, markerLength / 2.f, 0.f));
    objPoints.push_back(Vec3d( markerLength / 2.f,-markerLength / 2.f, 0.f));
    objPoints.push_back(Vec3d(-markerLength / 2.f,-markerLength / 2.f, 0.f));
}

// Euclidean distance between two points
static double dist(const cv::Point2d &p1, const cv::Point2d &p2)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

// Compute area in image of a fiducial, using Heron's formula
// to find the area of two triangles
static double calcFiducialArea(const std::vector<cv::Point2d> &pts)
{
    const Point2d &p0 = pts.at(0);
    const Point2d &p1 = pts.at(1);
    const Point2d &p2 = pts.at(2);
    const Point2d &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return a1+a2;
}

// estimate reprojection error
double Estimator::getReprojectionError(const vector<Point3d> &objectPoints,
                            const vector<Point2d> &imagePoints,
                            const Vec3d &rvec, const Vec3d &tvec) {

    vector<Point2d> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                      distortionCoeffs, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;
    for (size_t i=0; i<objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error*error;
    }
    double rerror = totalError/(float)objectPoints.size();
    return rerror;
}


void Estimator::estimatePose(int fid, const vector<Point3d> &worldPoints,
                              const vector<Point2d> &imagePoints,
                              Observation &obs, fiducial_msgs::FiducialTransform &ft,
                              const ros::Time& stamp, const string& frame)
{
    Vec3d rvec, tvec;
    bool haveHistory = false;

    if (rvecHistory.find(fid) != rvecHistory.end()) {
        rvec = rvecHistory[fid];
        tvec = tvecHistory[fid];
        haveHistory = true;
    }

    cv::solvePnP(worldPoints, imagePoints, cameraMatrix, distortionCoeffs, rvec, tvec, haveHistory);

    double reprojectionError =
          getReprojectionError(worldPoints, imagePoints, rvec, tvec);

    ROS_INFO("Detected id %d T %.2f %.2f %.2f R %.2f %.2f %.2f", fid,
              tvec[0], tvec[1], tvec[2], rvec[0], rvec[1], rvec[2]);

    double angle = norm(rvec);
    Vec3d axis = rvec / angle;
    ROS_INFO("angle %f axis %f %f %f", angle, axis[0], axis[1], axis[2]);

    tf2::Quaternion q;
    q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

    // Convert image_error (in pixels) to object_error (in meters)
        double objectError =
            (reprojectionError / dist(imagePoints[0], imagePoints[2])) *
            (norm(tvec) / fiducialLen);

    tf2::Transform T(q, tf2::Vector3(tvec[0], tvec[1], tvec[2]));

    obs = Observation(fid,
                      tf2::Stamped<TransformWithVariance>(TransformWithVariance(
                      T, objectError), stamp, frame));

    if (reprojectionError < errorThreshold) {
        rvecHistory[fid] = rvec;
        tvecHistory[fid] = tvec;
    }

    ft.fiducial_id = fid;

    ft.transform.translation.x = tvec[0];
    ft.transform.translation.y = tvec[1];
    ft.transform.translation.z = tvec[2];

    ft.transform.rotation.w = q.w();
    ft.transform.rotation.x = q.x();
    ft.transform.rotation.y = q.y();
    ft.transform.rotation.z = q.z();

    ft.fiducial_area = calcFiducialArea(imagePoints);
    ft.image_error = reprojectionError;
    ft.object_error = objectError;
}


Estimator::Estimator(Map &fiducialMap): map(fiducialMap)
{
    haveCaminfo = false;

    // Camera intrinsics
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

    // distortion coefficients
    distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);
}


void Estimator::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    if (haveCaminfo) {
        return;
    }

    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            cameraMatrix.at<double>(i, j) = msg->K[i*3+j];
        }
    }

    for (size_t i=0; i<msg->D.size(); i++) {
        distortionCoeffs.at<double>(0,i) = msg->D[i];
    }

    haveCaminfo = true;
    frameId = msg->header.frame_id;
}


void Estimator::estimatePoses(const fiducial_msgs::FiducialArray::ConstPtr& msg,
                              vector<Observation> &observations,
                              fiducial_msgs::FiducialTransformArray &outMsg)
{
    if (!haveCaminfo) {
        if (frameNum > 5) {
            ROS_ERROR("No camera intrinsics");
        }
        return;
    }

    vector<Point3d> markerObjPoints;
    getSingleMarkerObjectPoints(fiducialLen, markerObjPoints);

    vector<Point3d> allWorldPoints;
    vector<Point2d> allImagePoints;

    for (size_t i=0; i<msg->fiducials.size(); i++) {

        const fiducial_msgs::Fiducial& fid = msg->fiducials[i];

        vector<Point2d > corners;
        corners.push_back(Point2d(fid.x0, fid.y0));
        corners.push_back(Point2d(fid.x1, fid.y1));
        corners.push_back(Point2d(fid.x2, fid.y2));
        corners.push_back(Point2d(fid.x3, fid.y3));

        Vec3d rvec, tvec;

        if (map.fiducials.find(fid.fiducial_id) != map.fiducials.end()) {
            const tf2::Transform&  fiducialTransform =
                map.fiducials[fid.fiducial_id].pose.transform;

            for (int j=0; j<4; j++) {
                // vertex in coordinate system of fiducial
                Point3d& vertex = markerObjPoints[j];
                tf2::Vector3 vertex2(vertex.x, vertex.y, vertex.z);
                // vertex in world coordinates
                tf2::Vector3 worldPoint = fiducialTransform * vertex2;
                allWorldPoints.push_back(Point3d(worldPoint.x(), worldPoint.y(), worldPoint.z()));
                allImagePoints.push_back(corners[j]);
            }
        }

        Observation obs;
        fiducial_msgs::FiducialTransform ft;
        estimatePose(fid.fiducial_id, markerObjPoints, corners, obs, ft,
           msg->header.stamp, frameId);

        observations.push_back(obs);
        outMsg.transforms.push_back(ft);
    }

    if (allWorldPoints.size() > 0) {
        Observation obs;
        fiducial_msgs::FiducialTransform ft;

        estimatePose(0, allWorldPoints, allImagePoints, obs, ft,
           msg->header.stamp, frameId);

        observations.push_back(obs);
        outMsg.transforms.push_back(ft);
    }
}
