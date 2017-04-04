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

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>


#include <list>
#include <string>

using namespace std;
using namespace cv;

// Euclidean distance between two points
static double dist(const cv::Point2f &p1, const cv::Point2f &p2)
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
static double calcFiducialArea(std::vector<cv::Point2f> pts)
{
    Point2f &p0 = pts.at(0);
    Point2f &p1 = pts.at(1);
    Point2f &p2 = pts.at(2);
    Point2f &p3 = pts.at(3);

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

// An observation of a single fiducial in a single image
class Observation {
  public:
    int fid;
    double imageError;
    double objectError;
    tf2::Transform TfidCam;
    tf2::Transform TcamFid;
  
    Observation(int fid, Vec3d &rvec, Vec3d &tvec, double ierr, double oerr);
};

// A single fiducial that is in the map
class Fiducial {
  public:
    int id;
    tf2::Transform pose;
    double variance;
    ros::Time lastPublished;
    bool anchor;

    void update(tf2::Transform &pose, double variance);

    Fiducial() {}

    Fiducial(int id, tf2::Transform &T, double variance);
    Fiducial(int id, tf2::Quaternion &q, tf2::Vector3 &tvec, double variance,
             bool anchor);
};

// Class containing map data
class Map {
  public:
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer *tfBuffer;
    tf2_ros::TransformListener *listener;

    ros::Publisher *markerPub;
    string filename;

    map<int, Fiducial> fiducials;

    Map(ros::NodeHandle &nh);
    void update(vector<Observation> &obs, ros::Time time);
    void autoInit(vector<Observation> &obs, ros::Time time);
    void updateMap(vector<Observation> &obs, ros::Time time);
    void updatePose(vector<Observation> &obs, ros::Time time);
    bool load();
    bool save();
    void publishMarker(Fiducial &fid);
    void publishMarkers();
};

