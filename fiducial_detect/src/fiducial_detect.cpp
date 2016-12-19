/*
 * Copyright (c) 2014, Austin Hendrix
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
 * Author: Austin Hendrix <namniart@gmail.com>
 */

#include <assert.h>
#include <sys/time.h>
#include <unistd.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <list>
#include <string>

#include "fiducial_lib/Fiducials.hpp"
#include "fiducial_lib/File.hpp"

#include "fiducial_pose/rosrpp.h"

#include "fiducial_pose/Fiducial.h"
#include "fiducial_pose/FiducialTransform.h"
#include "fiducial_pose/FiducialTransformArray.h"

class FiducialsNode {
private:
    ros::Publisher vertices_pub;
    ros::Publisher pose_pub;
    fiducial_pose::FiducialTransformArray fiducialTransformArray;

    ros::Subscriber caminfo_sub;
    image_transport::Subscriber img_sub;
    bool processing_image;
    int frameNum;
    bool haveCamInfo;

    RosRpp *pose_est;


    // the last frame we saw on the camera header
    std::string last_camera_frame;

    int last_image_seq;
    ros::Time last_image_time;

    // if set, we publish the images that contain fiducials
    bool publish_images;

    // pose estimtion params
    bool estimate_pose;
    double fiducial_len;
    bool undistort_points;

    image_transport::Publisher image_pub;

    const double scale;

    Fiducials fiducials;
    std::string tag_height_file;
    std::string data_directory;
    std::string map_file;
    std::string log_file;

    std::vector<fiducial_pose::Fiducial> detected_fiducials;

    geometry_msgs::Pose scale_position(double x, double y, double z,
                                       double theta);
    visualization_msgs::Marker createMarker(std::string ns, int id);

    static void arc_announce(void *t, int from_id, double from_x, double from_y,
                             double from_z, int to_id, double to_x, double to_y,
                             double to_z, double goodness,
                             bool in_spanning_tree);

    static void tag_announce(void *t, int id, double x, double y, double z,
                             double twist, double diagonal,
                             double distance_per_pixel, bool visible,
                             int hop_count);
    void tag_cb(int id, double x, double y, double z, double twist, double dx,
                double dy, double dz, bool visible);

    static void location_announce(void *t, int id, double x, double y, double z,
                                  double bearing);
    void location_cb(int id, double x, double y, double z, double bearing);

    static void fiducial_announce(void *t, int id, int direction,
                                  double world_diagonal, double x0, double y0,
                                  double x1, double y1, double x2, double y2,
                                  double x3, double y3);

    void fiducial_cb(int id, int direction, double world_diagonal, double x0,
                     double y0, double x1, double y1, double x2, double y2,
                     double x3, double y3);

    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void processImage(const sensor_msgs::ImageConstPtr &msg);
    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

    boost::thread *update_thread;

public:
    FiducialsNode(ros::NodeHandle &nh);
    ~FiducialsNode();
};

FiducialsNode::~FiducialsNode() {
    if (update_thread) {
        update_thread->join();
        delete update_thread;
        update_thread = NULL;
    }
}

geometry_msgs::Pose FiducialsNode::scale_position(double x, double y, double z,
                                                  double theta) {
    geometry_msgs::Pose res;
    res.position.x = x / scale;
    res.position.y = y / scale;
    res.position.z = z / scale;

    res.orientation = tf::createQuaternionMsgFromYaw(theta);

    return res;
}

void FiducialsNode::arc_announce(void *t, int from_id, double from_x,
                                 double from_y, double from_z, int to_id,
                                 double to_x, double to_y, double to_z,
                                 double goodness, bool in_spanning_tree) {}

void FiducialsNode::tag_announce(void *t, int id, double x, double y, double z,
                                 double twist, double diagonal,
                                 double distance_per_pixel, bool visible,
                                 int hop_count) {
    ROS_INFO("tag_announce:id=%d x=%f y=%f twist=%f", id, x, y, twist);
    FiducialsNode *ths = (FiducialsNode *)t;
    // sqrt(2) = 1.414213...
    double dx = (diagonal * distance_per_pixel) / 1.4142135623730950488016887;
    double dy = dx;
    double dz = 1.0;
    ths->tag_cb(id, x, y, z, twist, dx, dy, dz, visible);
}

void FiducialsNode::fiducial_announce(void *t, int id, int direction,
                                      double world_diagonal, double x0,
                                      double y0, double x1, double y1,
                                      double x2, double y2, double x3,
                                      double y3) {
    FiducialsNode *ths = (FiducialsNode *)t;
    ths->fiducial_cb(id, direction, world_diagonal, x0, y0, x1, y1, x2, y2, x3,
                     y3);
}

void FiducialsNode::fiducial_cb(int id, int direction, double world_diagonal,
                                double x0, double y0, double x1, double y1,
                                double x2, double y2, double x3, double y3) {
    fiducial_pose::Fiducial fid;

    ROS_INFO(
        "fiducial: id=%d dir=%d diag=%f (%.2f,%.2f), (%.2f,%.2f), (%.2f,%.2f), "
        "(%.2f,%.2f)",
        id, direction, world_diagonal, x0, y0, x1, y1, x2, y2, x3, y3);

    fid.header.stamp = last_image_time;
    fid.header.frame_id = last_camera_frame;
    fid.image_seq = last_image_seq;
    fid.direction = direction;
    fid.fiducial_id = id;
    fid.x0 = x0;
    fid.y0 = y0;
    fid.x1 = x1;
    fid.y1 = y1;
    fid.x2 = x2;
    fid.y2 = y2;
    fid.x3 = x3;
    fid.y3 = y3;

    vertices_pub.publish(fid);
    detected_fiducials.push_back(fid);

    if (estimate_pose) {
        if (!haveCamInfo && frameNum < 5) {
            return;
        }
        fiducial_pose::FiducialTransform ft;
        geometry_msgs::Transform trans;
        ft.transform = trans;
        if (pose_est->fiducialCallback(&fid, &ft)) {
            fiducialTransformArray.transforms.push_back(ft);
        }
    }
}

void FiducialsNode::tag_cb(int id, double x, double y, double z, double twist,
                           double dx, double dy, double dz, bool visible) {
}

tf2::Transform msg_to_tf(geometry_msgs::TransformStamped &msg) {
    return tf2::Transform(
        tf2::Quaternion(msg.transform.rotation.x, msg.transform.rotation.y,
                        msg.transform.rotation.z, msg.transform.rotation.w),
        tf2::Vector3(msg.transform.translation.x, msg.transform.translation.y,
                     msg.transform.translation.z));
}

void FiducialsNode::location_announce(void *t, int id, double x, double y,
                                      double z, double bearing) {
    FiducialsNode *ths = (FiducialsNode *)t;
    ths->location_cb(id, x, y, z, bearing);
}

void FiducialsNode::location_cb(int id, double x, double y, double z,
                                double bearing) {
}

void FiducialsNode::camInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr &msg) {
    if (pose_est) {
        pose_est->camInfoCallback(msg);
    }
    haveCamInfo = true;

    last_camera_frame = msg->header.frame_id;
}

void FiducialsNode::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    if (!processing_image) {
        processing_image = true;
        if (update_thread) {
            update_thread->join();
            delete update_thread;
            update_thread = NULL;
        }
        processing_image = true;
        update_thread = new boost::thread(
            boost::bind(&FiducialsNode::processImage, this, msg));
    } else {
        ROS_INFO("Dropping image");
    }
}

void FiducialsNode::processImage(const sensor_msgs::ImageConstPtr &msg) {
    processing_image = true;
    frameNum++;

    last_image_seq = msg->header.seq;
    last_image_time = msg->header.stamp;
    ROS_INFO("Got image seq %d", last_image_seq);

    fiducialTransformArray.transforms.clear();
    fiducialTransformArray.header.stamp = msg->header.stamp;
    fiducialTransformArray.header.frame_id = last_camera_frame;
    fiducialTransformArray.image_seq = msg->header.seq;

    try {
        cv_bridge::CvImageConstPtr cv_img;
        cv_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        IplImage *image = new IplImage(cv_img->image);
        if (fiducials == NULL) {
            ROS_INFO("Got first image! Setting up Fiducials library");
            // Load up *fiducials_create*:
            Fiducials_Create fiducials_create =
                Fiducials_Create__one_and_only();
            fiducials_create->fiducials_path = data_directory.c_str();
            fiducials_create->lens_calibrate_file_name = (String_Const)0;
            fiducials_create->announce_object = (Memory) this;
            fiducials_create->arc_announce_routine = arc_announce;
            fiducials_create->location_announce_routine = location_announce;
            fiducials_create->tag_announce_routine = tag_announce;
            fiducials_create->log_file_name = log_file.c_str();
            fiducials_create->map_base_name = map_file.c_str();
            fiducials_create->tag_heights_file_name = tag_height_file.c_str();
            fiducials_create->fiducial_announce_routine = fiducial_announce;
            fiducials_create->do_2d_slam = false;

            // Create *fiducials* object using first image:
            fiducials = Fiducials__create(image, fiducials_create);
        }
        Fiducials__image_set(fiducials, image);
        Fiducials_Results results = Fiducials__process(fiducials);
        ROS_INFO("Processed image");
        if (publish_images) {
            for (unsigned i = 0; i < detected_fiducials.size(); i++) {
                fiducial_pose::Fiducial &fid = detected_fiducials[i];
                cvLine(image, cvPoint(fid.x0, fid.y0), cvPoint(fid.x1, fid.y1),
                       CV_RGB(255, 0, 0), 3);
                cvLine(image, cvPoint(fid.x1, fid.y1), cvPoint(fid.x2, fid.y2),
                       CV_RGB(255, 0, 0), 3);
                cvLine(image, cvPoint(fid.x2, fid.y2), cvPoint(fid.x3, fid.y3),
                       CV_RGB(255, 0, 0), 3);
                cvLine(image, cvPoint(fid.x3, fid.y3), cvPoint(fid.x0, fid.y0),
                       CV_RGB(255, 0, 0), 3);
            }
            detected_fiducials.clear();
            image_pub.publish(msg);
        }
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    pose_pub.publish(fiducialTransformArray);
    ROS_INFO("Finished processing image seq %d", last_image_seq);
    processing_image = false;
}

FiducialsNode::FiducialsNode(ros::NodeHandle &nh) : scale(0.75) {
    frameNum = 0;
    haveCamInfo = false;
    processing_image = false;
    update_thread = NULL;

    nh.param<std::string>("tag_height", tag_height_file, "");
    nh.param<std::string>("data_directory", data_directory, ".");
    nh.param<std::string>("map_file", map_file, "ROS_Map");
    nh.param<std::string>("log_file", log_file, "fiducials.log.txt");

    nh.param<bool>("publish_images", publish_images, false);
    nh.param<bool>("estimate_pose", estimate_pose, true);

    nh.param<double>("fiducial_len", fiducial_len, 0.146);
    nh.param<bool>("undistort_points", undistort_points, false);

    image_transport::ImageTransport img_transport(nh);

    if (publish_images) {
        image_pub = img_transport.advertise("fiducial_images", 1);
    }

    vertices_pub = nh.advertise<fiducial_pose::Fiducial>("/fiducial_vertices", 1);

    if (estimate_pose) {
        pose_pub =  nh.advertise<fiducial_pose::FiducialTransformArray>(
                    "/fiducial_transforms", 1);
        pose_est = new RosRpp(fiducial_len, undistort_points);
    } else {
        pose_est = NULL;
    }

    fiducials = NULL;

    img_sub = img_transport.subscribe("/camera", 1,
                                      &FiducialsNode::imageCallback, this);

    caminfo_sub =
        nh.subscribe("/camera_info", 1, &FiducialsNode::camInfoCallback, this);

    ROS_INFO("Fiducials Localization ready");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fiducial_detect");
    ros::NodeHandle nh("~");

    FiducialsNode node(nh);

    ros::spin();

    return 0;
}
