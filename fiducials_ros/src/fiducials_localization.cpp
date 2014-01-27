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

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <list>
#include <string>

#include "fiducials/File.h"
#include "fiducials/Fiducials.h"
#include "fiducials/List.h"
#include "fiducials/Logical.h"

/// @brief Print out tag update information.
/// @param anounce_object is an opaque object from *Map*->*announce_object*.
/// @param id is the tag id.
/// @param x is the tag X location.
/// @param y is the tag Y location.
/// @param z is the tag Z location.
/// @param twist is the tag twist in radians.
/// @param dx is the tag size along the X axis (before twist).
/// @param dy is the tag size along the Y axis (before twist).
/// @param dz is the tag height in the Z axis.
///
/// *Map__tag_announce*() is called each time the map algorithm
/// updates the location or twist for a *tag*.

ros::Publisher * marker_pub;
tf::TransformBroadcaster * tf_pub;

std::string world_frame;
std::string pose_frame;

const double scale = 1000.0;
std::string fiducial_namespace;
std::string position_namespace;

std_msgs::ColorRGBA tag_color;
std_msgs::ColorRGBA position_color;

Fiducials fiducials;
std::string tag_height_file;

void tag_announce(void *rviz, int id,
  double x, double y, double z, double twist, double dx, double dy, double dz,
  int visible) {
    ROS_INFO("tag_announce:id=%d x=%f y=%f twist=%f\n",
      id, x, y, twist);

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = world_frame;

    marker.ns = fiducial_namespace;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x / scale;
    marker.pose.position.y = y / scale;
    marker.pose.position.z = z / scale;

    marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    marker.scale.x = dx / scale;
    marker.scale.y = dy / scale;
    marker.scale.z = dz / scale;

    marker.color = tag_color;

    // cycle through tag colors
    tag_color.r += 0.1;
    if( tag_color.r > 1.0 ) {
      tag_color.r = 0.0;
      tag_color.g += 0.1;
    }
    if( tag_color.g > 1.0 ) {
      tag_color.g = 0.0;
      tag_color.b += 0.1;
    }
    if( tag_color.b > 1.0 ) {
      tag_color.b = 0.0;
    }

    marker.lifetime = ros::Duration();

    marker_pub->publish(marker);

    // publish text(ID) version of marker
    char str_id[12];
    snprintf(str_id, 12, "%d", id);
    marker.text = str_id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.id = id + 10000;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.pose.position.z += 0.05;
    marker.ns = fiducial_namespace + "_text";
    marker_pub->publish(marker);
}

void location_announce(void *rviz, int id,
  double x, double y, double z, double bearing) {
    ROS_INFO("location_announce:id=%d x=%f y=%f bearing=%f\n",
      id, x, y, bearing * 180. / 3.1415926);

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = world_frame;

    marker.ns = position_namespace;
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x / scale;
    marker.pose.position.y = y / scale;
    marker.pose.position.z = z / scale;

    marker.pose.orientation = tf::createQuaternionMsgFromYaw(bearing);

    marker.scale.x = 200.0 / scale;
    marker.scale.y = 50.0 / scale;
    marker.scale.z = 50.0 / scale;

    marker.color = position_color;

    marker.lifetime = ros::Duration();

    marker_pub->publish(marker);
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(marker.pose.position.x, 
                                     marker.pose.position.y,
                                     marker.pose.position.z));
    transform.setRotation( tf::Quaternion(marker.pose.orientation.x,
                                          marker.pose.orientation.y,
                                          marker.pose.orientation.z,
                                          marker.pose.orientation.w));
    tf_pub->sendTransform(tf::StampedTransform(transform, marker.header.stamp,
          world_frame, pose_frame));
}

void imageCallback(const sensor_msgs::ImageConstPtr & msg) {
    ROS_INFO("Got image");
    try {
        cv_bridge::CvImageConstPtr cv_img;
        cv_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        IplImage *image = new IplImage(cv_img->image);
        if(fiducials == NULL) {
            ROS_INFO("Git first image! Setting up Fiducials library");
            fiducials = Fiducials__create(image, NULL, NULL, location_announce,
              tag_announce);
            Fiducials__tag_heights_xml_read(fiducials, tag_height_file.c_str());
        }
        Fiducials__image_set(fiducials, image);
        Fiducials__process(fiducials);
    } catch(cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char ** argv) {
    struct timeval start_time_value_struct;    
    struct timeval end_time_value_struct;    
    struct timeval difference_time_value_struct;    
    Time_Value start_time_value = &start_time_value_struct;
    Time_Value end_time_value = &end_time_value_struct;
    Time_Value difference_time_value = &difference_time_value_struct;

    fiducial_namespace = "fiducials";
    position_namespace = "position";
    // Define tags to be green
    tag_color.r = 0.0f;
    tag_color.g = 1.0f;
    tag_color.b = 0.0f;
    tag_color.a = 1.0f;

    // define position ot be blue
    position_color.r = 0.0f;
    position_color.g = 0.0f;
    position_color.b = 1.0f;
    position_color.a = 1.0f;

    world_frame = "map";
    pose_frame = "base_link";

    ros::init(argc, argv, "fiducials_localization");
    ros::NodeHandle nh("~");
    nh.param<std::string>("tag_height", tag_height_file, "Tag_Heights.xml");

    marker_pub = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("fiducials", 1));

    tf_pub = new tf::TransformBroadcaster();

    fiducials = NULL;

    image_transport::ImageTransport img_transport(nh);
    image_transport::Subscriber img_sub = img_transport.subscribe("camera", 1,
        imageCallback);

    ROS_INFO("Fiducials Localization ready");

    ros::spin();

    return 0;
}

