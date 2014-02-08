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
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
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

class FiducialsNode {
  private:
    ros::Publisher * marker_pub;

    // transform bits
    tf2_ros::TransformBroadcaster tf_pub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_sub;

    std::string world_frame;
    std::string output_frame;
    std::string odom_frame;
    bool use_odom;

    const double scale;
    std::string fiducial_namespace;
    std::string position_namespace;

    std_msgs::ColorRGBA tag_color;
    std_msgs::ColorRGBA hidden_tag_color;
    std_msgs::ColorRGBA position_color;

    Fiducials fiducials;
    std::string tag_height_file;

    geometry_msgs::Pose scale_position(double x, double y, double z,
        double theta);
    visualization_msgs::Marker createMarker(std::string ns, int id);

    static void tag_announce(void *t, int id, double x, double y, double z,
        double twist, double dx, double dy, double dz, int visible);
    void tag_cb(int id, double x, double y, double z, double twist, double dx,
        double dy, double dz, int visible);

    static void location_announce(void *t, int id, double x, double y,
        double z, double bearing);
    void location_cb(int id, double x, double y, double z, double bearing);

    void imageCallback(const sensor_msgs::ImageConstPtr & msg);

  public:
    FiducialsNode(ros::NodeHandle &nh);
};

geometry_msgs::Pose FiducialsNode::scale_position(double x, double y, 
    double z, double theta) {
  geometry_msgs::Pose res;
  res.position.x = x / scale;
  res.position.y = y / scale;
  res.position.z = z / scale;

  res.orientation = tf::createQuaternionMsgFromYaw(theta);

  return res;
}

visualization_msgs::Marker FiducialsNode::createMarker(std::string ns, int id) {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = world_frame;
    marker.ns = ns;
    marker.id = id;
    return marker;
}

void FiducialsNode::tag_announce(void *t, int id,
  double x, double y, double z, double twist, double dx, double dy, double dz,
  int visible) {
    ROS_INFO("tag_announce:id=%d x=%f y=%f twist=%f\n",
      id, x, y, twist);
    FiducialsNode * ths = (FiducialsNode*)t;
    ths->tag_cb(id, x, y, z, twist, dx, dy, dz, visible);
}

void FiducialsNode::tag_cb(int id, double x, double y, double z, double twist,
    double dx, double dy, double dz, int visible) {
    visualization_msgs::Marker marker = createMarker(fiducial_namespace, id);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = scale_position(x, y, z, 0);

    marker.scale.x = dx / scale;
    marker.scale.y = dy / scale;
    marker.scale.z = dz / scale;

    if( visible ) {
      marker.color = tag_color;
    } else {
      marker.color = hidden_tag_color;
    }

    marker.lifetime = ros::Duration();

    marker_pub->publish(marker);

    // publish text(ID) version of marker
    char str_id[12];
    snprintf(str_id, 12, "%d", id);
    marker.text = str_id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.color.r = marker.color.g = marker.color.b = 1.0; // white
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
    marker.id = id + 10000;
    marker.pose.position.z += 0.05; // draw text above marker
    marker.ns = fiducial_namespace + "_text";
    marker_pub->publish(marker);
}


void FiducialsNode::location_announce(void * t, int id, double x, double y,
    double z,double bearing) {
    FiducialsNode * ths = (FiducialsNode*)t;
    ths->location_cb(id, x, y, z, bearing);
}

void FiducialsNode::location_cb(int id, double x, double y, double z,
    double bearing) {
    ROS_INFO("location_announce:id=%d x=%f y=%f bearing=%f\n",
      id, x, y, bearing * 180. / 3.1415926);

    visualization_msgs::Marker marker = createMarker(world_frame, id);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = scale_position(x, y, z, bearing);

    marker.scale.x = 200.0 / scale;
    marker.scale.y = 50.0 / scale;
    marker.scale.z = 50.0 / scale;

    marker.color = position_color;

    marker.lifetime = ros::Duration();

    marker_pub->publish(marker);

    // publish a transform based on the position
    geometry_msgs::TransformStamped transform;
    // TODO: subtract out odometry position, and publish transform from
    //  map to odom
    transform.transform.translation.x = marker.pose.position.x;
    transform.transform.translation.y = marker.pose.position.y;
    transform.transform.translation.z = marker.pose.position.z;
    if( use_odom ) {
    }
    transform.transform.rotation = marker.pose.orientation;
    transform.header.stamp = marker.header.stamp;
    transform.header.frame_id = world_frame;
    transform.child_frame_id = output_frame;

    tf_pub.sendTransform(transform);
}

void FiducialsNode::imageCallback(const sensor_msgs::ImageConstPtr & msg) {
    ROS_INFO("Got image");
    try {
        cv_bridge::CvImageConstPtr cv_img;
        cv_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        IplImage *image = new IplImage(cv_img->image);
        if(fiducials == NULL) {
            ROS_INFO("Git first image! Setting up Fiducials library");
            fiducials = Fiducials__create(image, NULL, NULL, location_announce,
	      tag_announce, NULL, NULL);
            Fiducials__tag_heights_xml_read(fiducials, tag_height_file.c_str());
        }
        Fiducials__image_set(fiducials, image);
        Fiducials__process(fiducials);
    } catch(cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

FiducialsNode::FiducialsNode(ros::NodeHandle & nh) : scale(1000.0), tf_sub(tf_buffer) {
    fiducial_namespace = "fiducials";
    position_namespace = "position";
    // Define tags to be green
    tag_color.r = 0.0f;
    tag_color.g = 1.0f;
    tag_color.b = 0.0f;
    tag_color.a = 1.0f;

    // Define hidden tags to be red
    hidden_tag_color.r = 1.0f;
    hidden_tag_color.g = 0.0f;
    hidden_tag_color.b = 0.0f;
    hidden_tag_color.a = 1.0f;

    // define position ot be blue
    position_color.r = 0.0f;
    position_color.g = 0.0f;
    position_color.b = 1.0f;
    position_color.a = 1.0f;

    nh.param<std::string>("tag_height", tag_height_file, "Tag_Heights.xml");
    nh.param<std::string>("map_frame", world_frame, "map");
    if( nh.hasParam("odom_frame") ) {
      use_odom = true;
      nh.getParam("odom_frame", odom_frame);
      nh.param<std::string>("output_frame", output_frame, "odom");
    } else {
      use_odom = false;
      nh.param<std::string>("output_frame", output_frame, "base_link");
    }

    marker_pub = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("fiducials", 1));

    fiducials = NULL;

    image_transport::ImageTransport img_transport(nh);
    image_transport::Subscriber img_sub = img_transport.subscribe("camera", 1,
        &FiducialsNode::imageCallback, this);

    ROS_INFO("Fiducials Localization ready");
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "fiducials_localization");
    ros::NodeHandle nh("~");

    FiducialsNode * node = new FiducialsNode(nh);

    ros::spin();

    return 0;
}

