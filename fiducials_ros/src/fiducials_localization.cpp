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
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

#include "fiducials_ros/Fiducial.h"

class FiducialsNode {
  private:
    ros::Publisher * marker_pub;
    ros::Publisher * vertices_pub;
    image_transport::Subscriber img_sub;

    // transform bits
    tf2_ros::TransformBroadcaster tf_pub;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_sub;

    std::string world_frame;
    std::string pose_frame;
    std::string odom_frame;
    bool use_odom;

    // this would only be turned off if we are publishing the tf
    // in another node
    bool publish_tf;

    // this would only be turned off if we are publishing the markers
    // in another node
    bool publish_markers;

    // the last frame we saw on the camera header
    std::string last_camera_frame;

    int last_image_seq;

    // if set, we publish the images that contain fiducials
    bool publish_images;

    // if set, we publish the images that are "interesting", for debugging
    bool publish_interesting_images;

    image_transport::Publisher image_pub;
    image_transport::Publisher interesting_image_pub;

    const double scale;
    std::string fiducial_namespace;
    std::string position_namespace;

    std_msgs::ColorRGBA tag_color;
    std_msgs::ColorRGBA hidden_tag_color;
    std_msgs::ColorRGBA position_color;

    Fiducials fiducials;
    std::string tag_height_file;
    std::string data_directory;
    std::string map_file;
    std::string log_file;

    geometry_msgs::Pose scale_position(double x, double y, double z,
        double theta);
    visualization_msgs::Marker createMarker(std::string ns, int id);

    static void arc_announce(void *t, int from_id, double from_x,
        double from_y, double from_z, int to_id, double to_x, double to_y,
        double to_z, double goodness, int in_spanning_tree);

    static void tag_announce(void *t, int id, double x, double y, double z,
        double twist, double diagonal, double distance_per_pixel, int visible,
        int hop_count);
    void tag_cb(int id, double x, double y, double z, double twist, double dx,
        double dy, double dz, int visible);

    static void location_announce(void *t, int id, double x, double y,
        double z, double bearing);
    void location_cb(int id, double x, double y, double z, double bearing);

    static void fiducial_announce(void *t,
    int id, int direction, double world_diagonal,
        double x0, double y0, double x1, double y1,
        double x2, double y2, double x3, double y3);
    void fiducial_cb(int id, int direction, double world_diagonal,
        double x0, double y0, double x1, double y1,
        double x2, double y2, double x3, double y3);

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

void FiducialsNode::arc_announce(void *t, int from_id, double from_x,
    double from_y, double from_z, int to_id, double to_x, double to_y,
    double to_z, double goodness, int in_spanning_tree) {
}

void FiducialsNode::tag_announce(void *t, int id, double x, double y, double z,
  double twist, double diagonal, double distance_per_pixel, int visible,
  int hop_count) {
    ROS_INFO("tag_announce:id=%d x=%f y=%f twist=%f",
      id, x, y, twist);
    FiducialsNode * ths = (FiducialsNode*)t;
    // sqrt(2) = 1.414213...
    double dx = (diagonal * distance_per_pixel) / 1.4142135623730950488016887;
    double dy = dx;
    double dz = 1.0;
    ths->tag_cb(id, x, y, z, twist, dx, dy, dz, visible);
}

void FiducialsNode::fiducial_announce(void *t,
    int id, int direction, double world_diagonal,
    double x0, double y0, double x1, double y1,
    double x2, double y2, double x3, double y3) {

    FiducialsNode * ths = (FiducialsNode*)t;
    ths->fiducial_cb(id, direction, world_diagonal, 
        x0, y0, x1, y1, x2, y2, x3, y3);
}

void FiducialsNode::fiducial_cb(int id, int direction, double world_diagonal,
    double x0, double y0, double x1, double y1,
    double x2, double y2, double x3, double y3)
{
    fiducials_ros::Fiducial fid;

    ROS_INFO("fiducial: id=%d dir=%d diag=%f (%.2f,%.2f), (%.2f,%.2f), (%.2f,%.2f), (%.2f,%.2f)",
       id, direction, world_diagonal, x0, y0, x1, y1, x2, y2, x3, y3);

    fid.header.stamp = ros::Time::now();
    fid.header.frame_id = last_camera_frame;
    fid.image_seq = last_image_seq;
    fid.direction = direction;
    fid.fiducial_id = id;
    fid.x0 = x0; fid.y0 = y0;
    fid.x1 = x1; fid.y1 = y1;
    fid.x2 = x2; fid.y2 = y2;
    fid.x3 = x3; fid.y3 = y3;

    vertices_pub->publish(fid);
}



void FiducialsNode::tag_cb(int id, double x, double y, double z, double twist,
    double dx, double dy, double dz, int visible) {

    if (!publish_markers)
       return;

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

tf2::Transform msg_to_tf(geometry_msgs::TransformStamped &msg) {
  return tf2::Transform(
            tf2::Quaternion(
              msg.transform.rotation.x,
              msg.transform.rotation.y,
              msg.transform.rotation.z,
              msg.transform.rotation.w),
            tf2::Vector3(
              msg.transform.translation.x,
              msg.transform.translation.y,
              msg.transform.translation.z));
}

void FiducialsNode::location_announce(void * t, int id, double x, double y,
    double z,double bearing) {
    FiducialsNode * ths = (FiducialsNode*)t;
    ths->location_cb(id, x, y, z, bearing);
}

void FiducialsNode::location_cb(int id, double x, double y, double z,
    double bearing) {
    ROS_INFO("location_announce:id=%d x=%f y=%f bearing=%f",
      id, x, y, bearing * 180. / 3.1415926);

    visualization_msgs::Marker marker = createMarker(position_namespace, id);
    ros::Time now = marker.header.stamp;

    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = scale_position(x, y, z, bearing);

    marker.scale.x = 0.2 / scale;
    marker.scale.y = 0.05 / scale;
    marker.scale.z = 0.05 / scale;

    marker.color = position_color;

    marker.lifetime = ros::Duration();

    marker_pub->publish(marker);

    // TODO: subtract out odometry position, and publish transform from
    //  map to odom
    double tf_x = marker.pose.position.x;
    double tf_y = marker.pose.position.y;
    double tf_yaw = bearing;

    // publish a transform based on the position
    if( use_odom ) {
      // if we're using odometry, look up the odom transform and subtract it
      //  from our position so that we can publish a map->odom transform
      //  such that map->odom->base_link reports the correct position
      std::string tf_err;
      if( tf_buffer.canTransform(pose_frame, odom_frame, now,
            ros::Duration(0.1), &tf_err ) ) {
        // get odometry position from TF
        tf2::Quaternion tf_quat;
        tf_quat.setRPY(0.0, 0.0, tf_yaw);

        tf2::Transform pose(tf_quat, tf2::Vector3(tf_x, tf_y, 0));


        geometry_msgs::TransformStamped odom;
        odom = tf_buffer.lookupTransform(odom_frame, pose_frame, now);
        tf2::Transform odom_tf = msg_to_tf(odom);

        // M = C * O
        // C^-1 * M = O
        // C^-1 = O * M-1
        tf2::Transform odom_correction = (odom_tf * pose.inverse()).inverse();

        // look up camera transform if we can
        if( last_camera_frame.length() > 0 ) {
          if( tf_buffer.canTransform(pose_frame, last_camera_frame, now,
                ros::Duration(0.1), &tf_err) ) {
            geometry_msgs::TransformStamped camera_tf;
            camera_tf = tf_buffer.lookupTransform(pose_frame,
                                                    last_camera_frame, now);
            tf2::Transform camera = msg_to_tf(camera_tf);
            odom_correction = odom_correction * camera.inverse();
          } else {
            ROS_ERROR("Cannot look up transform from %s to %s: %s",
                pose_frame.c_str(), last_camera_frame.c_str(), tf_err.c_str());
          }
        }
        
        geometry_msgs::TransformStamped transform;
        tf2::Vector3 odom_correction_v = odom_correction.getOrigin();
        transform.transform.translation.x = odom_correction_v.getX();
        transform.transform.translation.y = odom_correction_v.getY();
        transform.transform.translation.z = odom_correction_v.getZ();

        tf2::Quaternion odom_correction_q = odom_correction.getRotation();
        transform.transform.rotation.x = odom_correction_q.getX();
        transform.transform.rotation.y = odom_correction_q.getY();
        transform.transform.rotation.z = odom_correction_q.getZ();
        transform.transform.rotation.w = odom_correction_q.getW();

        transform.header.stamp = now;
        transform.header.frame_id = world_frame;
        transform.child_frame_id = odom_frame;
        //tf2::transformTF2ToMsg(odom_correction, transform, now, world_frame,
            //odom_frame);

        if (publish_tf)
            tf_pub.sendTransform(transform);
      } else {
        ROS_ERROR("Can't look up base transform from %s to %s: %s",
            pose_frame.c_str(),
            odom_frame.c_str(),
            tf_err.c_str());
      }
    } else {
      // we're publishing absolute position
      geometry_msgs::TransformStamped transform;
      transform.header.stamp = now;
      transform.header.frame_id = world_frame;
      transform.child_frame_id = pose_frame;

      transform.transform.translation.x = tf_x;
      transform.transform.translation.y = tf_y;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation = tf::createQuaternionMsgFromYaw(tf_yaw);

      if (publish_tf)
          tf_pub.sendTransform(transform);
    }
}

void FiducialsNode::imageCallback(const sensor_msgs::ImageConstPtr & msg) {
    ROS_INFO("Got image");
    last_camera_frame = msg->header.frame_id;
    last_image_seq = msg->header.seq;
    try {
        cv_bridge::CvImageConstPtr cv_img;
        cv_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        IplImage *image = new IplImage(cv_img->image);
        if(fiducials == NULL) {
            ROS_INFO("Got first image! Setting up Fiducials library");
	    // Load up *fiducials_create*:
	    Fiducials_Create fiducials_create =
	      Fiducials_Create__one_and_only();
	    fiducials_create->fiducials_path = data_directory.c_str();
	    fiducials_create->lens_calibrate_file_name = (String_Const)0;
	    fiducials_create->announce_object = (Memory)this;
	    fiducials_create->arc_announce_routine = arc_announce;
	    fiducials_create->location_announce_routine = location_announce;
	    fiducials_create->tag_announce_routine = tag_announce;
	    fiducials_create->log_file_name = log_file.c_str();
	    fiducials_create->map_base_name = map_file.c_str();
	    fiducials_create->tag_heights_file_name = tag_height_file.c_str();
            fiducials_create->fiducial_announce_routine = fiducial_announce;

	    // Create *fiducials* object using first image:
            fiducials = Fiducials__create(image, fiducials_create);
        }
        Fiducials__image_set(fiducials, image);
        Fiducials_Results results = Fiducials__process(fiducials);
	if (publish_images) {
  	    if (results->map_changed) {
	      image_pub.publish(msg);
            }
        }
	if (publish_interesting_images) {
	  if (results->image_interesting) {
	    interesting_image_pub.publish(msg);
	  }
	}
    } catch(cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

FiducialsNode::FiducialsNode(ros::NodeHandle & nh) : scale(0.75), tf_sub(tf_buffer) {
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

    // define position to be blue
    position_color.r = 0.0f;
    position_color.g = 0.0f;
    position_color.b = 1.0f;
    position_color.a = 1.0f;

    nh.param<std::string>("tag_height", tag_height_file, "Tag_Heights.xml");
    nh.param<std::string>("data_directory", data_directory, ".");
    nh.param<std::string>("map_file", map_file, "ROS_Map");
    nh.param<std::string>("log_file", log_file, "fiducials.log.txt");
    nh.param<std::string>("map_frame", world_frame, "map");
    nh.param<std::string>("pose_frame", pose_frame, "base_link");
    ROS_INFO("Publishing transform from %s to %s", world_frame.c_str(),
        pose_frame.c_str());

    if( nh.hasParam("odom_frame") ) {
      use_odom = true;
      nh.getParam("odom_frame", odom_frame);
      ROS_INFO("Using odometry frame %s", odom_frame.c_str());
    } else {
      use_odom = false;
      ROS_INFO("Not using odometry");
    }

    nh.param<bool>("publish_images", publish_images, false);
    nh.param<bool>("publish_tf", publish_tf, true);
    nh.param<bool>("publish_markers", publish_markers, true);
    nh.param<bool>("publish_interesting_images", publish_interesting_images, 
		   false);

    image_transport::ImageTransport img_transport(nh);

    if (publish_images) {
      image_pub = img_transport.advertise("fiducials_images", 1);
    }
    if (publish_interesting_images) {
      interesting_image_pub = img_transport.advertise("interesting_images", 1);
    }

    marker_pub = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("fiducials", 1));
   
    vertices_pub = new ros::Publisher(nh.advertise<fiducials_ros::Fiducial>("vertices", 1));
 
    fiducials = NULL;

    img_sub = img_transport.subscribe("camera", 1,
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
