// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include <sys/time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <list>
#include <string>

#include "fiducials/CV.h"
#include "fiducials/Character.h"
#include "fiducials/Double.h"
#include "fiducials/File.h"
#include "fiducials/Fiducials.h"
#include "fiducials/Float.h"
#include "fiducials/High_GUI2.h"
#include "fiducials/Integer.h"
#include "fiducials/List.h"
#include "fiducials/Logical.h"
#include "fiducials/String.h"
#include "fiducials/Unsigned.h"

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

void Rviz__tag_announce(void *rviz, Integer id,
  Double x, Double y, Double z, Double twist, Double dx, Double dy, Double dz) {
    ROS_INFO("Rviz__tag_announce:id=%d x=%f y=%f twist=%f\n",
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

    marker.lifetime = ros::Duration();

    marker_pub->publish(marker);
}

void Rviz__location_announce(void *rviz, Integer id,
  Double x, Double y, Double z, Double bearing) {
    ROS_INFO("Rviz__location_announce:id=%d x=%f y=%f bearing=%f\n",
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

    marker_pub = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("fiducials", 1));

    tf_pub = new tf::TransformBroadcaster();


    assert(gettimeofday(start_time_value, (struct timezone *)0) == 0);

    std::list<std::string> image_file_names;
    std::string lens_calibration_file;

    nh.getParam("lens_calibration", lens_calibration_file);

    for( int i=1; i<argc; ++i ) {
      image_file_names.push_back(argv[i]);
      ROS_INFO("Image file: %s", argv[i]);
    }

    int size = image_file_names.size();
    if (size > 0) {
        std::string image_file_name0 = image_file_names.front();
        CV_Image image = (CV_Image)0;
        image = CV_Image__pnm_read(image_file_name0.c_str());
        assert (image != (CV_Image)0);
        Fiducials fiducials =
          Fiducials__create(image, lens_calibration_file.c_str(),
          NULL, Rviz__location_announce, Rviz__tag_announce);
        Fiducials__tag_heights_xml_read(fiducials, "Tag_Heights.xml");

        for( std::list<std::string>::const_iterator itr = 
            image_file_names.begin() ; itr != image_file_names.end(); ++itr) {
            image = CV_Image__pnm_read(itr->c_str());
            Fiducials__image_set(fiducials, image);
            Fiducials__process(fiducials);
            sleep(1);
        }

        assert (gettimeofday(end_time_value, (struct timezone *)0) == 0);

        Double start_time = (Double)start_time_value->tv_usec / 1000000.0;
        Double end_time =
          (Double)(end_time_value->tv_sec - start_time_value->tv_sec) +
          (Double)end_time_value->tv_usec / 1000000.0;
        Double time = end_time - start_time;
        Double frames_per_second = (Double)size / time;

        ROS_INFO("%d frames / %f sec = %f Frame/sec\n", size, time,
            frames_per_second);

        if (size == 1) {
            Fiducials__image_show(fiducials, (Logical)1);
        } else {
            Map map = fiducials->map;
            Map__save(map, "Rviz_Demo.xml");
            List /*<Location>*/ locations = fiducials->locations;
            ROS_INFO("Outputing %d locations\n", List__size(locations));
            Map__svg_write(map, "Demo", locations);
        }
    }


    return 0;
}

