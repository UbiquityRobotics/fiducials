// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

#include <assert.h>
#include <sys/time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/tf.h>
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

#include "fiducials_rviz.h"

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

ros::Publisher marker_pub;

void Rviz__tag_announce(void *rviz, Integer id,
  Double x, Double y, Double z, Double twist, Double dx, Double dy, Double dz) {
    ROS_INFO("Rviz__tag_announce:id=%d x=%f y=%f twist=%f\n",
      id, x, y, twist);
    Double scale = 100.0;
    // TODO: publish directly to marker_pub
    sendMarker(rviz, "fiducial_frame", id, x / scale, y / scale, z / scale,
      0.0, dx / scale, dy / scale, dz / scale);
}

void Rviz__location_announce(void *rviz, Integer id,
  Double x, Double y, Double z, Double bearing) {
    ROS_INFO("Rviz__location_announce:id=%d x=%f y=%f bearing=%f\n",
      id, x, y, bearing * 180. / 3.1415926);
    Double scale = 100.0;
    // TODO: publish directly to marker_pub
    sendArrow(rviz, "fiducial_frame", id, x / scale, y / scale, z / scale,
        0.0, .2, .05, .05, bearing);
}

int main(int argc, char ** argv) {
    struct timeval start_time_value_struct;    
    struct timeval end_time_value_struct;    
    struct timeval difference_time_value_struct;    
    Time_Value start_time_value = &start_time_value_struct;
    Time_Value end_time_value = &end_time_value_struct;
    Time_Value difference_time_value = &difference_time_value_struct;

    ros::init(argc, argv, "fiducials_localization");
    ros::NodeHandle nh("~");

    assert(gettimeofday(start_time_value, (struct timezone *)0) == 0);

    std::list<std::string> image_file_names;
    std::string lens_calibration_file;

    nh.getParam("lens_calibration", lens_calibration_file);

    for( int i=0; i<argc; ++i ) {
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
            sleep(2);
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

