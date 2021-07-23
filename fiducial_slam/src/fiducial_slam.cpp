/*
 * Copyright (c) 2017-20, Ubiquity Robotics
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

#include <fiducial_slam/helpers.h>

#include <assert.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialArray.h"
#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"

#include "fiducial_slam/map.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include <list>
#include <string>

using namespace std;
using namespace cv;

class FiducialSlam {
private:
    ros::Subscriber ft_sub;

    bool use_fiducial_area_as_weight;
    double weighting_scale;

    void transformCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg);

public:
    Map fiducialMap;
    int pose_publish_rate;
    FiducialSlam(ros::NodeHandle &nh);
};

void FiducialSlam::transformCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr &msg) {
    vector<Observation> observations;

    for (size_t i = 0; i < msg->transforms.size(); i++) {
        const fiducial_msgs::FiducialTransform &ft = msg->transforms[i];

        tf2::Vector3 tvec(ft.transform.translation.x, ft.transform.translation.y,
                          ft.transform.translation.z);

        tf2::Quaternion q(ft.transform.rotation.x, ft.transform.rotation.y, ft.transform.rotation.z,
                          ft.transform.rotation.w);

        double variance;
        if (use_fiducial_area_as_weight) {
            variance = weighting_scale / ft.fiducial_area;
        } else {
            variance = weighting_scale * ft.object_error;
        }

        Observation obs(ft.fiducial_id, tf2::Stamped<TransformWithVariance>(
                                            TransformWithVariance(ft.transform, variance),
                                            msg->header.stamp, msg->header.frame_id));
        observations.push_back(obs);
    }

    fiducialMap.update(observations, msg->header.stamp);
}

FiducialSlam::FiducialSlam(ros::NodeHandle &nh) : fiducialMap(nh) {

    // If set, use the fiducial area in pixels^2 as an indication of the
    // 'goodness' of it. This will favor fiducials that are close to the
    // camera and center of the image. The reciprical of the area is actually
    // used, in place of reprojection error as the estimate's variance
    nh.param<bool>("use_fiducial_area_as_weight", use_fiducial_area_as_weight, false);
    // Scaling factor for weighing
    nh.param<double>("weighting_scale", weighting_scale, 1e9);
    nh.param<int>("pose_publish_rate", pose_publish_rate, 20);

    ft_sub = nh.subscribe("/fiducial_transforms", 1, &FiducialSlam::transformCallback, this);

    ROS_INFO("Fiducial Slam ready");
}

auto node = unique_ptr<FiducialSlam>(nullptr);

void mySigintHandler(int sig) {
    if (node != nullptr) node->fiducialMap.saveMap();

    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fiducial_slam", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");

    node = make_unique<FiducialSlam>(nh);
    signal(SIGINT, mySigintHandler);

    ros::Rate r(node->pose_publish_rate);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
        node->fiducialMap.update();
    }

    return 0;
}
