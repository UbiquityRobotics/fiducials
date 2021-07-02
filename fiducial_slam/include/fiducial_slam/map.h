/*
 * Copyright (c) 2017-9, Ubiquity Robotics
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
#ifndef MAP_H
#define MAP_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include "fiducial_msgs/msg/fiducial_map_entry.hpp"
#include "fiducial_msgs/msg/fiducial_map_entry_array.hpp"

#include <list>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/srv/empty.hpp>
#include "fiducial_msgs/srv/add_fiducial.hpp"

#include "fiducial_slam/transform_with_variance.h"
#include "fiducial_slam/observation.h"

// A single fiducial that is in the map
class Fiducial {
public:
    int id;
    int numObs;
    bool visible;
    std::set<int> links;  // Stores the IDs of connected fiducials

    tf2::Stamped<TransformWithVariance> pose;
    rclcpp::Time lastPublished;

    void update(const tf2::Stamped<TransformWithVariance> &newPose);

    Fiducial() {}

    Fiducial(int id, const tf2::Stamped<TransformWithVariance> &pose);
};

// Class containing map data
class Map {
public:
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> listener;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;
    rclcpp::Publisher<fiducial_msgs::msg::FiducialMapEntryArray>::SharedPtr mapPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robotPosePub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr cameraPosePub;


    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearSrv;
    rclcpp::Service<fiducial_msgs::srv::AddFiducial>::SharedPtr addSrv;
    bool clearCallback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);
    bool addFiducialCallback(const fiducial_msgs::srv::AddFiducial::Request::SharedPtr req,
                             fiducial_msgs::srv::AddFiducial::Response::SharedPtr res);

    std::string mapFilename;
    std::string mapFrame;
    std::string odomFrame;
    std::string cameraFrame;
    std::string baseFrame;
    double future_date_transforms;
    bool publish_6dof_pose;
    double multiErrorThreshold;

    bool isInitializingMap;
    bool readOnly;
    int frameNum;
    int initialFrameNum;
    int originFid;

    bool overridePublishedCovariance;
    std::vector<double> covarianceDiagonal;

    bool havePose;
    float tfPublishInterval;
    bool publishPoseTf;
    rclcpp::Time tfPublishTime;
    geometry_msgs::msg::TransformStamped poseTf;

    std::map<int, Fiducial> fiducials;
    int fiducialToAdd;

    rclcpp::Node::SharedPtr nh_;

    Map(rclcpp::Node::SharedPtr &nh);
    void update();
    void update(std::vector<Observation> &obs, const rclcpp::Time &time);
    void autoInit(const std::vector<Observation> &obs, const rclcpp::Time &time);
    int updatePose(std::vector<Observation> &obs, const rclcpp::Time &time,
                   tf2::Stamped<TransformWithVariance> &cameraPose);
    void updateMap(const std::vector<Observation> &obs, const rclcpp::Time &time,
                   const tf2::Stamped<TransformWithVariance> &cameraPose);
    void handleAddFiducial(const std::vector<Observation> &obs);

    bool loadMap();
    bool loadMap(std::string filename);
    bool saveMap();
    bool saveMap(std::string filename);

    void publishTf();
    void publishMap();
    void publishMarker(Fiducial &fid);
    void publishMarkers();
    void drawLine(const tf2::Vector3 &p0, const tf2::Vector3 &p1);

    bool lookupTransform(const std::string &from, const std::string &to, const rclcpp::Time &time,
                         tf2::Transform &T) const;
};

#endif
