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

#include <fiducial_slam/helpers.h>
#include <fiducial_slam/map.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <string>

#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <boost/filesystem.hpp>


static double systematic_error = 0.01;

// Update a fiducial position in map with a new estimate
void Fiducial::update(const tf2::Stamped<TransformWithVariance> &newPose) {
    pose.update(newPose);
    numObs++;
}

// Create a fiducial from an estimate of its position in the map
Fiducial::Fiducial(int id, const tf2::Stamped<TransformWithVariance> &pose) {
    this->id = id;
    this->pose = pose;
    this->lastPublished = rclcpp::Time(0);
    this->numObs = 0;
    this->visible = false;
}

// Constructor for map

Map::Map(rclcpp::Node::SharedPtr &nh)
    : nh_(nh),
    tfBuffer(nh->get_clock(), tf2::Duration(std::chrono::seconds(30))),
    broadcaster(nh)
{
    frameNum = 0;
    initialFrameNum = 0;
    originFid = -1;
    isInitializingMap = false;
    havePose = false;
    fiducialToAdd = -1;

    listener = make_unique<tf2_ros::TransformListener>(tfBuffer);

    robotPosePub = nh->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/fiducial_pose", 1);
    cameraPosePub = nh->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/fiducial_slam/camera_pose", 1);

    markerPub = nh->create_publisher<visualization_msgs::msg::Marker>("/fiducials", 100);
    mapPub = nh->create_publisher<fiducial_msgs::msg::FiducialMapEntryArray>("/fiducial_map", 1);

    clearSrv = nh->create_service<std_srvs::srv::Empty>("clear_map", std::bind(&Map::clearCallback, this, std::placeholders::_1, std::placeholders::_2));
    addSrv = nh->create_service<fiducial_msgs::srv::AddFiducial>("add_fiducial", std::bind(&Map::addFiducialCallback, this, std::placeholders::_1, std::placeholders::_2));

    mapFrame = nh->declare_parameter("map_frame", "map");
    odomFrame = nh->declare_parameter("odom_frame", "odom");
    baseFrame = nh->declare_parameter("base_frame", "base_footprint");

    tfPublishInterval = nh->declare_parameter("tf_publish_interval", 1.0);
    publishPoseTf = nh->declare_parameter("publish_tf", true);
    systematic_error = nh->declare_parameter("systematic_error", 0.01);
    future_date_transforms = nh->declare_parameter("future_date_transforms", 0.1);
    publish_6dof_pose = nh->declare_parameter("publish_6dof_pose", false);
    readOnly = nh->declare_parameter("read_only_map", false);

    std::fill(covarianceDiagonal.begin(), covarianceDiagonal.end(), 0);

    overridePublishedCovariance = nh->get_parameter("covariance_diagonal", covarianceDiagonal);
    if (overridePublishedCovariance) {
        if (covarianceDiagonal.size() != 6) {
            RCLCPP_WARN(nh->get_logger(), "ignoring covariance_diagonal because it has %ld elements, not 6", covarianceDiagonal.size());
            overridePublishedCovariance = false;
        }
        // Check to make sure that the diagonal is non-zero
        for (auto variance : covarianceDiagonal) {
            if (variance == 0) {
                RCLCPP_WARN(nh->get_logger(), "ignoring covariance_diagonal because it has 0 values");
                std::fill(covarianceDiagonal.begin(), covarianceDiagonal.end(), 0);
                break;
            }
        }
    }

    // threshold of object error for using multi-fidicial pose
    // set -ve to never use
    multiErrorThreshold = nh->declare_parameter("multi_error_theshold", -1.0);
    mapFilename = nh->declare_parameter("map_file", std::string(getenv("HOME")) + "/.ros/slam/map.txt");


    boost::filesystem::path mapPath(mapFilename);
    boost::filesystem::path dir = mapPath.parent_path();
    boost::filesystem::create_directories(dir);

    std::string initialMap;
    initialMap = nh->declare_parameter("initial_map_file", "");

    if (!initialMap.empty()) {
        loadMap(initialMap);
    } else {
        loadMap();
    }

    publishMarkers();
}

// Update map with a set of observations

void Map::update(std::vector<Observation> &obs, const rclcpp::Time &time) {
    RCLCPP_INFO(nh_->get_logger(), "Updating map with %d observations. Map has %d fiducials", (int)obs.size(),
             (int)fiducials.size());

    frameNum++;

    if (obs.size() > 0 && fiducials.size() == 0) {
        isInitializingMap = true;
    }

    if (isInitializingMap) {
        autoInit(obs, time);
    } else {
        tf2::Stamped<TransformWithVariance> T_mapCam;
        T_mapCam.frame_id_ = mapFrame;

        if (updatePose(obs, time, T_mapCam) > 0 && obs.size() > 1 && !readOnly) {
            updateMap(obs, time, T_mapCam);
        }
    }

    handleAddFiducial(obs);

    publishMap();
}

// update estimates of observed fiducials from previously estimated
// camera pose

void Map::updateMap(const std::vector<Observation> &obs, const rclcpp::Time &time,
                    const tf2::Stamped<TransformWithVariance> &T_mapCam) {
    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        f.visible = false;
    }

    for (const Observation &o : obs) {
        // This should take into account the variances from both
        tf2::Stamped<TransformWithVariance> T_mapFid = T_mapCam * o.T_camFid;
        T_mapFid.frame_id_ = mapFrame;

        // New scope for logging vars
        {
            tf2::Vector3 trans = T_mapFid.transform.getOrigin();

            RCLCPP_INFO(nh_->get_logger(), "Estimate of %d %lf %lf %lf var %lf %lf", o.fid, trans.x(), trans.y(),
                     trans.z(), o.T_camFid.variance, T_mapFid.variance);

            if (std::isnan(trans.x()) || std::isnan(trans.y()) || std::isnan(trans.z())) {
                RCLCPP_WARN(nh_->get_logger(), "Skipping NAN estimate\n");
                continue;
            };
        }

        if (fiducials.find(o.fid) == fiducials.end()) {
            RCLCPP_INFO(nh_->get_logger(), "New fiducial %d", o.fid);
            fiducials[o.fid] = Fiducial(o.fid, T_mapFid);
        }
        Fiducial &f = fiducials[o.fid];
        f.visible = true;
        if (f.pose.variance != 0) {
            f.update(T_mapFid);
            f.numObs++;
        }

        for (const Observation &observation : obs) {
            int fid = observation.fid;
            if (f.id != fid) {
                f.links.insert(fid);
            }
        }
        publishMarker(fiducials[o.fid]);
    }
}

// lookup specified transform

bool Map::lookupTransform(const std::string &from, const std::string &to, const rclcpp::Time &time,
                          tf2::Transform &T) const {
    geometry_msgs::msg::TransformStamped transform;

    try {
        transform = tfBuffer.lookupTransform(from, to, time, rclcpp::Duration(0.5));

        tf2::fromMsg(transform.transform, T);
        return true;
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
        return false;
    }
}

// update pose estimate of robot.  We combine the camera->base_link
// tf to each estimate so we can evaluate how good they are.  A good
// estimate would have z == roll == pitch == 0.
int Map::updatePose(std::vector<Observation> &obs, const rclcpp::Time &time,
                    tf2::Stamped<TransformWithVariance> &T_mapCam) {
    int numEsts = 0;
    tf2::Stamped<TransformWithVariance> T_camBase;
    tf2::Stamped<TransformWithVariance> T_baseCam;
    tf2::Stamped<TransformWithVariance> T_mapBase;

    if (obs.size() == 0) {
        return 0;
    }

    if (lookupTransform(obs[0].T_camFid.frame_id_, baseFrame, time, T_camBase.transform)) {
        tf2::Vector3 c = T_camBase.transform.getOrigin();
        RCLCPP_INFO(nh_->get_logger(), "camera->base   %lf %lf %lf", c.x(), c.y(), c.z());
        T_camBase.variance = 1.0;
    } else {
        RCLCPP_ERROR(nh_->get_logger(), "Cannot determine tf from camera to robot\n");
    }

    if (lookupTransform(baseFrame, obs[0].T_camFid.frame_id_, time, T_baseCam.transform)) {
        tf2::Vector3 c = T_baseCam.transform.getOrigin();
        RCLCPP_INFO(nh_->get_logger(), "base->camera   %lf %lf %lf", c.x(), c.y(), c.z());
        T_baseCam.variance = 1.0;
    } else {
        RCLCPP_ERROR(nh_->get_logger(), "Cannot determine tf from robot to camera\n");
        return numEsts;
    }

    for (Observation &o : obs) {
        if (fiducials.find(o.fid) != fiducials.end()) {
            const Fiducial &fid = fiducials[o.fid];

            tf2::Stamped<TransformWithVariance> p = fid.pose * o.T_fidCam;

            p.frame_id_ = mapFrame;
            p.stamp_ = o.T_fidCam.stamp_;

            p.setData(p * T_camBase);
            auto position = p.transform.getOrigin();
            double roll, pitch, yaw;
            p.transform.getBasis().getRPY(roll, pitch, yaw);

            // Create variance according to how well the robot is upright on the ground
            // TODO: Create variance for each DOF
            // TODO: Take into account position according to odom
            auto cam_f = o.T_camFid.transform.getOrigin();
            double s1 = std::pow(position.z() / cam_f.z(), 2) *
                        (std::pow(cam_f.x(), 2) + std::pow(cam_f.y(), 2));
            double s2 = position.length2() * std::pow(std::sin(roll), 2);
            double s3 = position.length2() * std::pow(std::sin(pitch), 2);
            p.variance = s1 + s2 + s3 + systematic_error;
            o.T_camFid.variance = p.variance;

            RCLCPP_INFO(nh_->get_logger(), "Pose %d %lf %lf %lf %lf %lf %lf %lf", o.fid, position.x(), position.y(),
                     position.z(), roll, pitch, yaw, p.variance);

            // drawLine(fid.pose.getOrigin(), o.position);

            if (std::isnan(position.x()) || std::isnan(position.y()) || std::isnan(position.z())) {
                RCLCPP_WARN(nh_->get_logger(), "Skipping NAN estimate\n");
                continue;
            };

            // compute base_link pose based on this estimate

            if (numEsts == 0) {
                T_mapBase = p;
            } else {
                T_mapBase.setData(averageTransforms(T_mapBase, p));
                T_mapBase.stamp_ = p.stamp_;
            }
            numEsts++;
        }
    }

    if (numEsts == 0) {
        RCLCPP_INFO(nh_->get_logger(), "Finished frame - no estimates\n");
        return numEsts;
    }

    // New scope for logging vars
    {
        tf2::Vector3 trans = T_mapBase.transform.getOrigin();
        double r, p, y;
        T_mapBase.transform.getBasis().getRPY(r, p, y);

        RCLCPP_INFO(nh_->get_logger(), "Pose ALL %lf %lf %lf %lf %lf %lf %f", trans.x(), trans.y(), trans.z(), r, p, y,
                 T_mapBase.variance);
    }

    tf2::Stamped<TransformWithVariance> basePose = T_mapBase;
    basePose.frame_id_ = mapFrame;
    auto robotPose = toPose(basePose);

    if (overridePublishedCovariance) {
        for (int i = 0; i <= 5; i++) {
            robotPose.pose.covariance[i * 6 + i] = covarianceDiagonal[i];  // Fill the diagonal
        }
    }

    T_mapCam = T_mapBase * T_baseCam;

    robotPosePub->publish(robotPose);

    tf2::Stamped<TransformWithVariance> outPose = basePose;
    outPose.frame_id_ = mapFrame;
    std::string outFrame = baseFrame;

    if (!odomFrame.empty()) {
        tf2::Transform odomTransform;
        if (lookupTransform(odomFrame, baseFrame, tf2_ros::toMsg(outPose.stamp_), odomTransform)) {
            outPose.setData(basePose * odomTransform.inverse());
            outFrame = odomFrame;

            tf2::Vector3 c = odomTransform.getOrigin();
            RCLCPP_INFO(nh_->get_logger(), "odom   %lf %lf %lf", c.x(), c.y(), c.z());
        }
        else {
            // Don't publish anything if map->odom was requested and is unavailaable
            return numEsts;
        }
    }

    // Make outgoing transform make sense - ie only consist of x, y, yaw
    // This can be disabled via the publish_6dof_pose param, mainly for debugging
    if (!publish_6dof_pose) {
        tf2::Vector3 translation = outPose.transform.getOrigin();
        translation.setZ(0);
        outPose.transform.setOrigin(translation);
        double roll, pitch, yaw;
        outPose.transform.getBasis().getRPY(roll, pitch, yaw);
        outPose.transform.getBasis().setRPY(0, 0, yaw);
    }

    poseTf = toMsg(outPose);
    poseTf.child_frame_id = outFrame;
    havePose = true;

    if (publishPoseTf) {
        publishTf();
    }

    RCLCPP_INFO(nh_->get_logger(), "Finished frame. Estimates %d\n", numEsts);
    return numEsts;
}

// Publish map -> odom tf

void Map::publishTf() {
    tfPublishTime = nh_->get_clock()->now();
    poseTf.header.stamp = tfPublishTime + rclcpp::Duration(future_date_transforms);
    broadcaster.sendTransform(poseTf);
}

// publish latest tf if enough time has elapsed

void Map::update() {
    rclcpp::Time now = nh_->get_clock()->now();
    if (publishPoseTf && havePose && tfPublishInterval != 0.0 &&
        (now - tfPublishTime).nanoseconds() > tfPublishInterval * 10e9) {
        publishTf();
        tfPublishTime = now;
    }
    publishMarkers();
}

// Find closest fiducial to camera

static int findClosestObs(const std::vector<Observation> &obs) {
    double smallestDist = -1;
    int closestIdx = -1;

    for (size_t i = 0; i < obs.size(); i++) {
        const Observation &o = obs[i];
        double d = o.T_camFid.transform.getOrigin().length2();
        if (smallestDist < 0 || d < smallestDist) {
            smallestDist = d;
            closestIdx = i;
        }
    }

    return closestIdx;
}

// Initialize a map from the closest observed fiducial
// Figure out the closest marker, and then figure out the
// pose of that marker such that base_link is at the origin of the
// map frame

void Map::autoInit(const std::vector<Observation> &obs, const rclcpp::Time &time) {
    RCLCPP_INFO(nh_->get_logger(), "Auto init map %d", frameNum);

    tf2::Transform T_baseCam;

    if (fiducials.size() == 0) {
        int idx = findClosestObs(obs);

        if (idx == -1) {
	     RCLCPP_WARN(nh_->get_logger(), "Could not find a fiducial to initialize map from");
	     return;
        }
        const Observation &o = obs[idx];
        originFid = o.fid;

        RCLCPP_INFO(nh_->get_logger(), "Initializing map from fiducial %d", o.fid);

        tf2::Stamped<TransformWithVariance> T = o.T_camFid;

        if (lookupTransform(baseFrame, o.T_camFid.frame_id_, tf2_ros::toMsg(o.T_camFid.stamp_), T_baseCam)) {
            T.setData(T_baseCam * T);
        }

        fiducials[o.fid] = Fiducial(o.fid, T);
    } else {
        for (const Observation &o : obs) {
            if (o.fid == originFid) {
                tf2::Stamped<TransformWithVariance> T = o.T_camFid;

                tf2::Vector3 trans = T.transform.getOrigin();
                RCLCPP_INFO(nh_->get_logger(), "Estimate of %d from base %lf %lf %lf err %lf", o.fid, trans.x(),
                         trans.y(), trans.z(), o.T_camFid.variance);

                if (lookupTransform(baseFrame, o.T_camFid.frame_id_, tf2_ros::toMsg(o.T_camFid.stamp_),
                                    T_baseCam)) {
                    T.setData(T_baseCam * T);
                }

                fiducials[originFid].update(T);
                break;
            }
        }
    }

    if (frameNum - initialFrameNum > 10 && originFid != -1) {
        isInitializingMap = false;

        fiducials[originFid].pose.variance = 0.0;
    }
}

// Attempt to add the specified fiducial to the map

void Map::handleAddFiducial(const std::vector<Observation> &obs) {

    if (fiducialToAdd == -1) {
        return;
    }

    if (fiducials.find(fiducialToAdd) != fiducials.end()) {
        RCLCPP_INFO(nh_->get_logger(), "Fiducial %d is already in map - ignoring add request",
                 fiducialToAdd);
        fiducialToAdd = -1;
        return;
    }

    for (const Observation &o : obs) {
        if (o.fid == fiducialToAdd) {
            RCLCPP_INFO(nh_->get_logger(), "Adding fiducial_id %d to map", fiducialToAdd);


            tf2::Stamped<TransformWithVariance> T = o.T_camFid;

            // Take into account position of camera on base
            tf2::Transform T_baseCam;
            if (lookupTransform(baseFrame, o.T_camFid.frame_id_,
                                tf2_ros::toMsg(o.T_camFid.stamp_), T_baseCam)) {
                T.setData(T_baseCam * T);
            }

            // Take into account position of robot in the world if known
            tf2::Transform T_mapBase;
            if (lookupTransform(mapFrame, baseFrame, rclcpp::Time(0), T_mapBase)) {
                T.setData(T_mapBase * T);
            }
            else {
                RCLCPP_INFO(nh_->get_logger(), "Placing robot at the origin");
            }

            fiducials[o.fid] = Fiducial(o.fid, T);
            fiducials[originFid].pose.variance = 0.0;
            isInitializingMap = false;

            fiducialToAdd = -1;
            return;
        }
    }

    RCLCPP_INFO(nh_->get_logger(), "Unable to add fiducial %d to map", fiducialToAdd);
}

// save map to file

bool Map::saveMap() { return saveMap(mapFilename); }

bool Map::saveMap(std::string filename) {
    RCLCPP_INFO(nh_->get_logger(), "Saving map with %d fiducials to file %s\n", (int)fiducials.size(), filename.c_str());

    FILE *fp = fopen(filename.c_str(), "w");
    if (fp == NULL) {
        RCLCPP_WARN(nh_->get_logger(), "Could not open %s for write\n", filename.c_str());
        return false;
    }

    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        tf2::Vector3 trans = f.pose.transform.getOrigin();
        double rx, ry, rz;
        f.pose.transform.getBasis().getRPY(rx, ry, rz);

        fprintf(fp, "%d %lf %lf %lf %lf %lf %lf %lf %d", f.id, trans.x(), trans.y(), trans.z(),
                rad2deg(rx), rad2deg(ry), rad2deg(rz), f.pose.variance, f.numObs);

        for (const auto linked_fid : f.links) {
            fprintf(fp, " %d", linked_fid);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    return true;
}

// Load map from file

bool Map::loadMap() { return loadMap(mapFilename); }

bool Map::loadMap(std::string filename) {
    int numRead = 0;

    RCLCPP_INFO(nh_->get_logger(), "Load map %s", filename.c_str());

    FILE *fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        RCLCPP_WARN(nh_->get_logger(), "Could not open %s for read\n", filename.c_str());
        return false;
    }

    const int BUFSIZE = 2048;
    char linebuf[BUFSIZE];
    char linkbuf[BUFSIZE];

    while (!feof(fp)) {
        if (fgets(linebuf, BUFSIZE - 1, fp) == NULL) break;

        int id;
        double tx, ty, tz, rx, ry, rz, var;
        int numObs = 0;

        linkbuf[0] = '\0';
        int nElems = sscanf(linebuf, "%d %lf %lf %lf %lf %lf %lf %lf %d%[^\t\n]*s", &id, &tx, &ty,
                            &tz, &rx, &ry, &rz, &var, &numObs, linkbuf);
        if (nElems == 9 || nElems == 10) {
            tf2::Vector3 tvec(tx, ty, tz);
            tf2::Quaternion q;
            q.setRPY(deg2rad(rx), deg2rad(ry), deg2rad(rz));

            auto twv = TransformWithVariance(tvec, q, var);
            // TODO: figure out what the timestamp in Fiducial should be
            Fiducial f =
                Fiducial(id, tf2::Stamped<TransformWithVariance>(twv, tf2_ros::fromMsg(nh_->get_clock()->now()), mapFrame));
            f.numObs = numObs;

            std::istringstream ss(linkbuf);
            std::string s;
            while (getline(ss, s, ' ')) {
                if (!s.empty()) {
                    f.links.insert(stoi(s));
                }
            }
            fiducials[id] = f;
            numRead++;
        } else {
            RCLCPP_WARN(nh_->get_logger(), "Invalid line: %s", linebuf);
        }
    }

    fclose(fp);
    RCLCPP_INFO(nh_->get_logger(), "Load map %s read %d entries", filename.c_str(), numRead);
    return true;
}

// Publish the map

void Map::publishMap() {
    fiducial_msgs::msg::FiducialMapEntryArray fmea;
    std::map<int, Fiducial>::iterator it;

    for (const auto &map_pair : fiducials) {
        const Fiducial &f = map_pair.second;

        fiducial_msgs::msg::FiducialMapEntry fme;
        fme.fiducial_id = f.id;

        tf2::Vector3 t = f.pose.transform.getOrigin();
        fme.x = t.x();
        fme.y = t.y();
        fme.z = t.z();

        double rx, ry, rz;
        f.pose.transform.getBasis().getRPY(rx, ry, rz);
        fme.rx = rx;
        fme.ry = ry;
        fme.rz = rz;

        fmea.fiducials.push_back(fme);
    }

    mapPub->publish(fmea);
}

// Publish the next marker visualization messages that hasn't been
// published recently

void Map::publishMarkers() {
    rclcpp::Time now = nh_->get_clock()->now();
    std::map<int, Fiducial>::iterator it;

    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        if ((now.nanoseconds() - f.lastPublished.nanoseconds()) > 1.0 * 10e9) {
            publishMarker(f);
        }
    }
}

// Publish visualization messages for a single fiducial

void Map::publishMarker(Fiducial &fid) {
    fid.lastPublished = nh_->get_clock()->now();

    // Flattened cube
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    toMsg(fid.pose.transform, marker.pose);

    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.01;
    if (fid.visible) {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
    } else {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
    }
    marker.id = fid.id;
    marker.ns = "fiducial";
    marker.header.frame_id = mapFrame;
    markerPub->publish(marker);

    // cylinder scaled by stddev
    visualization_msgs::msg::Marker cylinder;
    cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
    cylinder.action = visualization_msgs::msg::Marker::ADD;
    cylinder.header.frame_id = mapFrame;
    cylinder.color.r = 0.0f;
    cylinder.color.g = 0.0f;
    cylinder.color.b = 1.0f;
    cylinder.color.a = 0.5f;
    cylinder.id = fid.id + 10000;
    cylinder.ns = "sigma";
    cylinder.scale.x = cylinder.scale.y = std::max(std::sqrt(fid.pose.variance), 0.1);
    cylinder.scale.z = 0.01;
    cylinder.pose.position.x = marker.pose.position.x;
    cylinder.pose.position.y = marker.pose.position.y;
    cylinder.pose.position.z = marker.pose.position.z;
    cylinder.pose.position.z += (marker.scale.z / 2.0) + 0.05;
    markerPub->publish(cylinder);

    // Text
    visualization_msgs::msg::Marker text;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.header.frame_id = mapFrame;
    text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
    text.id = fid.id;
    text.scale.x = text.scale.y = text.scale.z = 0.1;
    text.pose.position.x = marker.pose.position.x;
    text.pose.position.y = marker.pose.position.y;
    text.pose.position.z = marker.pose.position.z;
    text.pose.position.z += (marker.scale.z / 2.0) + 0.1;
    text.id = fid.id + 30000;
    text.ns = "text";
    text.text = std::to_string(fid.id);
    markerPub->publish(text);

    // Links
    visualization_msgs::msg::Marker links;
    links.type = visualization_msgs::msg::Marker::LINE_LIST;
    links.action = visualization_msgs::msg::Marker::ADD;
    links.header.frame_id = mapFrame;
    links.color.r = 0.0f;
    links.color.g = 0.0f;
    links.color.b = 1.0f;
    links.color.a = 1.0f;
    links.id = fid.id + 40000;
    links.ns = "links";
    links.scale.x = links.scale.y = links.scale.z = 0.02;
    links.pose.position.x = 0;
    links.pose.position.y = 0;
    links.pose.position.z = 0;

    geometry_msgs::msg::Point gp0, gp1;
    tf2::Vector3 p0 = fid.pose.transform.getOrigin();
    gp0.x = p0.x();
    gp0.y = p0.y();
    gp0.z = p0.z();

    std::map<int, int>::iterator lit;
    for (const auto linked_fid : fid.links) {
        // only draw links in one direction
        if (fid.id < linked_fid) {
            if (fiducials.find(linked_fid) != fiducials.end()) {
                tf2::Vector3 p1 = fiducials[linked_fid].pose.transform.getOrigin();
                gp1.x = p1.x();
                gp1.y = p1.y();
                gp1.z = p1.z();
                links.points.push_back(gp0);
                links.points.push_back(gp1);
            }
        }
    }

    markerPub->publish(links);
}

// Publish a line marker between two points

void Map::drawLine(const tf2::Vector3 &p0, const tf2::Vector3 &p1) {
    static int lid = 60000;
    visualization_msgs::msg::Marker line;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.header.frame_id = mapFrame;
    line.color.r = 1.0f;
    line.color.g = 0.0f;
    line.color.b = 0.0f;
    line.color.a = 1.0f;
    line.id = lid++;
    line.ns = "lines";
    line.scale.x = line.scale.y = line.scale.z = 0.01;
    line.pose.position.x = 0;
    line.pose.position.y = 0;
    geometry_msgs::msg::Point gp0, gp1;
    gp0.x = p0.x();
    gp0.y = p0.y();
    gp0.z = p0.z();
    gp1.x = p1.x();
    gp1.y = p1.y();
    gp1.z = p1.z();
    line.points.push_back(gp0);
    line.points.push_back(gp1);

    markerPub->publish(line);
}

// Service to clear the map and enable auto initialization

bool Map::clearCallback(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res) {
    RCLCPP_INFO(nh_->get_logger(), "Clearing fiducial map from service call");

    fiducials.clear();
    initialFrameNum = frameNum;
    originFid = -1;

    return true;
}

// Service to add a fiducial to the map

bool Map::addFiducialCallback(const fiducial_msgs::srv::AddFiducial::Request::SharedPtr req,
                              fiducial_msgs::srv::AddFiducial::Response::SharedPtr res)
{
   RCLCPP_INFO(nh_->get_logger(), "Request to add fiducial %d to map", req->fiducial_id);
   fiducialToAdd = req->fiducial_id;

   return true;
}
