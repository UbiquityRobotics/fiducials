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

#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>


#include <boost/filesystem.hpp>

// For Nav2D usage of openCV  estimateRigidTransform()
#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

static double systematic_error = 0.01;

// Constructor for observation
Observation::Observation(int fid, const tf2::Stamped<TransformWithVariance> &camFid) {
    this->fid = fid;

    tf2::Stamped<TransformWithVariance> camFidFix = camFid;
    // test fix
    double r, p, y;
    camFidFix.transform.getBasis().getRPY(r, p, y);
    tf2::Quaternion filRot = camFidFix.transform.getRotation();
    filRot.setEuler(0, M_PI, y);
    camFidFix.transform.setRotation(filRot);
    // ---

    tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped ts = toMsg(camFidFix);
    ts.child_frame_id = "fid" + std::to_string(fid);
    broadcaster.sendTransform(ts);

    T_camFid = camFidFix;
    T_fidCam = T_camFid;
    T_fidCam.transform = T_camFid.transform.inverse();
}


// Update a fiducial position in map with a new estimate
void Fiducial::update(const tf2::Stamped<TransformWithVariance> &newPose) {
    pose.update(newPose);
    numObs++;
}

// Create a fiducial from an estimate of its position in the map
Fiducial::Fiducial(int id, const tf2::Stamped<TransformWithVariance> &pose) {
    this->id = id;
    this->pose = pose;
    this->lastPublished = ros::Time(0);
    this->numObs = 0;
    this->visible = false;
}

// Constructor for map

Map::Map(ros::NodeHandle &nh) : tfBuffer(ros::Duration(30.0)) {
    frameNum = 0;
    initialFrameNum = 0;
    originFid = -1;
    isInitializingMap = false;
    havePose = false;
    fiducialToAdd = -1;
    verboseInfo = true;

    listener = make_unique<tf2_ros::TransformListener>(tfBuffer);

    robotPosePub =
        ros::Publisher(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/fiducial_pose", 1));
    cameraPosePub = ros::Publisher(
        nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/fiducial_slam/camera_pose", 1));

    markerPub = ros::Publisher(nh.advertise<visualization_msgs::Marker>("/fiducials", 100));
    mapPub = ros::Publisher(nh.advertise<fiducial_msgs::FiducialMapEntryArray>("/fiducial_map", 1));

    clearSrv = nh.advertiseService("clear_map", &Map::clearCallback, this);
    addSrv = nh.advertiseService("add_fiducial", &Map::addFiducialCallback, this);

    nh.param<std::string>("map_frame", mapFrame, "map");
    nh.param<std::string>("odom_frame", odomFrame, "odom");
    nh.param<std::string>("base_frame", baseFrame, "base_footprint");

    nh.param<float>("tf_publish_interval", tfPublishInterval, 1.0);
    nh.param<bool>("publish_tf", publishPoseTf, true);
    nh.param<double>("systematic_error", systematic_error, 0.01);
    nh.param<double>("future_date_transforms", future_date_transforms, 0.1);
    nh.param<bool>("publish_6dof_pose", publish_6dof_pose, false);
    nh.param<bool>("read_only_map", readOnly, false);

    // navigateFlat when true uses Nav2D pose for robotPose when in read_only_map mode
    // Nav2D projects fiducials to the floor and solved for translation and 2D rotation
    nh.param<bool>("navigate_flat", navigateFlat, false);
    if (navigateFlat) {
        ROS_INFO("Map: Forceing read_only_map mode because we are in navigate_flat mode");
        readOnly = true;   // Force read_only_map mode when in navigate_flat
    }
    nh.param<bool>("verbose_info", verboseInfo, true);

    std::fill(covarianceDiagonal.begin(), covarianceDiagonal.end(), 0);
    overridePublishedCovariance = nh.getParam("covariance_diagonal", covarianceDiagonal);
    if (overridePublishedCovariance) {
        // Check to make sure that the diagonal is non-zero
        for (auto variance : covarianceDiagonal) {
            if (variance = 0) {
                ROS_WARN("ignoring covariance_diagonal because it has 0 values");
                std::fill(covarianceDiagonal.begin(), covarianceDiagonal.end(), 0);
                break;
            }
        }
    }

    // threshold of object error for using multi-fidicial pose
    // set -ve to never use
    nh.param<double>("multi_error_theshold", multiErrorThreshold, -1);

    nh.param<std::string>("map_file", mapFilename,
                          std::string(getenv("HOME")) + "/.ros/slam/map.txt");

    boost::filesystem::path mapPath(mapFilename);
    boost::filesystem::path dir = mapPath.parent_path();
    boost::filesystem::create_directories(dir);

    std::string initialMap;
    nh.param<std::string>("initial_map_file", initialMap, "");

    if (!initialMap.empty()) {
        loadMap(initialMap);
    } else {
        loadMap();
    }

    publishMarkers();
}

// Update map with a set of observations

void Map::update(std::vector<Observation> &obs, const ros::Time &time) {
    if (!readOnly) {
        ROS_INFO("Map: Updating pose and map with %d observations. Map has %d fids. autoInit fid %d",
            (int)obs.size(), (int)fiducials.size(), originFid);
    } else {
        ROS_INFO("Map: Updating pose but not map with %d observations. Map has %d fids. autoInit fid %d",
            (int)obs.size(), (int)fiducials.size(), originFid);
    }

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
    ROS_INFO("Map: Done Map::update");

    publishMap();
}

// update estimates of observed fiducials from previously estimated
// camera pose

void Map::updateMap(const std::vector<Observation> &obs, const ros::Time &time,
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

            if (verboseInfo) {
                ROS_INFO("Map: Estimate of %d %lf %lf %lf var %lf %lf", o.fid, trans.x(), trans.y(),
                     trans.z(), o.T_camFid.variance, T_mapFid.variance);
            }

            if (std::isnan(trans.x()) || std::isnan(trans.y()) || std::isnan(trans.z())) {
                ROS_WARN("Skipping NAN estimate\n");
                continue;
            };
        }

        if (fiducials.find(o.fid) == fiducials.end()) {
            ROS_INFO("New fiducial %d", o.fid);
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

bool Map::lookupTransform(const std::string &from, const std::string &to, const ros::Time &time,
                          tf2::Transform &T) const {
    geometry_msgs::TransformStamped transform;

    try {
        transform = tfBuffer.lookupTransform(from, to, time);

        tf2::fromMsg(transform.transform, T);
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }
}

// update pose estimate of robot.  We combine the camera->base_link
// tf to each estimate so we can evaluate how good they are.  A good
// estimate would have z == roll == pitch == 0.
int Map::updatePose(std::vector<Observation> &obs, const ros::Time &time,
                    tf2::Stamped<TransformWithVariance> &T_mapCam) {
    int numEsts = 0;
    tf2::Stamped<TransformWithVariance> T_camBase;
    tf2::Stamped<TransformWithVariance> T_baseCam;
    tf2::Stamped<TransformWithVariance> T_mapBase;

    std::vector<cv::Point2f> fidsInMap;
    std::vector<cv::Point2f> fidsInCam;
    cv::Point2f         fidCamXY;
    cv::Point2f         fidMapXY;


    tf2::Stamped<TransformWithVariance> T_fid0Cam;    // MJ reversion test
    bool useMulti = false;    // MJ reversion test


    if (obs.size() == 0) {
        return 0;
    }

    tf2::Vector3 cb;
    if (lookupTransform(obs[0].T_camFid.frame_id_, baseFrame, time, T_camBase.transform)) {
        cb = T_camBase.transform.getOrigin();
        if (verboseInfo) {
            ROS_INFO("Map: camera->base   %lf %lf %lf", cb.x(), cb.y(), cb.z());
        }
        T_camBase.variance = 1.0;
    } else {
        ROS_ERROR("Map: Cannot determine tf from camera to robot\n");
    }

    tf2::Vector3 cf;
    if (lookupTransform(baseFrame, obs[0].T_camFid.frame_id_, time, T_baseCam.transform)) {
        cf = T_baseCam.transform.getOrigin();
        if (verboseInfo) {
            ROS_INFO("Map: base->camera   %lf %lf %lf", cf.x(), cf.y(), cf.z());
        }
        T_baseCam.variance = 1.0;
    } else {
        ROS_ERROR("Map: Cannot determine tf from robot to camera\n");
        return numEsts;
    }

    for (Observation &o : obs) {
        if (fiducials.find(o.fid) != fiducials.end()) {
            const Fiducial &fid = fiducials[o.fid];

            // EXPERIMENT: Use estimateRigidTransform
            auto ot = o.T_camFid.transform.getOrigin();
            //fidCamXY = cv::Point2f(((ot.x()*(-1.0)) + cb.x()), ((ot.y()*(-1.0)) + cb.y()));
            //fidCamXY = cv::Point2f((ot.x() + cb.x()), (ot.y() + cb.y()));
            fidCamXY = cv::Point2f(ot.x(), ot.y());
            fidsInCam.push_back(fidCamXY);

            auto mt = fid.pose.transform.getOrigin();
            fidMapXY = cv::Point2f((mt.x()), (mt.y()));    // !!! TEST ONLY, LATER USE GLOBAL MAP 
            fidsInMap.push_back(fidMapXY);
            ROS_INFO("Map: Nav2D Shift fid %d  mapXY  %9.6lf %9.6lf  camXY  %9.6lf %9.6lf", o.fid, 
                fidMapXY.x, fidMapXY.y, fidCamXY.x, fidCamXY.y);

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

            if (verboseInfo) {
                ROS_INFO("Map: Pose %d %8.5lf %8.5lf %8.5lf   %8.5lf  %8.5lf %8.5lf %8.5lf", o.fid, 
                     position.x(), position.y(), position.z(), roll, pitch, yaw, p.variance);
            }
            // drawLine(fid.pose.getOrigin(), o.position);

            if (std::isnan(position.x()) || std::isnan(position.y()) || std::isnan(position.z())) {
                ROS_WARN("Skipping NAN estimate\n");
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
        ROS_INFO("Finished frame - no estimates\n");
        return numEsts;
    }

    // New scope for logging vars
    {
        tf2::Vector3 trans = T_mapBase.transform.getOrigin();
        double r, p, y;
        T_mapBase.transform.getBasis().getRPY(r, p, y);

        ROS_INFO("Map: Pose ALL %lf %lf %lf %lf %lf %lf %f", trans.x(), trans.y(), trans.z(), r, p, y,
                 T_mapBase.variance);
    }


    // Nav2D: Use estimateRigidTransform for pose if that mode is active
    if (navigateFlat) {
        tf2::Stamped<TransformWithVariance> T_mapCameraRigid;

        // Using Nav2D and rigid transform.  
        if (numEsts < 2) {
            // skip this pass if not enough fiducials 
            ROS_WARN("Map: Less than 2 fiducials to attempt rigid transform. SKIPPING THIS PASS!"); 
            return 0;
            // If we get here we will use older pose but it may lead to a jump. Not tested yet.
        } else {
            cv::Mat_<float> nt;
            try {
                tf2::Stamped<TransformWithVariance> T_mapBase2;
                nt = estimateRigidTransform(fidsInCam, fidsInMap, false);
            } catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
            }
            ROS_INFO("Map: Done  rigid transform calc"); 
            if (nt.empty()) {
                ROS_WARN("Map: NO SOLUTION for rigid transform calc! We see %d fiducials", numEsts); 
                return 0;
            } else {
                ROS_INFO("Map: Nav2D matrix: [ %9.6f %9.6f %9.6f ]",
                    nt.at<float>(0,0), nt.at<float>(0,1), nt.at<float>(0,2));
                ROS_INFO("                   [ %9.6f %9.6f %9.6f ]",
                    nt.at<float>(1,0), nt.at<float>(1,1), nt.at<float>(1,2));

                float fidDisp = sqrt((nt.at<float>(0,2) * nt.at<float>(0,2)) + (nt.at<float>(1,2) * nt.at<float>(1,2)));
                float cameraYaw = atan2(nt.at<float>(1,0), nt.at<float>(1,1));
                ROS_INFO("Map: Nav2D radius: %9.6f meters from map with bot yaw %9.4f rads %6.2f deg",
                    fidDisp, cameraYaw, rad2deg(cameraYaw));

                // Create a map to camera transform
                T_mapCameraRigid.transform.setOrigin(tf2::Vector3(nt.at<float>(0,2), nt.at<float>(1,2), 0.0));
                T_mapCameraRigid.transform.getBasis().setRPY(0, 0, cameraYaw);

                T_mapCameraRigid.setData(T_mapCameraRigid * T_camBase);
                auto position = T_mapCameraRigid.transform.getOrigin();
                double roll, pitch, yaw;
                T_mapCameraRigid.transform.getBasis().getRPY(roll, pitch, yaw);

                // Since we are using Nav2D we replace prior 3D nav pose now
                T_mapCameraRigid.stamp_ = T_mapBase.stamp_;
                T_mapBase.setData(T_mapCameraRigid);
                ROS_INFO("Map: Nav2DPose xyz %8.5lf %8.5lf %8.5lf  rpy  %8.5lf  %8.5lf %8.5lf rad %6.2lf deg INUSE",  
                    position.x(), position.y(), position.z(), roll, pitch, yaw, rad2deg(yaw));
            }
        }
        // End use of Nav2D for pose if enabled
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

    robotPosePub.publish(robotPose);

    tf2::Stamped<TransformWithVariance> outPose = basePose;
    outPose.frame_id_ = mapFrame;
    std::string outFrame = baseFrame;

    if (verboseInfo) {
        tf2::Vector3 bp = basePose.transform.getOrigin();
        double r, p, y;
        basePose.transform.getBasis().getRPY(r, p, y);
        ROS_INFO("Map: robotPose xyz  %lf %lf %lf  rpy %lf %lf %lf", bp.x(), bp.y(), bp.z(), r, p, y);
    }

    if (!odomFrame.empty()) {
        outFrame = odomFrame;
        tf2::Transform odomTransform;
        if (lookupTransform(odomFrame, baseFrame, outPose.stamp_, odomTransform)) {
            outPose.setData(basePose * odomTransform.inverse());
            outFrame = odomFrame;

            tf2::Vector3 c = odomTransform.getOrigin();
            ROS_INFO("Map: odom   %lf %lf %lf", c.x(), c.y(), c.z());
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

    ROS_INFO("Map: Finished frame. Estimates %d\n", numEsts);
    return numEsts;
}

// Publish map -> odom tf

void Map::publishTf() {
    tfPublishTime = ros::Time::now();
    poseTf.header.stamp = tfPublishTime + ros::Duration(future_date_transforms);
    broadcaster.sendTransform(poseTf);
}

// publish latest tf if enough time has elapsed

void Map::update() {
    ros::Time now = ros::Time::now();
    if (publishPoseTf && havePose && tfPublishInterval != 0.0 &&
        (now - tfPublishTime).toSec() > tfPublishInterval) {
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

void Map::autoInit(const std::vector<Observation> &obs, const ros::Time &time) {
    ROS_INFO("Map: Auto init map %d", frameNum);

    tf2::Transform T_baseCam;

    if (fiducials.size() == 0) {
        int idx = findClosestObs(obs);

        if (idx == -1) {
            ROS_WARN("Map: Could not find a fiducial to initialize map from");
        }
        const Observation &o = obs[idx];
        originFid = o.fid;

        ROS_INFO("Map: Initializing map from fiducial %d", o.fid);

        tf2::Stamped<TransformWithVariance> T = o.T_camFid;

        if (lookupTransform(baseFrame, o.T_camFid.frame_id_, o.T_camFid.stamp_, T_baseCam)) {
            T.setData(T_baseCam * T);
        }

        fiducials[o.fid] = Fiducial(o.fid, T);
    } else {
        for (const Observation &o : obs) {
            if (o.fid == originFid) {
                tf2::Stamped<TransformWithVariance> T = o.T_camFid;

                tf2::Vector3 trans = T.transform.getOrigin();
                ROS_INFO("Map: Estimate of %d from base %lf %lf %lf err %lf", o.fid, trans.x(),
                         trans.y(), trans.z(), o.T_camFid.variance);

                if (lookupTransform(baseFrame, o.T_camFid.frame_id_, o.T_camFid.stamp_,
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
        ROS_INFO("Map: Fiducial %d is already in map - ignoring add request",
                 fiducialToAdd);
        fiducialToAdd = -1;
        return;
    }

    for (const Observation &o : obs) {
        if (o.fid == fiducialToAdd) {
            ROS_INFO("Map: Adding fiducial_id %d to map", fiducialToAdd);


            tf2::Stamped<TransformWithVariance> T = o.T_camFid;

            // Take into account position of camera on base
            tf2::Transform T_baseCam;
            if (lookupTransform(baseFrame, o.T_camFid.frame_id_,
                                o.T_camFid.stamp_, T_baseCam)) {
                T.setData(T_baseCam * T);
            }

            // Take into account position of robot in the world if known
            tf2::Transform T_mapBase;
            if (lookupTransform(mapFrame, baseFrame, ros::Time(0), T_mapBase)) {
                T.setData(T_mapBase * T);
            }
            else {
                ROS_INFO("Map: Placing robot at the origin");
            }

            fiducials[o.fid] = Fiducial(o.fid, T);
            fiducials[originFid].pose.variance = 0.0;
            isInitializingMap = false;

            fiducialToAdd = -1;
            return;
        }
    }

    ROS_INFO("Map: Unable to add fiducial %d to map", fiducialToAdd);
}

// save map to file

bool Map::saveMap() { return saveMap(mapFilename); }

bool Map::saveMap(std::string filename) {
    ROS_INFO("Map: Saving map with %d fiducials to file %s\n", (int)fiducials.size(), filename.c_str());

    FILE *fp = fopen(filename.c_str(), "w");
    if (fp == NULL) {
        ROS_WARN("Could not open %s for write\n", filename.c_str());
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

    ROS_INFO("Map: Load map %s", filename.c_str());

    FILE *fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        ROS_WARN("Could not open %s for read\n", filename.c_str());
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
                Fiducial(id, tf2::Stamped<TransformWithVariance>(twv, ros::Time::now(), mapFrame));
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
            ROS_WARN("Invalid line: %s", linebuf);
        }
    }

    fclose(fp);
    ROS_INFO("Map: Load map %s read %d entries", filename.c_str(), numRead);
    return true;
}

// Publish the map

void Map::publishMap() {
    fiducial_msgs::FiducialMapEntryArray fmea;
    std::map<int, Fiducial>::iterator it;

    for (const auto &map_pair : fiducials) {
        const Fiducial &f = map_pair.second;

        fiducial_msgs::FiducialMapEntry fme;
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

    mapPub.publish(fmea);
}

// Publish the next marker visualization messages that hasn't been
// published recently

void Map::publishMarkers() {
    ros::Time now = ros::Time::now();
    std::map<int, Fiducial>::iterator it;

    for (auto &map_pair : fiducials) {
        Fiducial &f = map_pair.second;
        if ((now - f.lastPublished).toSec() > 1.0) {
            publishMarker(f);
        }
    }
}

// Publish visualization messages for a single fiducial

void Map::publishMarker(Fiducial &fid) {
    fid.lastPublished = ros::Time::now();

    // Flattened cube
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
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
    marker.header.frame_id = "/map";
    markerPub.publish(marker);

    // cylinder scaled by stddev
    visualization_msgs::Marker cylinder;
    cylinder.type = visualization_msgs::Marker::CYLINDER;
    cylinder.action = visualization_msgs::Marker::ADD;
    cylinder.header.frame_id = "/map";
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
    markerPub.publish(cylinder);

    // Text
    visualization_msgs::Marker text;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.header.frame_id = "/map";
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
    markerPub.publish(text);

    // Links
    visualization_msgs::Marker links;
    links.type = visualization_msgs::Marker::LINE_LIST;
    links.action = visualization_msgs::Marker::ADD;
    links.header.frame_id = "/map";
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

    geometry_msgs::Point gp0, gp1;
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

    markerPub.publish(links);
}

// Publish a line marker between two points

void Map::drawLine(const tf2::Vector3 &p0, const tf2::Vector3 &p1) {
    static int lid = 60000;
    visualization_msgs::Marker line;
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::ADD;
    line.header.frame_id = "/map";
    line.color.r = 1.0f;
    line.color.g = 0.0f;
    line.color.b = 0.0f;
    line.color.a = 1.0f;
    line.id = lid++;
    line.ns = "lines";
    line.scale.x = line.scale.y = line.scale.z = 0.01;
    line.pose.position.x = 0;
    line.pose.position.y = 0;
    geometry_msgs::Point gp0, gp1;
    gp0.x = p0.x();
    gp0.y = p0.y();
    gp0.z = p0.z();
    gp1.x = p1.x();
    gp1.y = p1.y();
    gp1.z = p1.z();
    line.points.push_back(gp0);
    line.points.push_back(gp1);

    markerPub.publish(line);
}

// Service to clear the map and enable auto initialization

bool Map::clearCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("Map: Clearing fiducial map from service call");

    fiducials.clear();
    initialFrameNum = frameNum;
    originFid = -1;

    return true;
}

// Service to add a fiducial to the map

bool Map::addFiducialCallback(fiducial_slam::AddFiducial::Request &req,
                              fiducial_slam::AddFiducial::Response &res)
{
   ROS_INFO("Map: Request to add fiducial %d to map", req.fiducial_id);
   fiducialToAdd = req.fiducial_id;

   return true;
}
