/*
 * Copyright (c) 2017, Ubiquity Robotics
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

#include <fiducial_slam/map.h>
#include <fiducial_slam/helpers.h>

#include <string>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include <boost/filesystem.hpp>


// Update the variance of a gaussian that has been combined with another
// Does not Take into account the degree of overlap of observations
static double updateVarianceAlexey(double var1, double var2) {

    return max(1.0 / (1.0/var1 + 1.0/var2), 1e-6);
}

static bool useAlexey = false;

// Update the variance of a gaussian that has been combined with another
// Taking into account the degree of overlap
static double updateVarianceDavid(const tf2::Vector3 &newMean,
                                  const tf2::Vector3 &mean1, double var1,
                                  const tf2::Vector3 &mean2, double var2) {
    if (useAlexey) {
       return updateVarianceAlexey(var1, var2);
    }

    //=((2*PI())^0.5)*C3*D3*EXP((((((C2-E2)^2))/(2*C3^2))+(((D2-E2)^2)/(2*(D3^2)))))
    double d1 = (mean1 - newMean).length2();
    double d2 = (mean2 - newMean).length2();

    double newVar = sqrt(2.0*M_PI) * var1 * var2 *
         exp(((d1 / (2.0*var1)) + d2 / (2.0*var2)));

    if (newVar > 100)
        newVar = 100;
    if (newVar < 10e-4)
        newVar = 10e-4;
    return newVar;
}

// Update transform t1 with t2 using variances as weights.
// The result is in t1
static void updateTransform(tf2::Transform &t1, double var1,
                            const tf2::Transform &t2, double var2) {
    tf2::Vector3 o1 = t1.getOrigin();
    tf2::Vector3 o2 = t2.getOrigin();

    t1.setOrigin((var1 * o2 + var2 * o1) / (var1 + var2));

    tf2::Quaternion q1 = t1.getRotation();
    tf2::Quaternion q2 = t2.getRotation();
    t1.setRotation(q1.slerp(q2, var1 / (var1 + var2)).normalized());
}

// Update this transform with a new one, with variances as weights
// combine variances using David method
void TransformWithVariance::update(const TransformWithVariance& newT) {
    tf2::Vector3 o1 = transform.getOrigin();
    tf2::Quaternion q1 = transform.getRotation();
    double var1 = variance;

    tf2::Vector3 o2 = newT.transform.getOrigin();
    tf2::Quaternion q2 = newT.transform.getRotation();
    double var2 = newT.variance;

    transform.setOrigin((var1 * o2 + var2 * o1) / (var1 + var2));
    transform.setRotation(q1.slerp(q2, var1 / (var1 + var2)).normalized());

    variance = updateVarianceDavid(transform.getOrigin(), o1, var1, o2, var2);
}

// Weighted average of 2 transforms, variances computed using Alexey Method
TransformWithVariance averageTransforms(const TransformWithVariance& t1, const TransformWithVariance& t2) {
    TransformWithVariance out;
    tf2::Vector3 o1 = t1.transform.getOrigin();
    tf2::Quaternion q1 = t1.transform.getRotation();
    double var1 = t1.variance;

    tf2::Vector3 o2 = t2.transform.getOrigin();
    tf2::Quaternion q2 = t2.transform.getRotation();
    double var2 = t2.variance;

    out.transform.setOrigin((var1 * o2 + var2 * o1) / (var1 + var2));
    out.transform.setRotation(q1.slerp(q2, var1 / (var1 + var2)).normalized());
    out.variance = updateVarianceAlexey(var1, var2);

    return out;
}


// Constructor for observation
Observation::Observation(int fid, const tf2::Stamped<TransformWithVariance>& camFid,
                         double ierr, double oerr) {
    this->fid = fid;
    this->imageError = ierr;

    this->poseError = 0.0;

    tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped ts = toMsg(camFid);
    ts.child_frame_id = "fid" + to_string(fid);
    broadcaster.sendTransform(ts);

    T_camFid = camFid;
    T_fidCam = T_camFid;
    T_fidCam.transform  = T_camFid.transform.inverse();
}


// Update a fiducial position in map with a new estimate
void Fiducial::update(const tf2::Stamped<TransformWithVariance>& newPose) {
    pose.update(newPose);
    numObs++;
}


// Create a fiducial from an estimate of its position in the map
Fiducial::Fiducial(int id, const tf2::Stamped<TransformWithVariance>& pose) {
    this->id = id;
    this->pose = pose;
    this->lastPublished = ros::Time(0);
    this->numObs = 0;
    this->visible = false;
}


// Constructor for map

Map::Map(ros::NodeHandle &nh) : tfBuffer(ros::Duration(30.0)){
    frameNum = 0;
    isInitializingMap = false;

    listener = make_unique<tf2_ros::TransformListener>(tfBuffer);

    posePub = ros::Publisher(
          nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/fiducial_pose", 1));
    markerPub = ros::Publisher(
          nh.advertise<visualization_msgs::Marker>("/fiducials", 100));
    mapPub = ros::Publisher(
          nh.advertise<fiducial_msgs::FiducialMapEntryArray>("/fiducial_map",
          1));

    nh.param<std::string>("map_frame", mapFrame, "map");
    nh.param<std::string>("odom_frame", odomFrame, "odom");
    nh.param<std::string>("base_frame", baseFrame, "base_link");

    nh.param<double>("future_date_transforms", future_date_transforms, 0.1);


    nh.param<std::string>("map_file", mapFilename,
        string(getenv("HOME")) + "/.ros/slam/map.txt");

    boost::filesystem::path mapPath(mapFilename);
    boost::filesystem::path dir = mapPath.parent_path();
    boost::filesystem::create_directory(dir);

    std::string initialMap;
    nh.param<std::string>("initial_map_file", initialMap, "");

    if (!initialMap.empty()) {
        loadMap(initialMap);
    }
    else {
        loadMap();
    }

    publishMarkers();
}


// Update map with a set of observations

void Map::update(vector<Observation>& obs, const ros::Time &time)
{
    ROS_INFO("Updating map with %d observations. Map has %d fiducials",
        (int)obs.size(), (int)fiducials.size());

    frameNum++;

    if (obs.size() > 0 && fiducials.size() == 0) {
        isInitializingMap = true;
    }

    if (isInitializingMap) {
        autoInit(obs, time);
    }
    else {
        tf2::Stamped<TransformWithVariance> cameraPose;
        cameraPose.frame_id_ = mapFrame;

        if (updatePose(obs, time, cameraPose) > 0 && obs.size() > 1) {
            updateMap(obs, time, cameraPose);
        }
    }

    publishMap();
}


// update estimates of observed fiducials from previously estimated
// camera pose

void Map::updateMap(const vector<Observation>& obs, const ros::Time &time,
                    const tf2::Stamped<TransformWithVariance>& cameraPose)
{
    map<int, Fiducial>::iterator fit;

    for (fit = fiducials.begin(); fit != fiducials.end(); fit++) {
        Fiducial &f = fit->second;
        f.visible = false;
    }

    for (int i=0; i<obs.size(); i++) {
        const Observation &o = obs[i];

        // This should take into account the variances from both
        tf2::Stamped<TransformWithVariance> T_mapFid = cameraPose * o.T_camFid;
        T_mapFid.frame_id_ = mapFrame;

        // New scope for logging vars
        {
            tf2::Vector3 trans = T_mapFid.transform.getOrigin();

            ROS_INFO("Estimate of %d %lf %lf %lf err %lf %lf var %lf",
                     o.fid, trans.x(), trans.y(), trans.z(),
                     o.T_camFid.variance, o.poseError, T_mapFid.variance);

            if (isnan(trans.x()) || isnan(trans.y()) || isnan(trans.z())) {
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

        for (int j=0; j<obs.size(); j++) {
            int fid = obs[j].fid;
            if (f.id != fid) {
                f.links[fid] = 1;
            }
        }
        publishMarker(fiducials[o.fid]);
    }
}


// lookup specified transform

bool Map::lookupTransform(const std::string &from, const std::string &to,
                          const ros::Time &time, tf2::Transform &T) const
{
    geometry_msgs::TransformStamped transform;

    try {
        transform = tfBuffer.lookupTransform(from, to, time);

        tf2::fromMsg(transform.transform, T);
        return true;
     }
     catch (tf2::TransformException &ex) {
         ROS_WARN("%s",ex.what());
         return false;
     }
}

// update pose estimate of robot

int Map::updatePose(vector<Observation>& obs, const ros::Time &time,
                    tf2::Stamped<TransformWithVariance>& cameraPose)
{
    double variance = 0.0;
    int numEsts = 0;

    for (int i=0; i<obs.size(); i++) {
        Observation &o = obs[i];
        if (fiducials.find(o.fid) != fiducials.end()) {
            const Fiducial &fid = fiducials[o.fid];

            tf2::Stamped<TransformWithVariance> p = fid.pose * o.T_fidCam;
            p.frame_id_ = mapFrame;
            p.stamp_ = o.T_fidCam.stamp_;

            o.position = p.transform.getOrigin();
            ROS_INFO("Pose %d %lf %lf %lf %lf", o.fid,
              o.position.x(), o.position.y(), o.position.z(), p.variance);

            //drawLine(fid.pose.getOrigin(), o.position);

            if (isnan(o.position.x()) || isnan(o.position.y())
                || isnan(o.position.z())) {
                ROS_WARN("Skipping NAN estimate\n");
                continue;
            };

            if (numEsts == 0) {
                cameraPose = p;
            }
            else {
                cameraPose.setData(averageTransforms(cameraPose, p));
                cameraPose.stamp_ = p.stamp_;
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
        tf2::Vector3 trans = cameraPose.transform.getOrigin();
        tf2::Quaternion q = cameraPose.transform.getRotation();
        ROS_INFO("Pose all %lf %lf %lf %f",
                 trans.x(), trans.y(), trans.z(), variance);
    }

    // Determine transform from camera to robot
    tf2::Transform cameraTransform;
    // Use robotPose instead of camera pose to hold map to robot
    tf2::Stamped<TransformWithVariance> basePose = cameraPose;

    if (lookupTransform(obs[0].T_camFid.frame_id_, baseFrame, time, cameraTransform)) {
        basePose.setData(cameraPose * cameraTransform);

        // New scope for logging vars
        {
            tf2::Vector3 c = cameraPose.transform.getOrigin();
            ROS_INFO("camera   %lf %lf %lf %f",
                     c.x(), c.y(), c.z(), variance);

            tf2::Vector3 trans = basePose.transform.getOrigin();
            ROS_INFO("Pose b_l %lf %lf %lf %f",
                     trans.x(), trans.y(), trans.z(), variance);
        }
     }

    posePub.publish(toPose(basePose));

    tf2::Stamped<TransformWithVariance> outPose = basePose;
    outPose.frame_id_ = mapFrame;
    string outFrame=baseFrame;
    if (!odomFrame.empty()) {
         outFrame=odomFrame;
         tf2::Transform odomTransform;
         if (lookupTransform(odomFrame, baseFrame, outPose.stamp_, odomTransform)) {

             outPose.setData(basePose * odomTransform.inverse());
             outFrame = odomFrame;

             tf2::Vector3 c = odomTransform.getOrigin();
             ROS_INFO("odom   %lf %lf %lf %f",
                c.x(), c.y(), c.z(), variance);
         }
    }
 
    // Make outgoing transform make sense - ie only consist of x, y, yaw
    tf2::Vector3 translation = outPose.transform.getOrigin();
    translation.setZ(0);
    outPose.transform.setOrigin(translation);
    double roll, pitch, yaw;
    outPose.transform.getBasis().getRPY(roll, pitch, yaw);
    outPose.transform.getBasis().setRPY(0, 0, yaw);

    geometry_msgs::TransformStamped ts = toMsg(outPose);
    ts.child_frame_id = outFrame;
    ts.header.stamp += ros::Duration(future_date_transforms);
    broadcaster.sendTransform(ts);

    ROS_INFO("Finished frame\n");
    return numEsts;
}


// Find closest fiducial to camera

static int findClosestObs(const vector<Observation>& obs)
{
    double smallestDist = -1;
    int closestIdx = -1;

    for (int i=0; i<obs.size(); i++) {
        const Observation &o = obs[0];
        double d = o.T_camFid.transform.getOrigin().length2();
        if (smallestDist < 0 || d < smallestDist) {
            smallestDist = d;
            closestIdx = i;
        }
    }

    return closestIdx;
}


// Initialize a map from the closest observed fiducial

void Map::autoInit(const vector<Observation>& obs, const ros::Time &time){

    ROS_INFO("Auto init map %d", frameNum);

    static int originFid = -1;
    tf2::Transform cameraTransform;

    if (fiducials.size() == 0) {
        int idx = findClosestObs(obs);

        if (idx == -1) {
            ROS_WARN("Could not find a fiducial to initialize map from");
        }
        const Observation &o = obs[idx];
        originFid = o.fid;

        ROS_INFO("Initializing map from fiducial %d", o.fid);

        tf2::Stamped<TransformWithVariance>T = o.T_camFid;

        if(lookupTransform(baseFrame, o.T_camFid.frame_id_, o.T_camFid.stamp_, cameraTransform)) {
            T.setData(T * cameraTransform);
        }

        fiducials[o.fid] = Fiducial(o.fid, T);
    }
    else {
        for (int i=0; i<obs.size(); i++) {
            const Observation &o = obs[0];

            if (o.fid == originFid) {
                tf2::Stamped<TransformWithVariance> T = o.T_camFid;

                tf2::Vector3 trans = T.transform.getOrigin();
                ROS_INFO("Estimate of %d from base %lf %lf %lf err %lf",
                     o.fid, trans.x(), trans.y(), trans.z(), o.T_camFid.variance);

                if(lookupTransform(baseFrame, o.T_camFid.frame_id_, o.T_camFid.stamp_, cameraTransform)) {
                    T.setData(T * cameraTransform);
                }

                fiducials[originFid].update(T);
                break;
            }
        }
    }

    if (frameNum > 10 && originFid != -1) {
        isInitializingMap = false;

        fiducials[originFid].pose.variance = 0.0;
    }
}


// save map to file

bool Map::saveMap() {
    return saveMap(mapFilename);
}

bool Map::saveMap(std::string filename)
{
    ROS_INFO("Saving map with %d fiducials to file %s\n",
         (int)fiducials.size(), filename.c_str());

    FILE *fp = fopen(filename.c_str(), "w");
    if (fp == NULL) {
        ROS_WARN("Could not open %s for write\n", filename.c_str());
        return false;
    }

    map<int, Fiducial>::iterator it;
    map<int, int>::iterator lit;

    for (it = fiducials.begin(); it != fiducials.end(); it++) {
        Fiducial &f = it->second;
        tf2::Vector3 trans = f.pose.transform.getOrigin();
        double rx, ry, rz;
        f.pose.transform.getBasis().getRPY(rx, ry, rz);

        fprintf(fp, "%d %lf %lf %lf %lf %lf %lf %lf %d", f.id,
                 trans.x(), trans.y(), trans.z(),
                 rad2deg(rx), rad2deg(ry), rad2deg(rz), f.pose.variance, f.numObs);

        for (lit = f.links.begin(); lit != f.links.end(); lit++) {
            fprintf(fp, " %d", lit->first);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    return true;
}


// Load map from file

bool Map::loadMap() {
    return loadMap(mapFilename);
}

bool Map::loadMap(std::string filename)
{
    int numRead = 0;

    ROS_INFO("Load map %s", filename.c_str());

    FILE *fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        ROS_WARN("Could not open %s for read\n", filename.c_str());
        return false;
    }

    const int BUFSIZE = 2048;
    char linebuf[BUFSIZE];
    char linkbuf[BUFSIZE];

    while (!feof(fp)) {
        if (fgets(linebuf, BUFSIZE - 1, fp) == NULL)
            break;

        int id;
        double tx, ty, tz, rx, ry, rz, var;
        int numObs = 0;

        linkbuf[0] = '\0';
        int nElems = sscanf(linebuf, "%d %lf %lf %lf %lf %lf %lf %lf %d%[^\t\n]*s",
                            &id, &tx, &ty, &tz, &rx, &ry, &rz, &var, &numObs, linkbuf);
        if (nElems == 9 || nElems == 10) {
             tf2::Vector3 tvec(tx, ty, tz);
             tf2::Quaternion q;
             q.setRPY(deg2rad(rx), deg2rad(ry), deg2rad(rz));

             auto twv = TransformWithVariance(tvec, q, var);
             // TODO: figure out what the timestamp in Fiducial should be
             Fiducial f = Fiducial(id, tf2::Stamped<TransformWithVariance>(twv, ros::Time::now(), mapFrame));
             f.numObs = numObs;

             istringstream ss(linkbuf);
             string s;
             while (getline(ss, s, ' ')) {
                 if (!s.empty()) {
                     f.links[stoi(s)] = 1;
                 }
             }
             fiducials[id] = f;
             numRead++;
        }
        else {
             ROS_WARN("Invalid line: %s", linebuf);
        }
    }

    fclose(fp);
    ROS_INFO("Load map %s read %d entries", filename.c_str(), numRead);
    return true;
}


// Publish the map

void Map::publishMap()
{
    fiducial_msgs::FiducialMapEntryArray fmea;
    map<int, Fiducial>::iterator it;

    for (it = fiducials.begin(); it != fiducials.end(); it++) {
        const Fiducial &f = it->second;

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

void Map::publishMarkers()
{
    ros::Time now = ros::Time::now();
    map<int, Fiducial>::iterator it;

    for (it = fiducials.begin(); it != fiducials.end(); it++) {
        Fiducial &f = it->second;
        if ((now - f.lastPublished).toSec() > 1.0) {
            publishMarker(f);
        }
    }
}


// Publish visualization messages for a single fiducial

void Map::publishMarker(Fiducial &fid)
{
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
    }
    else {
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
    cylinder.id = fid.id; + 10000;
    cylinder.ns = "sigma";
    cylinder.scale.x = cylinder.scale.y = min(sqrt(fid.pose.variance), 0.1);
    cylinder.scale.z = 0.01;
    cylinder.pose.position.x = marker.pose.position.x;
    cylinder.pose.position.y = marker.pose.position.y;
    cylinder.pose.position.z = marker.pose.position.z;
    cylinder.pose.position.z += (marker.scale.z/2.0) + 0.05;
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
    text.pose.position.z += (marker.scale.z/2.0) + 0.1;
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

    map<int, int>::iterator lit;
    for (lit = fid.links.begin(); lit != fid.links.end(); lit++) {
        int ofid = lit->first;
        // only draw links in one direction
        if (fid.id < ofid) {
            if (fiducials.find(ofid) != fiducials.end()) {
                tf2::Vector3 p1 = fiducials[ofid].pose.transform.getOrigin();
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

void Map::drawLine(const tf2::Vector3 &p0, const tf2::Vector3 &p1)
{
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
