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

#include <fiducial_slam2/map.h>

#include <string>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

// Degrees to radians
static double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

// Radians to degrees
static double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}

// Update the variance of a gaussian that has been combined with another
// Taking into account the degree of overlap
static double updateVarianceDavid(const tf2::Vector3 &newMean,
                                  const tf2::Vector3 &mean1, double var1,
                                  const tf2::Vector3 &mean2, double var2) {
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

// Update the variance of a gaussian that has been combined with another
// Does not Take into account the degree of overlap
static double updateVarianceAlexey(double var1, double var2) {
    return max(1.0 / (1.0/var1 + 1.0/var2), 1e-6);
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
    t1.setRotation(q1.slerp(q2, var1 / (var1 + var2)).normalize());
}

// Constructor for observation
Observation::Observation(int fid, const cv::Vec3d &rvec, 
                         const cv::Vec3d &tvec,
                         double ierr, double oerr) {
    this->fid = fid;
    this->imageError = ierr;
    this->objectError = oerr;

    double angle = norm(rvec);
    cv::Vec3d axis = rvec / angle;

    TfidCam.setRotation(tf2::Quaternion(tf2::Vector3(axis[0], axis[1], axis[2]), angle)); 
    TfidCam.setOrigin(tf2::Vector3(tvec[0], tvec[1], tvec[2]));

    TcamFid = TfidCam.inverse();
}


Observation::Observation(int fid, const tf2::Quaternion &q, 
                         const tf2::Vector3 &tvec,
                         double ierr, double oerr) {
    this->fid = fid;
    this->imageError = ierr;
    this->objectError = oerr;

    TfidCam.setRotation(q); 
    TfidCam.setOrigin(tvec);

    TcamFid = TfidCam.inverse();
}

// Update a fiducial with a new pose estimate
void Fiducial::update(const tf2::Transform &newPose, double newVariance)
{
    tf2::Vector3 mean1 = pose.getOrigin();
    tf2::Quaternion q = pose.getRotation();

    double rx, ry, rz, yaw;
    pose.getBasis().getRPY(rx, ry, rz);
    yaw = rz;

    updateTransform(pose, variance, newPose, newVariance);

    if (anchor) {
       // Preserve tx, ty, rz 
       tf2::Vector3 trans = pose.getOrigin();
       trans.setX(mean1.x());
       trans.setY(mean1.y());
       pose.setOrigin(trans);
       pose.getBasis().getRPY(rx, ry, rz);
       q.setRPY(rx, ry, yaw);
       pose.setRotation(q);
    }
    tf2::Vector3 mean2 = newPose.getOrigin();
    tf2::Vector3 newMean = pose.getOrigin();
    printf("Original %f %f %f", mean1.x(), mean1.y(), mean1.z());
    printf("mean2 %f %f %f", mean2.x(), mean2.y(), mean2.z());
    printf("newMean %f %f %f", newMean.x(), newMean.y(), newMean.z());
 
    double v = updateVarianceDavid(newMean, mean1, variance,
                              mean2, newVariance);
    printf("Fiducial %d variance changed from %lf to %lf\n", id,
           variance, v);
    variance = v;
}

// Create a fiduciial from an pose estimate
Fiducial::Fiducial(int id, const tf2::Transform &pose, double variance) {
    this->id = id;
    this->pose = pose;
    this->variance = variance;
    this->lastPublished = ros::Time(0);
    this->anchor = false;
}

// Create a fiducial from the pose components
Fiducial::Fiducial(int id, const tf2::Quaternion &q, 
                   const tf2::Vector3 &tvec, 
                   double variance, bool anchor) {
    this->id = id;

    pose.setRotation(q); 
    pose.setOrigin(tf2::Vector3(tvec[0], tvec[1], tvec[2]));

    this->variance = variance;
    this->lastPublished = ros::Time(0);
    this->anchor = anchor;
};   
         
// Constructor for map
Map::Map(ros::NodeHandle &nh) {
    tfBuffer = new tf2_ros::Buffer(ros::Duration(30.0));
    listener = new tf2_ros::TransformListener(*tfBuffer);

    markerPub = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("/fiducials", 100));
    mapPub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialMapEntryArray>("/fiducial_map", 100));
    nh.param<std::string>("map_file", filename, string(getenv("HOME")) + "/.ros/slam/map.txt");

    std::string initialMap;
    nh.param<std::string>("initial_map_file", initialMap, "");

    if (initialMap != "") {
        loadMap(initialMap);
    }
    else {
        loadMap();
    }

    publishMarkers();
}

// Update map with a set of observations
void Map::update(const vector<Observation>& obs, ros::Time time)
{
    printf("Updating map with %d observations\n", (int)obs.size());

    if (obs.size() > 0 && fiducials.size() == 0) {
        autoInit(obs, time);
    }
   
    updateMap(obs, time);
    updatePose(obs, time);
    publishMap();
}

// update pose estimates of oberved fiducials
void Map::updateMap(const vector<Observation>& obs, ros::Time time)
{
    for (int i=0; i<obs.size(); i++) {
        for (int j=0; j<obs.size(); j++) {
            const Observation &o1 = obs[i];
            const Observation &o2 = obs[j];

            // source and dest are the same
            if (o1.fid == o2.fid) {
                continue;
            }
            
            // source not in map
            if (fiducials.find(o1.fid) == fiducials.end()) {
                ROS_WARN("No path to %d", o1.fid);
                continue;
            }

            tf2::Transform T = fiducials[o1.fid].pose * o1.TfidCam * o2.TcamFid;
            double variance = o1.objectError + o2.objectError + 
              max(fiducials[o1.fid].variance, 10e-5);

             
            if (fiducials.find(o2.fid) == fiducials.end()) {
                ROS_INFO("New fiducial %d from %d", o2.fid, o1.fid);
                fiducials[o2.fid] = Fiducial(o2.fid, T, variance);
                saveMap();
            }
            else {
                ROS_INFO("Updated fiducial %d from %d", o2.fid, o1.fid);
                fiducials[o2.fid].update(T, variance);
            }
            publishMarker(fiducials[o1.fid]);
            publishMarker(fiducials[o2.fid]);
        }
    }
}

// update pose estimate of robot
void Map::updatePose(const vector<Observation>& obs, ros::Time time)
{
    tf2::Transform pose;
    double variance = 0.0;

    for (int i=0; i<obs.size(); i++) {
        const Observation &o = obs[i];
        if (fiducials.find(o.fid) != fiducials.end()) {
            tf2::Transform p = fiducials[o.fid].pose * o.TfidCam;
            double v = fiducials[o.fid].variance + o.objectError;

            tf2::Vector3 trans = p.getOrigin();
            printf("Pose %d %lf %lf %lf %lf\n", o.fid, 
              trans.x(), trans.y(), trans.z(), v);

            if (variance == 0.0) {
                pose = p;
                variance = v;
            }
            else {
                updateTransform(pose, variance, p, v);
                variance = updateVarianceAlexey(variance, v); 
            }
        }
    }
    tf2::Vector3 trans = pose.getOrigin();
    tf2::Quaternion q = pose.getRotation();
    printf("Pose all %lf %lf %lf %f\n",
           trans.x(), trans.y(), trans.z(), variance);

    // Determine transform from camera to robot
    geometry_msgs::TransformStamped cameraTransform;
    try{
        // TODO: params
        cameraTransform = tfBuffer->lookupTransform("base_link", "raspicam",
                                                    ros::Time(0));
     }
     catch (tf2::TransformException &ex) {
         ROS_WARN("Could not lookup camera transform %s",ex.what());
     }

    tf2::Transform ct;
    ct.setOrigin(tf2::Vector3(
       cameraTransform.transform.translation.x,
       cameraTransform.transform.translation.y,
       cameraTransform.transform.translation.z));
 
    ct.setRotation(tf2::Quaternion(
       cameraTransform.transform.rotation.x,
       cameraTransform.transform.rotation.y,
       cameraTransform.transform.rotation.z,
       cameraTransform.transform.rotation.w));
 
    //pose = pose * ct;
    trans = pose.getOrigin();
    printf("Pose b_l %lf %lf %lf %f\n",
           trans.x(), trans.y(), trans.z(), variance);

    // TODO: nicer way to init TransformStamped
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = ros::Time::now(); //time;
    // TODO: params for frames
    ts.header.frame_id = "map";
    ts.child_frame_id = "base_link2";
    ts.transform.translation.x = trans.x();
    ts.transform.translation.y = trans.y();
    ts.transform.translation.z = trans.z();
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();
 
    broadcaster.sendTransform(ts);

    ROS_WARN("Done with this image");
    // TODO: publish PoseWithCovarianceStamped

    fflush(stdout);
}

// Initialize a map from the closest observed fiducial
void Map::autoInit(const vector<Observation>& obs, ros::Time time){
    double smallestDist = -1;
    int closestIdx = -1;

    for (int i=0; i<obs.size(); i++) {
        const Observation &o = obs[0];
        double d = o.TcamFid.getOrigin().length2();
        if (smallestDist < 0 || d < smallestDist) {
            smallestDist = d;
            closestIdx = i;
        }
    }
    if (closestIdx == -1) {
        ROS_WARN("Could not find a fiducial to initialize map from");
        return;
    }

    const Observation &o = obs[closestIdx];
    ROS_INFO("Initializing map from fiducial %d", o.fid);

    // set fiducial with only z specified and zero rotation
    tf2::Quaternion q(0, 0, 0, 1);
    tf2::Vector3 tvec(0, 0, o.TcamFid.getOrigin().z());
    fiducials[o.fid] = Fiducial(o.fid, q, tvec,
        o.objectError, true);
}

// save map to file

bool Map::saveMap() {
    return saveMap(this->filename);
}

bool Map::saveMap(std::string filename) 
{
    // TODO: handle links
    printf("saving map %d\n", (int)fiducials.size());

    FILE *fp = fopen(filename.c_str(), "w");
    if (fp == NULL) {
        ROS_WARN("Could not open %s for write\n", filename.c_str());
        return false;
    }

    map<int, Fiducial>::iterator it;

    for (it = fiducials.begin(); it != fiducials.end(); it++) {
        Fiducial &f = it->second;
        tf2::Vector3 trans = f.pose.getOrigin();
        double rx, ry, rz;
        f.pose.getBasis().getRPY(rx, ry, rz);

        fprintf(fp, "%d %lf %lf %lf %lf %lf %lf %lf %d\n", f.id, 
                 trans.x(), trans.y(), trans.z(), 
                 rad2deg(rx), rad2deg(ry), rad2deg(rz), f.variance,
                 f.anchor ? 1 : 0);
    }
    fclose(fp);
    printf("map saved\n");
    return true;
}

// Load map from file

bool Map::loadMap() {
    return loadMap(this->filename);
}

bool Map::loadMap(std::string filename) 
{
    printf("Load map %s\n", filename.c_str());
    FILE *fp = fopen(filename.c_str(), "r");
    if (fp == NULL) {
        ROS_WARN("Could not open %s for read\n", filename.c_str());
        return false;
    }

    const int BUFSIZE = 2048;
    char linebuf[BUFSIZE];
    int id;
    double tx, ty, tz, rx, ry, rz, var;
    int anchor = 0;

    // TODO: read links
    while (!feof(fp)) {
        if (fgets(linebuf, BUFSIZE - 1, fp) == NULL)
            break;
         if (sscanf(linebuf, "%d %lf %lf %lf %lf %lf %lf %lf %d",
                    &id, &tx, &ty, &tz, &rx, &ry, &rz, &var, &anchor) == 9) {
             tf2::Vector3 tvec(tx, ty, tz);
             tf2::Quaternion q;
             q.setRPY(deg2rad(rx), deg2rad(ry), deg2rad(rz));
             fiducials[id] = Fiducial(id, q, tvec, var, (anchor>0));
         }
    }
    fclose(fp);
    return true;
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
 
void Map::publishMap()
{
    fiducial_msgs::FiducialMapEntryArray fmea;
    map<int, Fiducial>::iterator it;

    for (it = fiducials.begin(); it != fiducials.end(); it++) {
        const Fiducial &f = it->second;

        fiducial_msgs::FiducialMapEntry fme;
        fme.fiducial_id = f.id;
 
        tf2::Vector3 t = f.pose.getOrigin();
        fme.x = t.x();
        fme.y = t.y();
        fme.z = t.z();

        double rx, ry, rz;
        f.pose.getBasis().getRPY(rx, ry, rz);
        fme.rx = rx;
        fme.ry = ry;
        fme.rz = rz;
     
        fmea.fiducials.push_back(fme);
    }

    mapPub->publish(fmea);
}

// Publish a single marker visualization message
void Map::publishMarker(Fiducial &fid) 
{
    // TODO: publish links
    fid.lastPublished = ros::Time::now();

    // Flattened cube
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Vector3 t = fid.pose.getOrigin();
    marker.pose.position.x = t.x(); 
    marker.pose.position.y = t.y(); 
    marker.pose.position.z = t.z(); 
    tf2::Quaternion q = fid.pose.getRotation();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
   
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.01;
    std_msgs::ColorRGBA c;
    c.r = c.b = 0.0f;
    c.g = c.a = 1.0f;
    marker.color = c;
    marker.id = fid.id;
    marker.ns = "fiducial_namespace";
    marker.header.frame_id = "/map";
    markerPub->publish(marker);

    // cylinder scaled by stddev
    visualization_msgs::Marker cyl;
    cyl.type = visualization_msgs::Marker::CYLINDER;
    cyl.action = visualization_msgs::Marker::ADD;
    cyl.header.frame_id = "/map";
    c.r = c.g = c.g = 0.0f;
    c.b = 1.0f;
    c.a = 0.8f;
    cyl.color = c;
    cyl.id = fid.id;
    cyl.scale.x = cyl.scale.y = sqrt(fid.variance);
    cyl.scale.z = 0.1;
    cyl.pose.position.x = marker.pose.position.x;
    cyl.pose.position.y = marker.pose.position.y;
    cyl.pose.position.z = marker.pose.position.z;
    cyl.pose.position.z += (marker.scale.z/2.0) + 0.05;
    markerPub->publish(cyl);

    // Text
    visualization_msgs::Marker text;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.header.frame_id = "/map";
    c.r = c.b = c.g = c.a = 1.0f;
    text.color = c;
    text.id = fid.id;
    text.scale.x = text.scale.y = text.scale.z = 0.1;
    text.pose.position.x = marker.pose.position.x;
    text.pose.position.y = marker.pose.position.y;
    text.pose.position.z = marker.pose.position.z;
    text.pose.position.z += (marker.scale.z/2.0) + 0.1;
    text.id = fid.id + 10000;
    text.ns = "fiducial_namespace_text";
    text.text = std::to_string(fid.id);
    markerPub->publish(text);
}
