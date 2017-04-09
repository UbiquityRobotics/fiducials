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
#include <fiducial_slam2/helpers.h>

#include <string>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
// Does not Take into account the degree of overlap of observations
static double updateVarianceAlexey(double var1, double var2) {

    return max(1.0 / (1.0/var1 + 1.0/var2), 1e-6);
}

static bool useAlexey;

// Update the variance of a gaussian that has been combined with another
// Taking into account the degree of overlap
// XXX This does not converge well
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


// Constructor for observation

Observation::Observation(int fid, const tf2::Quaternion &q, 
                         const tf2::Vector3 &tvec,
                         double ierr, double oerr) {
    this->fid = fid;
    this->imageError = ierr;
    this->objectError = oerr;
   
    this->poseError = 0.0;

    tf2_ros::TransformBroadcaster broadcaster;

    geometry_msgs::TransformStamped ts;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "base_link"; // XXX should be camera
    ts.child_frame_id = "fid" + to_string(fid);
    ts.transform.translation.x = tvec.x();
    ts.transform.translation.y = tvec.y();
    ts.transform.translation.z = tvec.z();
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();

    broadcaster.sendTransform(ts);

    T_camFid.setRotation(q);
    T_camFid.setOrigin(tvec);

    T_fidCam = T_camFid.inverse();
}


// Update a fiducial with a new pose estimate

void Fiducial::update(const tf2::Transform &newPose, double newVariance)
{
    tf2::Vector3 mean1 = pose.getOrigin();
    tf2::Quaternion q = pose.getRotation();

    updateTransform(pose, variance, newPose, newVariance);

    numObs++;

    tf2::Vector3 mean2 = newPose.getOrigin();
    tf2::Vector3 newMean = pose.getOrigin();
 
    double v = updateVarianceDavid(newMean, mean1, variance,
                              mean2, newVariance);
    variance = v;
}


// Create a fiduciial from an pose estimate

Fiducial::Fiducial(int id, const tf2::Transform &pose, double variance) {
    this->id = id;
    this->pose = pose;
    this->variance = variance;
    this->lastPublished = ros::Time(0);
    this->numObs = 0;
    this->visible = false;
}


// Create a fiducial from the pose components

Fiducial::Fiducial(int id, const tf2::Quaternion &q, 
                   const tf2::Vector3 &tvec, 
                   double variance) {
    this->id = id;

    pose.setRotation(q); 
    pose.setOrigin(tf2::Vector3(tvec[0], tvec[1], tvec[2]));

    this->variance = variance;
    this->lastPublished = ros::Time(0);
    this->numObs = 0;
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
    nh.param<std::string>("camera_frame", cameraFrame, "camera");
    nh.param<std::string>("base_frame", baseFrame, "base_link");

    nh.param<bool>("use_alexey", useAlexey, true);

    nh.param<std::string>("map_file", mapFilename, 
        string(getenv("HOME")) + "/.ros/slam/map.txt");

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
        tf2::Transform cameraPose;
        if (updatePose(obs, time, cameraPose) > 0 && obs.size() > 1) {
            updateMap(obs, time, cameraPose);
        }
    }

    publishMap();
}


// update estimates of oberved fiducials from previously estimated
// camera pose

void Map::updateMap(const vector<Observation>& obs, const ros::Time &time,
     const tf2::Transform &cameraPose)
{
    map<int, Fiducial>::iterator fit;

    for (fit = fiducials.begin(); fit != fiducials.end(); fit++) {
        Fiducial &f = fit->second;
        f.visible = false;
    }

    for (int i=0; i<obs.size(); i++) {
        const Observation &o = obs[i];

        tf2::Transform T_mapFid = cameraPose * o.T_camFid;

        tf2::Vector3 trans = T_mapFid.getOrigin();
            
        double variance = o.objectError;

        ROS_INFO("Estimate of %d %lf %lf %lf err %lf %lf var %lf",
          o.fid, trans.x(), trans.y(), trans.z(),
          o.objectError, o.poseError, variance);

        if (isnan(trans.x()) || isnan(trans.y()) || isnan(trans.z())) {
            ROS_WARN("Skipping NAN estimate\n");
            continue;
        };

        if (fiducials.find(o.fid) == fiducials.end()) {
            ROS_INFO("New fiducial %d", o.fid);
            fiducials[o.fid] = Fiducial(o.fid, T_mapFid, variance);
        }
        Fiducial &f = fiducials[o.fid]; 
        f.visible = true;
        if (f.variance != 0) {
           f.update(T_mapFid, variance);
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

        T.setOrigin(tf2::Vector3(
           transform.transform.translation.x,
           transform.transform.translation.y,
           transform.transform.translation.z));
 
        T.setRotation(tf2::Quaternion(
           transform.transform.rotation.x,
           transform.transform.rotation.y,
           transform.transform.rotation.z,
           transform.transform.rotation.w));
 
        return true;
     }
     catch (tf2::TransformException &ex) {
         ROS_WARN("%s",ex.what());
         return false;
     }
}

// update pose estimate of robot

int Map::updatePose(vector<Observation>& obs, const ros::Time &time,
                    tf2::Transform &cameraPose)
{
    tf2::Transform pose;
    double variance = 0.0;
    int numEsts = 0;

    for (int i=0; i<obs.size(); i++) {
        Observation &o = obs[i];
        if (fiducials.find(o.fid) != fiducials.end()) {
            const Fiducial &fid = fiducials[o.fid]; 

            tf2::Transform p = fid.pose * o.T_fidCam;

            double v = fid.variance + o.objectError;

            o.position = p.getOrigin();
            ROS_INFO("Pose %d %lf %lf %lf %lf", o.fid, 
              o.position.x(), o.position.y(), o.position.z(), v);

            //drawLine(fid.pose.getOrigin(), o.position);

            if (isnan(o.position.x()) || isnan(o.position.y())
                || isnan(o.position.z())) {
                ROS_WARN("Skipping NAN estimate\n");
                continue;
            };

            if (numEsts == 0) {
                pose = p;
                variance = v;
            }
            else {
                updateTransform(pose, variance, p, v);
                variance = updateVarianceAlexey(variance, v); 
            }
            numEsts++;
        }
    }

    if (numEsts == 0) {
        ROS_INFO("Finished frame - no estimates\n");
        return numEsts;
    }

    cameraPose = pose;

    tf2::Vector3 trans = pose.getOrigin();
    tf2::Quaternion q = pose.getRotation();
    ROS_INFO("Pose all %lf %lf %lf %f",
           trans.x(), trans.y(), trans.z(), variance);

    // Update error in observations

    for (int i=0; i<obs.size(); i++) {
        Observation &o = obs[i];
        o.poseError = (o.position - trans).length2();
    }

    // Determine transform from camera to robot
    tf2::Transform cameraTransform;

/*
    if (lookupTransform(cameraFrame, baseFrame, time, cameraTransform)) {
        pose = pose * cameraTransform;
     
        tf2::Vector3 c = cameraTransform.getOrigin();
        ROS_INFO("camera   %lf %lf %lf %f",
           c.x(), c.y(), c.z(), variance);

        trans = pose.getOrigin();
        ROS_INFO("Pose b_l %lf %lf %lf %f",
           trans.x(), trans.y(), trans.z(), variance);

     }
*/

     geometry_msgs::PoseWithCovarianceStamped pwcs;
     pwcs.header.frame_id = baseFrame;
     pwcs.header.stamp = time;
     pwcs.pose.pose.orientation.x = q.x();
     pwcs.pose.pose.orientation.y = q.y();
     pwcs.pose.pose.orientation.z = q.z();
     pwcs.pose.pose.orientation.w = q.w();
     pwcs.pose.pose.position.x = trans.x();
     pwcs.pose.pose.position.y = trans.y();
     pwcs.pose.pose.position.z = trans.z();

     for (int i=0; i<5; i++) {
         for (int j=0; j<5; j++) {
             pwcs.pose.covariance[i*5+j] = 0;
         }
     } 
     for (int i=0; i<5; i++) {
         pwcs.pose.covariance[i*5+i] = variance;
     } 

     posePub.publish(pwcs);

     string outFrame=baseFrame;
/*
     if (!odomFrame.empty()) {
         tf2::Transform odomTransform;
         if (lookupTransform(odomFrame, baseFrame, time, odomTransform)) {
             pose = pose * odomTransform;
             outFrame = odomFrame;
             tf2::Vector3 c = odomTransform.getOrigin();
             ROS_INFO("odom   %lf %lf %lf %f",
                c.x(), c.y(), c.z(), variance);
         }
    }
*/

    // TODO: nicer way to init TransformStamped
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = mapFrame;
    ts.child_frame_id = outFrame;
    ts.transform.translation.x = trans.x();
    ts.transform.translation.y = trans.y();
    ts.transform.translation.z = trans.z();
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();
 
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
        double d = o.T_camFid.getOrigin().length2();
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

        tf2::Transform T = o.T_camFid;


/*
        if (lookupTransform(baseFrame, cameraFrame, time, cameraTransform)) {
            T = cameraTransform * T;
        }
*/

        fiducials[o.fid] = Fiducial(o.fid, T, o.objectError);
    } 
    else {
        for (int i=0; i<obs.size(); i++) {
            const Observation &o = obs[0];

            if (o.fid == originFid) {
                tf2::Transform T = o.T_camFid;

/*
                if (lookupTransform(baseFrame, cameraFrame, time, 
                    cameraTransform)) {
                    T = cameraTransform * T;
                }
*/

                tf2::Vector3 trans = T.getOrigin();
                ROS_INFO("Estimate of %d from base %lf %lf %lf err %lf",
                     o.fid, trans.x(), trans.y(), trans.z(), o.objectError);

                fiducials[originFid].update(T, o.objectError);
                break;
            } 
        }
    }

    if (frameNum > 10 && originFid != -1) {
        isInitializingMap = false;

        fiducials[originFid].variance = 0.0;
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
        tf2::Vector3 trans = f.pose.getOrigin();
        double rx, ry, rz;
        f.pose.getBasis().getRPY(rx, ry, rz);

        fprintf(fp, "%d %lf %lf %lf %lf %lf %lf %lf %d", f.id, 
                 trans.x(), trans.y(), trans.z(), 
                 rad2deg(rx), rad2deg(ry), rad2deg(rz), f.variance, f.numObs);

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
             Fiducial f = Fiducial(id, q, tvec, var);
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
    cylinder.scale.x = cylinder.scale.y = min(sqrt(fid.variance), 0.1);
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
    tf2::Vector3 p0 = fid.pose.getOrigin();
    gp0.x = p0.x();
    gp0.y = p0.y();
    gp0.z = p0.z();
  
    map<int, int>::iterator lit;
    for (lit = fid.links.begin(); lit != fid.links.end(); lit++) {
        int ofid = lit->first;
        // only draw links in one direction
        if (fid.id < ofid) {
            if (fiducials.find(ofid) != fiducials.end()) {
                tf2::Vector3 p1 = fiducials[ofid].pose.getOrigin(); 
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
