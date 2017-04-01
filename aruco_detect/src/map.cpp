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

#include <map.h>

#include <string>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

/* 
updateVarianceDavid
def updateLinear(mean1, var1, mean2, var2):
    newMean = (mean1 * var2 + mean2 * var1) / (var1 + var2)
    # =((2*PI())^0.5)*C3*D3*EXP((((((C2-E2)^2))/(2*C3^2))+(((D2-E2)^2)/(2*(D3^2)))))
    d1 = dist = numpy.linalg.norm(mean1 - newMean)
    d2 = dist = numpy.linalg.norm(mean2 - newMean)
    try:
        newVar = ((2.0*math.pi)**0.5) * var1 * var2 * math.exp((((d1**2.0) / (2.0*var1)) +
            ((d2**2.0) / (2.0*var2))))
        #newVar = 1.0 / (1.0/var1 + 1.0/var2)
        if newVar > 100000:
            newVar = 100000
        if newVar < 10e-6:
            newVar = 10e-6
    except:
        newVar = 99999
    print "newVar = %f" %newVar
    return [newMean, newVar]
*/

static double updateVarianceAlexey(double var1, double var2) {
    return max(1.0 / (1.0/var1 + 1.0/var2), 1e-8);
}

static double updateVariance(double var1, double var2) {
    return updateVarianceAlexey(var1, var2);
}

static void updateTransform(tf2::Transform &t1, double var1, tf2::Transform &t2, double var2) {
    tf2::Vector3 o1 = t1.getOrigin();
    tf2::Vector3 o2 = t2.getOrigin();
 
    t1.setOrigin((var1 * o2 + var2 * o1) / (var1 + var2));
    
    tf2::Quaternion q1 = t1.getRotation();
    tf2::Quaternion q2 = t2.getRotation();
    q1.slerp(q2, var2 / (var1 + var2)).normalized();
    t1.setRotation(q1);
}

Observation::Observation(int fid, Vec3d &rvec, Vec3d &tvec, double ierr, double oerr) {
    this->fid = fid;
    this->imageError = ierr;
    this->objectError = oerr;

    double angle = norm(rvec);
    Vec3d axis = rvec / angle;

    TfidCam.setRotation(tf2::Quaternion(tf2::Vector3(axis[0], axis[1], axis[2]), angle)); 
    TfidCam.setOrigin(tf2::Vector3(tvec[0], tvec[1], tvec[2]));

    TcamFid = TfidCam.inverse();
}

void Fiducial::update(tf2::Transform &newPose, double newVariance)
{
    tf2::Vector3 t = pose.getOrigin();
    printf("Pose %f %f %f\n", t.x(), t.y(), t.z());
    updateTransform(pose, variance, newPose, newVariance);
    double v = updateVariance(this->variance, variance);
    printf("Fiducial %d variance changed from %lf to %lf\n", id,
           newVariance, v);
    t = pose.getOrigin();
    printf("New pose %f %f %f\n", t.x(), t.y(), t.z());
    variance = v;
}

Fiducial::Fiducial(int id, tf2::Transform &pose, double variance) {
    this->id = id;
    this->pose = pose;
    this->variance = variance;
}

Fiducial::Fiducial(int id, Vec3d rvec, Vec3d tvec, double variance) {
    this->id = id;

    double angle = norm(rvec);
    Vec3d axis = rvec / angle;

    pose.setRotation(tf2::Quaternion(tf2::Vector3(axis[0], axis[1], axis[2]), angle)); 
    pose.setOrigin(tf2::Vector3(tvec[0], tvec[1], tvec[2]));

    this->variance = variance;
};   

Fiducial::Fiducial(int id, Vec3d axis, double angle, Vec3d tvec, double variance) {
    this->id = id;

    pose.setRotation(tf2::Quaternion(tf2::Vector3(axis[0], axis[1], axis[2]), angle)); 
    pose.setOrigin(tf2::Vector3(tvec[0], tvec[1], tvec[2]));

    this->variance = variance;
};   
         
Map::Map(ros::NodeHandle &nh) {
    markerPub = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("/fiducials", 100));
    publishMarkers();
}

void Map::update(vector<Observation>& obs, ros::Time time)
{
    printf("Updating map with %d observations\n", (int)obs.size());
   
    // update map
    for (int i=0; i<obs.size(); i++) {
        for (int j=0; j<obs.size(); j++) {
            Observation o1 = obs[i];
            Observation o2 = obs[j];

            // source and dest are the same
            if (o1.fid == o2.fid) {
                continue;
            }
            
            // source not in map
            if (fiducials.find(o1.fid) == fiducials.end()) {
                ROS_WARN("No path to %d", o1.fid);
                continue;
            }

            // dest is origin
            if (fiducials.find(o2.fid) != fiducials.end() &&
                fiducials[o2.fid].variance == 0) {
                continue;
            }

            tf2::Transform T = fiducials[o1.fid].pose * o1.TfidCam * o2.TcamFid;
            double variance = o1.objectError + o2.objectError + 
              fiducials[o1.fid].variance + 0.1;

            if (fiducials.find(o2.fid) == fiducials.end()) {
                fiducials[o2.fid] = Fiducial(o2.fid, T, variance);
            }
            else {
                fiducials[o2.fid].update(T, variance);
            }
            publishMarker(fiducials[o2.fid]);
        }
    }

    // update pose
    tf2::Transform pose;
    double variance = 0.0;

    for (int i=0; i<obs.size(); i++) {
        Observation o = obs[i];
        if (fiducials.find(o.fid) != fiducials.end()) {
            tf2::Transform p = fiducials[o.fid].pose * o.TfidCam;
            double v = fiducials[o.fid].variance + o.objectError;

            tf2::Vector3 trans = p.getOrigin();
            printf("Pose %d %f %f %f %f\n", o.fid, 
              trans.x(), trans.y(), trans.z(), v);

            if (variance == 0.0) {
                pose = p;
                variance = v;
            }
            else {
                updateTransform(pose, variance, p, v);
                variance = updateVariance(variance, v); 
            }
        }
    }
    tf2::Vector3 trans = pose.getOrigin();
    tf2::Quaternion q = pose.getRotation();
    printf("Pose all %f %f %f %f\n",
           trans.x(), trans.y(), trans.z(), variance);

    geometry_msgs::TransformStamped ts;
    ts.header.stamp = ros::Time::now(); //time;
    ts.header.frame_id = "map";
    ts.child_frame_id = "base_link";
    ts.transform.translation.x = trans.x();
    ts.transform.translation.y = trans.y();
    ts.transform.translation.z = trans.z();
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();
 
    broadcaster.sendTransform(ts);

    fflush(stdout);
}

bool Map::save(std::string filename) 
{
   printf("saving map %d\n", (int)fiducials.size());

    map<int, Fiducial>::iterator it;

    for (it = fiducials.begin(); it != fiducials.end(); it++) {
        Fiducial &f = it->second;
        tf2::Vector3 trans = f.pose.getOrigin();

        printf("%d %f %f %f %f\n", f.id, 
               trans.x(), trans.y(), trans.z(), f.variance);
    }
    printf("map saved\n");
    return true;
}

bool Map::load(std::string filename) 
{
}

void Map::publishMarkers() 
{
    map<int, Fiducial>::iterator it;

    for (it = fiducials.begin(); it != fiducials.end(); it++) {
        publishMarker(it->second);
    }
}
 
void Map::publishMarker(Fiducial &fid) 
{
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
}
 

/*
        text = Marker()
        text.header.frame_id = "/map"
        text.color = ColorRGBA(1, 1, 1, 1) # white
        text.scale.x = text.scale.y = text.scale.z = 0.1
        text.pose.position.x = marker.pose.position.x
        text.pose.position.y = marker.pose.position.y
        text.pose.position.z = marker.pose.position.z
        text.pose.position.z += (marker.scale.z/2.0) + 0.1  # draw text above marker
        text.id = fiducialId + 10000
        text.ns = "fiducial_namespace_text"
        text.type = Marker.TEXT_VIEW_FACING
        text.text = str(fiducialId)
        text.action = Marker.ADD

*/
