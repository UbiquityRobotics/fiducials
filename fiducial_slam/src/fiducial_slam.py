#!/usr/bin/python

"""
Copyright (c) 2015, Ubiquity Robotics
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of ubiquity_motor nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
Simultaneous location and mapping based on fiducial poses.
Receives FiducialTransform messages and builds a map and estimates the
camera pose from them
"""


import rospy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion, \
                              TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

from fiducial_pose.msg import Fiducial, FiducialTransform, FiducialTransformArray

from tf.transformations import euler_from_quaternion, quaternion_slerp, \
                               translation_matrix, quaternion_matrix, \
                               translation_from_matrix, quaternion_from_matrix, \
                               quaternion_from_euler
import tf
import rosbag

from math import pi, sqrt
import sys
import os
import traceback
import math
import numpy
import time
import threading
import copy

# These thresholds are to prevent the map being updated repeateadly with the same
# observation.  They specify how much the robot needs to move (meters, radians) before
# the map will be updated again
MIN_UPDATE_TRANSLATION = 1.0
MIN_UPDATE_ROTATION = math.pi/4.0

# Used to make an estimeate of error, based on tilt
CEILING_HEIGHT = 2.77

# How long to wait before marking a seen marker as unseen
UNSEEN_TIME = 1.5


"""
Radians to degrees
"""
def rad2deg(rad):
    return rad / math.pi * 180.0

"""
Degrees to radians
"""
def deg2rad(deg):
    return float(deg) * math.pi / 180.0

"""
Distance from the nearest perpendicular
"""


def angularError(r):
    r = r % 360
    rDiff = r % 90
    quadrant = r - rDiff
    err = abs(min(rDiff, 90 - rDiff))
    return err

def angularError3D(r, p, y):
    a = angularError(rad2deg(r))
    b = angularError(rad2deg(p))
    c = angularError(rad2deg(y))
    tilt = deg2rad(math.sqrt(a*a + b*b + c*c))
    error = CEILING_HEIGHT * math.tan(tilt)
    variance = error * error
    print "Variance", variance
    return variance


"""
Weighted average of linear quantities
"""
def updateLinear(mean1, var1, mean2, var2):
    newMean = (mean1 * var2 + mean2 * var1) / (var1 + var2)
    newVar = 1.0 / (1.0/var1 + 1.0/var2)
    return [newMean, newVar]


"""
Weighted average of quaternions
"""
def updateAngular(quat1, var1, quat2, var2):
    newMean = quaternion_slerp(quat1, quat2, var1/(var1+var2))
    newVar = 1.0 / (1.0/var1 + 1.0/var2)
    return [newMean, newVar]


"""
Class to represent the pose and state of a single fiducial
"""
class Fiducial:
    def __init__(self, id):
        self.id = id
        self.position = None
        self.orientation = None
        self.variance = None
        self.observations = 0
        self.links = []
        self.publishedMarker = False
        self.lastSeenTime = rospy.Time(0,0)

    """
    Return pose as a 4x4 matrix
    """
    def pose44(self):
        return numpy.dot(translation_matrix(self.position),
                         quaternion_matrix(self.orientation))

    """
    Update pose with a new observation
    """
    def update(self, newPosition, newOrientation, newVariance):
        if self.observations == 0:
            self.position = newPosition
            self.orientation = numpy.array(newOrientation)
            self.variance = newVariance
            self.observations = 1
            return
        self.position, v1 = updateLinear(self.position, self.variance,
                                         newPosition, newVariance)
        self.orientation, v2 = updateAngular(self.orientation, self.variance,
                                             newOrientation, newVariance)
        self.variance = v1
        self.observations += 1


class FiducialSlam:
    def __init__(self):
       rospy.init_node('fiducial_slam')
       self.odomFrame = rospy.get_param("~odom_frame", "")
       self.poseFrame = rospy.get_param("~pose_frame", "base_link")
       self.mapFrame = rospy.get_param("~map_frame", "map")
       self.cameraFrame = rospy.get_param("~camera_frame", "camera")
       self.sendTf = rospy.get_param("~publish_tf", True)
       self.mappingMode = rospy.get_param("~mapping_mode", True)
       self.useExternalPose = rospy.get_param("~use_external_pose", False)
       self.mapFileName = rospy.get_param("~map_file", "map.txt")
       self.obsFileName = rospy.get_param("~obs_file", "obs.txt")
       self.transFileName = rospy.get_param("~trans_file", "trans.txt")
       # How much to future date our tfs
       self.future = rospy.get_param("~future", 0.0)
       # Republish tf
       self.republishTf = rospy.get_param("~republish_tf", True)
       print "frames: odom", self.odomFrame, "map:", self.mapFrame, "pose", self.poseFrame
       self.obsFile = open(self.obsFileName, "a")
       self.transFile = open(self.transFileName, "a")
       self.tfs = {}
       self.currentSeq = None
       self.imageTime = None
       self.numFiducials = 0
       self.fiducials = {}
       self.mapPublished = False
       if self.sendTf:
           self.br = tf.TransformBroadcaster()
       self.lr = tf.TransformListener(True, rospy.Duration(10))
       self.markerPub = rospy.Publisher("fiducials", Marker, queue_size=20)
       self.visibleMarkers = {}
       self.pose = None
       self.robotQuat = None
       self.robotXyz = None
       self.robotYaw = 0.0
       self.lastUpdateXyz = None
       self.lastUpdateYaw = None
       self.lastTfPubTime = 0
       self.loadMap()
       self.position = None
       rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.newTf)
       self.posePub = rospy.Publisher("/fiducial_pose", PoseWithCovarianceStamped, queue_size=1)


    def close(self):
        self.saveMap()
        self.obsFile.close()
        self.transFile.close()


    """
    Save current map to file
    """
    def saveMap(self):
        print "** save map **"
        file = open(self.mapFileName, "w")
        fids = self.fiducials.keys()
        fids.sort()
        for fid in fids:
            f = self.fiducials[fid]
            pos = f.position
            (r, p, y) = euler_from_quaternion(f.orientation)
            file.write("%d %r %r %r %r %r %r %r %d %s\n" % \
              (f.id, pos[0], pos[1], pos[2],
               rad2deg(r), rad2deg(p), rad2deg(y),
               f.variance, f.observations,
               ' '.join(map(str, f.links))))
        file.close()

    """
    Load map from file
    """
    def loadMap(self):
        file = open(self.mapFileName, "r")
        for line in file.readlines():
            words = line.split()
            fid = int(words[0])
            f = Fiducial(fid)
            f.position = numpy.array((float(words[1]), float(words[2]), float(words[3])))
            f.orientation = quaternion_from_euler(deg2rad(words[4]), deg2rad(words[5]), deg2rad(words[6]))
            f.variance = float(words[7])
            f.observations = int(words[8])
            f.links = map(int, words[9:])
            self.fiducials[fid] = f
        file.close()

    """
    Print out fiducual vertices
    """
    def showVertices(self):
        fids = self.fiducials.keys()
        fids.sort()
        for fid in fids:
            f = self.fiducials[fid]
            pos = f.position
            off = numpy.dot(translation_matrix(numpy.array((-1.0, -1.0, 0.0))), f.pose44())
            off = numpy.dot(translation_matrix(numpy.array((1.0, -1.0, 0.0))), f.pose44())
            off = numpy.dot(translation_matrix(numpy.array((1.0, 1.0, 0.0))), f.pose44())
            off = numpy.dot(translation_matrix(numpy.array((-1.0, 1.0, 0.0))), f.pose44())

    """
    Called when a FiducialTransformArray is received
    """
    def newTf(self, msg):
        self.currentSeq = msg.image_seq
        self.imageTime = m.header.stamp
        self.tfs = {}
        rospy.loginfo("got tfs from image seq %d", self.currentSeq)

        for m in msg.transforms:
            id = m.fiducial_id
            trans = m.transform.translation
            rot = m.transform.rotation
            mat = numpy.dot(translation_matrix((trans.x, trans.y, trans.z)),
                            quaternion_matrix((rot.x, rot.y, rot.z, rot.w)))
            invMat = numpy.linalg.inv(mat)
            self.tfs[id] = (mat, invMat, m.object_error, m.image_error)
            self.transFile.write("%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n" % \
                                 (id, seq, trans.x, trans.y, trans.z, 
                                  rot.x, rot.y, rot.z, rot.w,
                                  m.object_error, m.image_error, m.fiducial_area))
        """
        if self.useExternalPose:
            # XXXX needs to be verified with respect to time
            fiducialKnown = False
            for f in self.tfs.keys():
            if self.fiducials.has_key(f):
                fiducialKnown = True 
                if not fiducialKnown:
                    for f in self.tfs.keys():
                        self.updateMapFromExternal(f)
        """
        self.updatePose()
        if not self.pose is None:
            robotXyz = numpy.array(translation_from_matrix(self.pose))[:3]
            robotQuat = numpy.array(quaternion_from_matrix(self.pose))
            (r, p, robotYaw) = euler_from_quaternion(robotQuat)
            # Only update the map if the robot has moved significantly, to 
            # avoid the map variances decaying from repeated observations
            if self.lastUpdateXyz is None:
                self.updateMap()
                self.lastUpdateXyz = robotXyz
                self.lastUpdateYaw = robotYaw
            else:
                dist = numpy.linalg.norm(self.lastUpdateXyz - robotXyz)
                angle = self.lastUpdateYaw - self.robotYaw
                print "Distance moved", dist, angle
                if self.mappingMode or dist > MIN_UPDATE_TRANSLATION \
                   or angle > MIN_UPDATE_ROTATION:
                    self.updateMap()
                    self.lastUpdateXyz = robotXyz
                    self.lastUpdateYaw = robotYaw
        
    """
    Update the map with fiducial pairs
    """
    def updateMap(self):
        for f1 in self.tfs.keys():
            for f2 in self.tfs.keys():
                if f1 == f2:
                    continue
                if not self.fiducials.has_key(f1):
                    continue
                self.updateMapPair(f1, f2)

    """
    Update the map with the new transform between the fiducial pair
    f1 and f2
    """
    def updateMapPair(self, f1, f2):
        fid1 = self.fiducials[f1]
 
        # Don't update ground truth fidicuals
        if self.fiducials.has_key(f2):
            if self.fiducials[f2].variance == 0.0:
                return
        
        # Don't update f2 if the only estimate of f1 came from it
        if len(fid1.links) == 1 and f1 in fid1.links:
	    return

        P_fid1 = fid1.pose44()

        (T_camFid1, T_fid1Cam, oerr1, ierr1) = self.tfs[f1]
        (T_camFid2, T_fid2Cam, oerr2, ierr2) = self.tfs[f2]

        # transform form fiducial f1 to f2
        T_fid1Fid2 = numpy.dot(T_fid1Cam, T_camFid2)
                             
        # pose of f1 transformed by trans
        P_fid2 = numpy.dot(P_fid1, T_fid1Fid2)

        xyz = numpy.array(translation_from_matrix(P_fid2))[:3]
        quat = numpy.array(quaternion_from_matrix(P_fid2))
        (r, p, yaw) = euler_from_quaternion(quat)

        self.obsFile.write("%d %d %d %r %r %r %r %r %r %r %r %r %r\n" % \
                               (self.currentSeq, f2, f1, 
                                xyz[0], xyz[1], xyz[2], 
                                rad2deg(r), rad2deg(p), rad2deg(yaw),
                                oerr1, ierr1, oerr2, ierr2))
                                                
        addedNew = False
        if not self.fiducials.has_key(f2):
            self.fiducials[f2] = Fiducial(f2)
            addedNew = True
            
        fid2 = self.fiducials[f2]

        variance  = angularError3D(r, p , yaw)
        # and take into account the variance of the reference fiducial
        # we convolve the gaussians, which is achieved by adding the variances
        print "*** %f var %f %f" % (f1, self.fiducials[f1].variance, variance)
        variance = variance + self.fiducials[f1].variance
        self.fiducials[f2].update(xyz, quat, variance)

        print "%d updated to %.3f %.3f %.3f %.3f %.3f %.3f %.3f" % (f2, xyz[0], xyz[1], xyz[2],
            rad2deg(r), rad2deg(p), rad2deg(yaw), variance)
          
        p = self.fiducials[f2].position

        if self.mappingMode or addedNew:
            self.saveMap()

        if not f1 in fid2.links:
            fid2.links.append(f1)
        if not f2 in fid1.links:
            fid1.links.append(f2)

    """
    Update the map with a new fiducial based on an external robot pose
    """
    def updateMapFromExternal(self, f):
        rospy.logerr("update from external %d" % f)
        try:
            ct, cr = self.lr.lookupTransform(self.mapFrame,
                                             self.cameraFrame, 
                                             self.imageTime)
            T_worldCam = numpy.dot(translation_matrix((ct[0], ct[1], ct[2])),
                           quaternion_matrix((cr[0], cr[1], cr[2], cr[3])))
        except:
            rospy.logerr("Unable to lookup transfrom from map to camera (%s to %s)" % \
                         (self.mapFrame, self.cameraFrame))

	    return

        (T_camFid, T_fidCam, oerr1, ierr1) = self.tfs[f]

        P_fid = numpy.dot(T_worldCam, T_camFid)

        self.fiducials[f] = Fiducial(f)
                             
        xyz = numpy.array(translation_from_matrix(P_fid))[:3]
        quat = numpy.array(quaternion_from_matrix(P_fid))
        (r, p, yaw) = euler_from_quaternion(quat)

        variance = 0.3 # TODO: look at AMCL variance
        self.fiducials[f].update(xyz, quat, variance)

        print "%d updated from external to %.3f %.3f %.3f %.3f %.3f %.3f %.3f" % (f, xyz[0], xyz[1], xyz[2],
            rad2deg(r), rad2deg(p), rad2deg(yaw), variance)
          
        p = self.fiducials[f].position
        self.saveMap()


    """ 
    Estimate the pose of the camera from the fiducial to camera
    transforms in self.tfs
    """
    def updatePose(self):
        position = None
        orientation = None
        camera = None
        try:
            ct, cr = self.lr.lookupTransform(self.cameraFrame,
                                             self.poseFrame,
                                             self.imageTime)
            T_camBase = numpy.dot(translation_matrix((ct[0], ct[1], ct[2])),
                       quaternion_matrix((cr[0], cr[1], cr[2], cr[3])))
        except:
            rospy.logerr("Unable to lookup transfrom from camera to robot (%s to %s)" % \
                         (self.poseFrame, self.cameraFrame))
            return
        
        for t in self.tfs.keys():
            if not self.fiducials.has_key(t):
                rospy.logwarn("No path to %d" % t)
                continue

            self.fiducials[t].lastSeenTime = self.imageTime

            (T_camFid, T_fidCam, oerr, ierr) = self.tfs[t]
            T_worldFid = self.fiducials[t].pose44()

            T_worldCam = numpy.dot(T_worldFid, T_fidCam)
            T_worldBase = numpy.dot(T_worldCam, T_camBase)

            xyz = numpy.array(translation_from_matrix(T_worldBase))[:3]
            quat = numpy.array(quaternion_from_matrix(T_worldBase))
            (r, p, y) = euler_from_quaternion(quat)

            thisvar  = angularError3D(r, p , 0.0)
            rospy.loginfo("pose %d %f %f %f %f %f %f %f" % \
                             (t, xyz[0], xyz[1], xyz[2],
                              rad2deg(r), rad2deg(p), rad2deg(y),
                              thisvar))

            if position is None:
                position = xyz
                orientation = quat
                variance = thisvar
            else:
                position, v1 = updateLinear(position, variance,
                                       xyz, thisvar)
                orientation, v2 = updateAngular(orientation, variance,
                                                quat, thisvar)
                variance = v1
        if not position is None:
            (r, p, y) = euler_from_quaternion(orientation)
            xyz = position
            rospy.loginfo("pose ALL %f %f %f %f %f %f %f %d" % \
                            (xyz[0], xyz[1], xyz[2],
                             rad2deg(r), rad2deg(p), rad2deg(y), 
                             variance, self.currentSeq))
            self.pose = numpy.dot(translation_matrix((position[0], 
                                                      position[1],
                                                      0)),
                                  quaternion_matrix((orientation[0],
                                                     orientation[1],
                                                     orientation[2],
                                                     orientation[3])))
            self.computeTransform()

    def makeMarker(self, fiducialId, visible=False):
        fiducial = self.fiducials[fiducialId]
        marker = Marker()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        position = fiducial.position
        marker.pose = Pose(Point(position[0], position[1], position[2]),
                           Quaternion(0, 0, 0, 1))
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        if visible:
            marker.color = ColorRGBA(1, 0, 0, 1)
        else:
            marker.color = ColorRGBA(0, 1, 0, 1)
        marker.id = fiducialId
        marker.ns = "fiducial_namespace"
        marker.header.frame_id = "/map"

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

        links = Marker()
        links.ns = "fiducial_namespace"
        links.header.frame_id = "/map"
        links.action = Marker.ADD
        links.type = Marker.LINE_LIST
        links.pose.position.x = 0.0
        links.pose.position.y = 0.0
        links.pose.position.z = 0.0
        links.color = ColorRGBA(0, 0, 1, 1)
        links.scale.x = 0.05
        for fid2 in fiducial.links:
            if self.fiducials.has_key(fid2):
                position2 = self.fiducials[fid2].position
                links.points.append(Point(position[0], position[1], position[2]))
                links.points.append(Point(position2[0], position2[1], position[2]))
        links.id = fiducialId + 20000
        links.ns = "fiducial_namespace_link"

        return marker, text, links


    def publishMarker(self, fiducialId, visible=False):
        marker, text, links = self.makeMarker(fiducialId, visible)
        self.markerPub.publish(marker)
        self.markerPub.publish(text)
        self.markerPub.publish(links)

            
    """
    Publish the next unpublished marker in the map.
    This should be called repeatedly until all markers are published
    """
    def publishNextMarker(self):
        if self.mapPublished:
            return
        for fid in self.fiducials.keys():
            if not self.fiducials[fid].publishedMarker:
                self.publishMarker(fid)
                self.fiducials[fid].publishedMarker = True
                return
        self.mapPublished = True
        print "Map published"

    """
    Update markers with visibility colors
    """
    def publishMarkers(self):
        self.publishNextMarker()
        t = self.imageTime
        for fid in self.tfs.keys():
            if self.fiducials.has_key(fid):
                self.visibleMarkers[fid] =  True
        for fid in self.visibleMarkers.keys():
            if (t - self.fiducials[fid].lastSeenTime).to_sec() > UNSEEN_TIME:
                self.publishMarker(fid, False)
                del self.visibleMarkers[fid]
            else:
                self.publishMarker(fid, True)


    """
    Publish the transform in self.pose
    """
    def computeTransform(self):
        if self.pose is None:
            return
        robotXyz = numpy.array(translation_from_matrix(self.pose))[:3]
        robotQuat = numpy.array(quaternion_from_matrix(self.pose))
        (r, p, yaw) = euler_from_quaternion(robotQuat)
        robotQuat = quaternion_from_euler(0.0, 0.0, yaw) 
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.mapFrame
        m.header.stamp = self.imageTime
        m.pose.pose.orientation.x = robotQuat[0]
        m.pose.pose.orientation.y = robotQuat[1]
        m.pose.pose.orientation.z = robotQuat[2]
        m.pose.pose.orientation.w = robotQuat[3]
        m.pose.pose.position.x = robotXyz[0]
        m.pose.pose.position.y = robotXyz[1]
        m.pose.pose.position.z = robotXyz[2]
        """
        These values are designed to work with robot_localization.
        See http://wiki.ros.org/robot_localization/Tutorials/Migration%20from%20robot_pose_ekf
        """
        m.pose.covariance = [0.01,  0,     0,      0,     0,     0,
                             0,     0.01,  0,      0,     0,     0,
                             0,     0,     0.01,   0,     0,     0,
                             0,     0,     0,      0.01,  0,     0,
                             0,     0,     0,      0,     0.01,  0,
                             0,     0,     0,      0,     0,     0.01]
        self.posePub.publish(m)
        if self.odomFrame != "":
            try: 
                if self.imageTime is None:
                    rospy.logerr("imageTime is bogus!")
                odomt, odomr = self.lr.lookupTransform(self.poseFrame, self.odomFrame, self.imageTime)
                odom = numpy.dot(translation_matrix((odomt[0], odomt[1], odomt[2])),
                                 quaternion_matrix((odomr[0], odomr[1], odomr[2], odomr[3])))
                pose = numpy.dot(self.pose, odom)
            except:
                rospy.logerr("Unable to lookup transfrom from odom to robot (%s to %s)" % \
                             (self.poseFrame, self.odomFrame))
                return
        else:
            pose = self.pose
        self.robotXyz = numpy.array(translation_from_matrix(pose))[:3]
        robotQuat = numpy.array(quaternion_from_matrix(pose))
        (r, p, yaw) = euler_from_quaternion(robotQuat)
        self.robotQuat = quaternion_from_euler(0.0, 0.0, yaw) 
        self.robotYaw = yaw
        self.publishTransform()

    def publishTransform(self):
        if self.sendTf and not self.robotXyz is None:
            if self.odomFrame != "":
                toFrame = self.odomFrame
                fromFrame = self.mapFrame
            else:
                toFrame = self.poseFrame
                fromFrame = self.mapFrame
            self.br.sendTransform(self.robotXyz,
                                  self.robotQuat,
                                  rospy.Time.now() + rospy.Duration(self.future),
                                  toFrame,
                                  fromFrame)
            self.lastTfPubTime = rospy.get_time()

    def run(self):
        hz = 10.0
        rospy.loginfo("Fiducial Slam started")
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            dt = rospy.get_time() - self.lastTfPubTime
            #rospy.loginfo("Tf age %f", dt)
            if self.republishTf:
                self.publishTransform()
            self.publishMarkers()
            rate.sleep()
        self.close()
        rospy.loginfo("Fiducial Slam ended")

if __name__ == "__main__":
    node = FiducialSlam()
    node.run()
