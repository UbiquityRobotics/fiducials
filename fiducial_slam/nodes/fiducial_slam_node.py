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

from fiducial_slam.msg import FiducialMapEntry, FiducialMapEntryArray
from fiducial_pose.msg import Fiducial, FiducialTransform, FiducialTransformArray

from tf.transformations import euler_from_quaternion, quaternion_slerp, \
                               translation_matrix, quaternion_matrix, \
                               translation_from_matrix, quaternion_from_matrix, \
                               quaternion_from_euler

from fiducial_slam.srv import InitializeMap
from fiducial_slam.fiducial import Fiducial
from fiducial_slam.map import Map
from fiducial_slam import mkdirnotex, rad2deg, deg2rad, updateLinear, updateAngular

import tf2_ros

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
    rospy.loginfo("a = %f b = %f c = %f" % (a, b, c))
    tilt = deg2rad(math.sqrt(a*a + b*b))# + c*c))
    error = CEILING_HEIGHT * math.tan(tilt)
    variance = error * error
    rospy.loginfo("Variance = %f" % variance)
    return variance


class Observation:
   def __init__(self, T_camFid, objectError, imageError, cameraFrame):
       self.T_camFid = T_camFid
       self.T_fidCam = numpy.linalg.inv(T_camFid)
       self.objectError = objectError
       self.imageError = imageError
       self.cameraFrame = cameraFrame

class FiducialSlam:
    def __init__(self):
       rospy.init_node('fiducial_slam')
       self.odomFrame = rospy.get_param("~odom_frame", "")
       self.poseFrame = rospy.get_param("~pose_frame", "base_link")
       self.mapFrame = rospy.get_param("~map_frame", "map")
       self.sendTf = rospy.get_param("~publish_tf", True)
       self.mappingMode = rospy.get_param("~mapping_mode", True)
       self.useExternalPose = rospy.get_param("~use_external_pose", False)
       self.initialMapFileName = os.path.expanduser(rospy.get_param("~initial_map_file", ""))
       self.mapFileName = os.path.expanduser(rospy.get_param("~map_file", "map.txt"))
       self.obsFileName = os.path.expanduser(rospy.get_param("~obs_file", "obs.txt"))
       self.transFileName = os.path.expanduser(rospy.get_param("~trans_file", "trans.txt"))
       self.fiducialsAreLevel = rospy.get_param("~fiducials_are_level", True)
       self.medianFilterSamples = rospy.get_param("~median_filter_samples", 0)
       mkdirnotex(self.obsFileName)
       mkdirnotex(self.transFileName)
       # How much to future date our tfs
       self.future = rospy.get_param("~future", 0.0)
       # Republish tf
       self.republishTf = rospy.get_param("~republish_tf", True)
       print "frames: odom", self.odomFrame, "map:", self.mapFrame, "pose", self.poseFrame
       self.obsFile = open(self.obsFileName, "a")
       self.transFile = open(self.transFileName, "a")
       self.currentSeq = None
       self.imageTime = None
       self.tfs = {}
       self.numFiducials = 0
       self.mapPublished = False
       if self.sendTf:
           self.br = tf2_ros.TransformBroadcaster()
       self.tfBuffer = tf2_ros.Buffer(rospy.Time(30))
       self.lr = tf2_ros.TransformListener(self.tfBuffer)
       self.markerPub = rospy.Publisher("fiducials", Marker, queue_size=20)
       self.publishLock = threading.Lock()
       self.visibleMarkers = {}
       self.pose = None
       self.robotQuat = None
       self.robotXyz = None
       self.robotYaw = 0.0
       self.lastUpdateXyz = None
       self.lastUpdateYaw = None
       self.map = Map(self.mapFileName, self.initialMapFileName)
       self.position = None
       self.positionHistory = []
       self.posePub = rospy.Publisher("/fiducial_pose", PoseWithCovarianceStamped, queue_size=1)
       self.mapPub = rospy.Publisher("/fiducial_map", FiducialMapEntryArray, queue_size=100)
       rospy.Service('initialize_fiducial_map', InitializeMap, self.map.initialize)
       rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.newTf)


    def close(self):
        self.map.save()
        self.map.publish(self.mapPub)
        self.obsFile.close()
        self.transFile.close()


    """
    Called when a FiducialTransformArray is received
    """
    def newTf(self, msg):
        self.currentSeq = msg.image_seq
        imageTime = msg.header.stamp
        self.imageTime = imageTime
        tfs = {}
        rospy.loginfo("got tfs from image seq %d", self.currentSeq)

        numKnown = 0
        numUnknown = 0
        mapUpdated = False
        for m in msg.transforms:
            id = m.fiducial_id
            trans = m.transform.translation
            rot = m.transform.rotation
            mat = numpy.dot(translation_matrix((trans.x, trans.y, trans.z)),
                            quaternion_matrix((rot.x, rot.y, rot.z, rot.w)))
            tfs[id] = Observation(mat, m.object_error, m.image_error, msg.header.frame_id)
            self.transFile.write("%d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n" % \
                                 (id, self.currentSeq, trans.x, trans.y, trans.z,
                                  rot.x, rot.y, rot.z, rot.w,
                                  m.object_error, m.image_error, m.fiducial_area))
            if self.map.has_key(id):
                numKnown += 1
            else:
                numUnknown += 1
        self.tfs = tfs
        if numKnown == 0 and numUnknown == 0:
            return
        if numUnknown > 0 and numKnown > 0:
            self.updateMap(tfs)
            mapUpdated = True
        if self.useExternalPose and numKnown == 0:
                # Add fiducial pose from external (eg AMCL) localization
                f = tfs.keys()[0]
                self.updateMapFromExternal(tfs, f, imageTime, self.mapFrame)
        if numKnown == 0 and len(self.map.keys()) == 0:
                # Auto initialize map from one of the fiducials
                f = tfs.keys()[0]
                self.updateMapFromExternal(tfs, f, imageTime, self.odomFrame)
        self.updatePose(tfs, imageTime)
        if not self.pose is None:
            #robotXyz = numpy.array(translation_from_matrix(self.pose))[:3]
            #robotQuat = numpy.array(quaternion_from_matrix(self.pose))
            (r, p, robotYaw) = euler_from_quaternion(robotQuat)
            # Only update the map if the robot has moved significantly, to
            # avoid the map variances decaying from repeated observations
            if self.lastUpdateXyz is None or mapUpdated:
                if not mapUpdated:
                    self.updateMap(tfs)
                self.lastUpdateXyz = self.robotXyz
                self.lastUpdateYaw = robotYaw
            else:
                dist = numpy.linalg.norm(self.lastUpdateXyz - robotXyz)
                angle = self.lastUpdateYaw - self.robotYaw
                print "Distance moved", dist, angle
                if self.mappingMode or dist > MIN_UPDATE_TRANSLATION \
                   or angle > MIN_UPDATE_ROTATION:
                    self.updateMap(tfs)
                    self.lastUpdateXyz = self.robotXyz
                    self.lastUpdateYaw = robotYaw

    """
    Update the map with fiducial pairs
    """
    def updateMap(self, tfs):
        for f1 in tfs.keys():
            for f2 in tfs.keys():
                if f1 == f2:
                    continue
                if not self.map.has_key(f1):
                    continue
                self.updateMapPair(tfs, f1, f2)

    """
    Update the map with the new transform between the fiducial pair
    f1 and f2
    """
    def updateMapPair(self, tfs, f1, f2):
        fid1 = self.map[f1]

        # Don't update ground truth fidicuals
        if self.map.has_key(f2):
            if self.map[f2].variance == 0.0:
                return

        # Don't update f2 if the only estimate of f1 came from it
        if len(fid1.links) == 1 and f1 in fid1.links:
            return

        P_fid1 = fid1.pose44()

        obs1 = tfs[f1]
        obs2 = tfs[f2]

        # transform form fiducial f1 to f2
        T_fid1Fid2 = numpy.dot(obs1.T_fidCam, obs2.T_camFid)

        # pose of f1 transformed by trans
        P_fid2 = numpy.dot(P_fid1, T_fid1Fid2)

        xyz = numpy.array(translation_from_matrix(P_fid2))[:3]
        quat = numpy.array(quaternion_from_matrix(P_fid2))
        (r, p, yaw) = euler_from_quaternion(quat)

        self.obsFile.write("%d %d %d %r %r %r %r %r %r %r %r %r %r\n" % \
                               (self.currentSeq, f2, f1,
                                xyz[0], xyz[1], xyz[2],
                                rad2deg(r), rad2deg(p), rad2deg(yaw),
                                obs1.objectError, obs1.imageError, obs2.objectError, obs2.imageError))

        addedNew = False
        if not self.map.has_key(f2):
            self.map[f2] = Fiducial(f2)
            addedNew = True
            rospy.loginfo("New fiducial %s" % f2)

        fid2 = self.map[f2]

        variance  = angularError3D(r, p , yaw)
        # and take into account the variance of the reference fiducial
        # we convolve the gaussians, which is achieved by adding the variances
        print "*** %f var %f %f" % (f1, self.map[f1].variance, variance)
        variance = variance + self.map[f1].variance

        if self.fiducialsAreLevel:
            (r1, p1, y1) = euler_from_quaternion(fid1.orientation)
            (r, p, yaw) = euler_from_quaternion(quat)
            quat = quaternion_from_euler(r1, p1, yaw)

        self.map[f2].update(xyz, quat, variance)

        rospy.loginfo("%d updated to %.3f %.3f %.3f %.3f %.3f %.3f %.3f" % (f2, xyz[0], xyz[1], xyz[2],
            rad2deg(r), rad2deg(p), rad2deg(yaw), variance))

        p = self.map[f2].position

        if self.mappingMode or addedNew:
            self.map.save()
            self.map.publish(self.mapPub)

        if not f1 in fid2.links:
            fid2.links.append(f1)
        if not f2 in fid1.links:
            fid1.links.append(f2)

    """
    Update the map with a new fiducial based on an external robot pose
    """
    def updateMapFromExternal(self, tfs, f, imageTime, worldFrame):
        rospy.logerr("update from external %d" % f)
        obs = tfs[f]
        try:
            trans = self.tfBuffer.lookup_transform(worldFrame, obs.cameraFrame, imageTime)
            ct = trans.transform.translation
            cr = trans.transform.rotation
            T_worldCam = numpy.dot(translation_matrix((ct.x, ct.y, ct.z)),
                              quaternion_matrix((cr.x, cr.y, cr.z, cr.w)))
        except tf2_ros.TransformException:
            rospy.logerr("Unable to lookup transfrom from world to camera (%s to %s) at %s" % \
                         (worldFrame, obs.cameraFrame, imageTime))
            return


        P_fid = numpy.dot(T_worldCam, obs.T_camFid)

        self.map[f] = Fiducial(f)

        xyz = numpy.array(translation_from_matrix(P_fid))[:3]
        quat = numpy.array(quaternion_from_matrix(P_fid))
        (r, p, yaw) = euler_from_quaternion(quat)

        variance = 0.0 # TODO:find something better
        self.map[f].update(xyz, quat, variance)

        print "%d updated from external to %.3f %.3f %.3f %.3f %.3f %.3f %.3f" % (f, xyz[0], xyz[1], xyz[2],
            rad2deg(r), rad2deg(p), rad2deg(yaw), variance)

        p = self.map[f].position
        self.map.save()
        self.map.publish(self.mapPub)


    """
    Estimate the pose of the camera from the fiducial to camera
    transforms in tfs
    """
    def updatePose(self, tfs, imageTime):
        position = None
        orientation = None
        camera = None

        for t in tfs.keys():
            """
            if not t == 100:
                continue
            """
            if not self.map.has_key(t):
                rospy.logwarn("No path to %d" % t)
                continue

            obs = tfs[t]

            """
            try:
                trans = self.tfBuffer.lookup_transform(obs.cameraFrame,
                                                 self.poseFrame,
                                                 imageTime)
            except tf2_ros.TransformException:
                rospy.logerr("Unable to lookup transfrom from camera to robot (%s to %s) at %s" % \
                             (obs.cameraFrame, self.poseFrame, imageTime))
                return

            ct = trans.transform.translation
            cr = trans.transform.rotation
            T_camBase = numpy.dot(translation_matrix((ct.x, ct.y, ct.z)),
                                  quaternion_matrix((cr.x, cr.y, cr.z, cr.w)))

            """
            self.map[t].lastSeenTime = imageTime

            T_worldFid = self.map[t].pose44()
            T_worldCam = numpy.dot(T_worldFid, obs.T_fidCam)
            #T_worldCam = obs.T_camFid
            #T_worldBase = numpy.dot(T_worldCam, T_camBase)
            T_worldBase = T_worldCam ### For debugging

            xyz = numpy.array(translation_from_matrix(T_worldBase))[:3]
            quat = numpy.array(quaternion_from_matrix(T_worldBase))
            (r, p, y) = euler_from_quaternion(quat)

            if self.fiducialsAreLevel:
                quat = quaternion_from_euler(0, 0, y)

            thisvar  = angularError3D(r, p , 0.0)
            rospy.loginfo("pose %d %f %f %f %f %f %f %f" % \
                             (t, xyz[0], xyz[1], xyz[2],
                              rad2deg(r), rad2deg(p), rad2deg(y),
                              thisvar))

            if position is None:
                position = xyz
                orientation = quat
                variance = thisvar
                print "Position is None", position, variance
            else:
                position, v1 = updateLinear(position, variance,
                                       xyz, thisvar)
                print position, v1
                orientation, v2 = updateAngular(orientation, variance,
                                                quat, thisvar)
                variance = v1

        if self.medianFilterSamples > 0:
            self.positionHistory.append(position)
            if len(self.positionHistory) > self.medianFilterSamples:
                self.positionHistory.pop(0)
            position = numpy.median(self.positionHistory, axis=0)

        if not position is None:
            (r, p, y) = euler_from_quaternion(orientation)
            xyz = position
            rospy.loginfo("pose ALL %f %f %f %f %f %f %f %d" % \
                            (xyz[0], xyz[1], xyz[2],
                             rad2deg(r), rad2deg(p), rad2deg(y),
                             variance, self.currentSeq))
            """
            self.pose = numpy.dot(quaternion_matrix((orientation[0],
                                                     orientation[1],
                                                     orientation[2])),
                                  translation_matrix((position[0],
                                                      position[1],
                                                      0)))
            """
            self.robotXyz = position
            self.robotQuat = orientation
            self.computeTransform(imageTime)

    def makeMarker(self, fiducialId, visible=False):
        fiducial = self.map[fiducialId]
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
            if self.map.has_key(fid2):
                position2 = self.map[fid2].position
                links.points.append(Point(position[0], position[1], position[2]))
                links.points.append(Point(position2[0], position2[1], position2[2]))
        links.id = fiducialId + 20000
        links.ns = "fiducial_namespace_link"

        return marker, text, links


    def publishMarker(self, fiducialId, visible=False):
        try:
            marker, text, links = self.makeMarker(fiducialId, visible)
            self.markerPub.publish(marker)
            self.markerPub.publish(text)
            self.markerPub.publish(links)
        except:
            rospy.loginfo("Problem publishing marker %s" % fiducialId)


    """
    Publish the next unpublished marker in the map.
    This should be called repeatedly until all markers are published
    """
    def publishNextMarker(self):
        if self.mapPublished:
            return
        for fid in self.map.keys():
            if not self.map.fiducials[fid].publishedMarker:
                self.publishMarker(fid)
                self.map.fiducials[fid].publishedMarker = True
                return
        self.mapPublished = True
        print "Markers published"

    """
    Update markers with visibility colors
    """
    def publishMarkers(self, tfs):
        self.publishNextMarker()
        t = self.imageTime
        for fid in tfs.keys():
            if self.map.has_key(fid):
                self.visibleMarkers[fid] =  True
        for fid in self.visibleMarkers.keys():
            if (t - self.map[fid].lastSeenTime).to_sec() > UNSEEN_TIME:
                self.publishMarker(fid, False)
                del self.visibleMarkers[fid]
            else:
                self.publishMarker(fid, True)


    """
    Publish the transform in self.pose
    """
    def computeTransform(self, imageTime):
        if self.pose is None:
            return
        robotXyz = self.robotXyz #numpy.array(translation_from_matrix(self.pose))[:3]
        robotQuat = self.robotQuat #numpy.array(quaternion_from_matrix(self.pose))
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
                trans = self.tfBuffer.lookup_transform(self.poseFrame, self.odomFrame, imageTime)
                ct = trans.transform.translation
                cr = trans.transform.rotation
                odom = numpy.dot(translation_matrix((ct.x, ct.y, ct.z)),
                              quaternion_matrix((cr.x, cr.y, cr.z, cr.w)))
                pose = numpy.dot(self.pose, odom)
            except tf2_ros.TransformException:
                rospy.logerr("Unable to lookup transfrom from odom to robot (%s to %s) at %s" % \
                             (self.poseFrame, self.odomFrame, self.imageTime))
                return
        else:
            pose = self.pose
        self.publishLock.acquire()
        #self.robotXyz = numpy.array(translation_from_matrix(pose))[:3]
        robotQuat = numpy.array(quaternion_from_matrix(pose))
        (r, p, yaw) = euler_from_quaternion(robotQuat)
        #self.robotQuat = quaternion_from_euler(0.0, 0.0, yaw)
        self.robotYaw = yaw
        self.publishLock.release()
        self.publishTransform()

    def publishTransform(self):
        if self.sendTf and not self.robotXyz is None:
            self.publishLock.acquire()
            t = TransformStamped()
            if self.odomFrame != "":
                t.child_frame_id = self.odomFrame
                t.header.frame_id = self.mapFrame
            else:
                t.child_frame_id = self.poseFrame
                t.header.frame_id = self.mapFrame
            t.header.stamp = rospy.Time.now() + rospy.Duration(self.future)
            t.transform.translation.x = self.robotXyz[0]
            t.transform.translation.y = self.robotXyz[1]
            t.transform.translation.z = 0 #####self.robotXyz[2]
            t.transform.rotation.x = self.robotQuat[0]
            t.transform.rotation.y = self.robotQuat[1]
            t.transform.rotation.z = self.robotQuat[2]
            t.transform.rotation.w = self.robotQuat[3]
            self.br.sendTransform(t)
            self.publishLock.release()

    def run(self):
        hz = 10.0
        rospy.loginfo("Fiducial Slam started")
        rate = rospy.Rate(hz)
        tick = 0
        while not rospy.is_shutdown():
            if self.republishTf:
                self.publishTransform()
            self.publishMarkers(self.tfs)
            tick += 1
            if (tick % 10) == 0:
                self.map.publish(self.mapPub)
            try:
                rate.sleep()
            except:
                pass
        self.close()
        rospy.loginfo("Fiducial Slam ended")

if __name__ == "__main__":
    node = FiducialSlam()
    node.run()
