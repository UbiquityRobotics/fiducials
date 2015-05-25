#!/usr/bin/python

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
from visualization_msgs.msg import Marker

from fiducial_pose.msg import Fiducial
from fiducial_pose.msg import FiducialTransform

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
       rospy.init_node('fiducials_slam')
       self.odomFrame = rospy.get_param("~odom_frame", "")
       self.poseFrame = rospy.get_param("~pose_frame", "base_link")
       self.mapFrame = rospy.get_param("~map_frame", "map")
       self.cameraFrame = rospy.get_param("~camera_frame", "camera")
       self.sendTf = rospy.get_param("~publish_tf", True)
       self.mappingMode = rospy.get_param("~mapping_mode", True)
       self.mapFileName = rospy.get_param("~map_file", "map.txt")
       self.obsFileName = rospy.get_param("~obs_file", "obs.txt")
       self.transFileName = rospy.get_param("~trans_file", "trans.txt")
       self.obsFile = open(self.obsFileName, "a")
       self.transFile = open(self.transFileName, "a")
       self.tfs = {}
       self.currentSeq = None
       self.numFiducials = 0
       self.fiducials = {}
       #self.addOriginFiducial(543)
       self.br = tf.TransformBroadcaster()
       self.lr = tf.TransformListener()
       self.markerPub = rospy.Publisher("fiducials", Marker)
       self.visibleMarkers = {}
       self.mapPublished = False
       self.numFiducialsVisible = 0
       self.threadLock = threading.Lock()
       self.pose = None
       self.robotQuat = None
       self.robotXyz = None
       self.robotYaw = 0.0
       self.lastUpdateXyz = None
       self.lastUpdateYaw = None
       self.lastTfPubTime = rospy.get_time()
       self.loadMap()
       self.showVertices()
       self.position = None
       rospy.Subscriber("/fiducial_transforms", FiducialTransform, self.newTf)
       self.posePub = rospy.Publisher("/fiducial_pose", PoseWithCovarianceStamped)


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
            print fid, translation_from_matrix(off)[:3]
            off = numpy.dot(translation_matrix(numpy.array((1.0, -1.0, 0.0))), f.pose44())
            print fid, translation_from_matrix(off)[:3]
            off = numpy.dot(translation_matrix(numpy.array((1.0, 1.0, 0.0))), f.pose44())
            print fid, translation_from_matrix(off)[:3]
            off = numpy.dot(translation_matrix(numpy.array((-1.0, 1.0, 0.0))), f.pose44())
            print fid, translation_from_matrix(off)[:3]

    """
    Called when a FiducialTransform is received
    """
    def newTf(self, m):
        seq = m.image_seq
        if seq != self.currentSeq:
            """
            If this is a new frame, process pairs tfs from the previous one
            """
            self.updatePose()
            # Only update the map if the robot has moved significantly, to 
            # avoid the map variances decaying from repeated observations
            if self.lastUpdateXyz is None:
                self.updateMap()
                self.lastUpdateXyz = self.robotXyz
                self.lastUpdateYaw = self.robotYaw
            else:
                dist = numpy.linalg.norm(self.lastUpdateXyz - self.robotXyz)
                angle = self.lastUpdateYaw - self.robotYaw
                print "Distance moved", dist, angle
                if self.mappingMode or dist > MIN_UPDATE_TRANSLATION or angle > MIN_UPDATE_ROTATION:
                    self.updateMap()
                    self.lastUpdateXyz = self.robotXyz
                    self.lastUpdateYaw = self.robotYaw
            self.threadLock.acquire()
            self.numFiducialsVisible = len(self.tfs.keys())
            self.publishMarkers()
            self.threadLock.release()
            self.tfs = {}
            self.currentSeq = seq
        """ 
        This may be called multiple times per frame, so we store the tf
        """
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
    Update the map with fiducial pairs
    """
    def updateMap(self):
        for t1 in self.tfs.keys():
            for t2 in self.tfs.keys():
                if t1 == t2:
                    continue
                if not self.fiducials.has_key(t1):
                    continue
                self.updateMapPair(t1, t2)

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

        posef1 = fid1.pose44()

        (trans1, trans1Inv, oerr1, ierr1) = self.tfs[f1]
        (trans2, trans2Inv, oerr2, ierr2) = self.tfs[f2]

        # transform form fiducial f1 to f2
        trans = numpy.dot(trans1Inv, trans2)
                             
        # pose of f1 transformed by trans
        posef2 = numpy.dot(posef1, trans)

        xyz = numpy.array(translation_from_matrix(posef2))[:3]
        quat = numpy.array(quaternion_from_matrix(posef2))
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
    Estimate the pose of the camera from the fiducial to camera
    transforms in self.tfs
    """
    def updatePose(self):
        position = None
        orientation = None
        camera = None
        try:
            t = self.lr.getLatestCommonTime(self.poseFrame, self.cameraFrame)
            camt, camr = self.lr.lookupTransform(self.poseFrame, self.cameraFrame, t)
            camera = numpy.dot(translation_matrix((camt[0], camt[1], camt[2])),
                     quaternion_matrix((camr[0], camr[1], camr[2], camr[3])))
        except:
            rospy.logerr("Unable to lookup transfrom from camera to robot")
        
        for t in self.tfs.keys():
            if not self.fiducials.has_key(t):
                rospy.logwarn("No path to %d" % t)
                continue
            (trans, invTrans, oerr, ierr) = self.tfs[t]
            posef1 = self.fiducials[t].pose44()

            txpose = numpy.dot(posef1, invTrans)
            if not camera is None:
                txpose = numpy.dot(txpose, camera)

            xyz = numpy.array(translation_from_matrix(txpose))[:3]
            quat = numpy.array(quaternion_from_matrix(txpose))

            (r, p, y) = euler_from_quaternion(quat)
            thisvar  = angularError3D(r, p , 0.0)
            print "pose %d %f %f %f %f %f %f %f" % (t, xyz[0], xyz[1], xyz[2],
                                                 rad2deg(r), rad2deg(p), rad2deg(y),
                                                 thisvar)

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
            print "pose ALL %f %f %f %f %f %f %f %d" % (xyz[0], xyz[1], xyz[2],
                                                  rad2deg(r), rad2deg(p), rad2deg(y), 
                                                  variance, self.currentSeq)
            self.publishTransform(position, orientation)

    def publishMarkers(self):
        self.numFiducials = len(self.tfs.keys())
        for fid in self.tfs.keys():
            if self.fiducials.has_key(fid):
	        self.publishMarker(fid)
        for fid in self.visibleMarkers.keys():
	    self.publishMarker(fid)

    def publishMarker(self, fiducialId):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        fiducial = self.fiducials[fiducialId]
        position = fiducial.position
        marker.pose = Pose(Point(position[0], position[1], position[2]),
                           Quaternion(0, 0, 0, 1))
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        if self.tfs.has_key(fiducialId):
            marker.color = ColorRGBA(1, 0, 0, 1)
            self.visibleMarkers[fiducialId] = True
        else:
            marker.color = ColorRGBA(0, 1, 0, 1)
            if self.visibleMarkers.has_key(fiducialId):
                del self.visibleMarkers[fiducialId]
        marker.id = fiducialId
        marker.ns = "fiducial_namespace"
        marker.header.frame_id = "/map"
        self.markerPub.publish(marker)

        marker.pose.position.z += (marker.scale.z/2.0) + 0.05  # draw text above marker
        marker.color.r = marker.color.g = marker.color.b = 1.0 # white
        marker.scale.y = marker.scale.z = 0.1
        marker.id = fiducialId + 10000
        marker.ns = "fiducial_namespace_text"
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = str(fiducialId)
        self.markerPub.publish(marker)

        marker.type = Marker.LINE_LIST
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.color = ColorRGBA(0, 0, 1, 1)
        marker.scale.x = 0.05
        for fid2 in fiducial.links:
            if self.fiducials.has_key(fid2):
                position2 = self.fiducials[fid2].position
                marker.points.append(Point(position[0], position[1], position[2]))
                marker.points.append(Point(position2[0], position2[1], position[2]))
                marker.id = fiducialId + 20000
                marker.ns = "fiducial_namespace_link"
        self.markerPub.publish(marker)
            

    """
    Publish the transform
    """
    def publishTransform(self, trans, rot):
        pose = numpy.dot(translation_matrix((trans[0], trans[1], trans[2])),
                         quaternion_matrix((rot[0], rot[1], rot[2], rot[3])))

        robotXyz = numpy.array(translation_from_matrix(pose))[:3]
        robotQuat = numpy.array(quaternion_from_matrix(pose))
        (r, p, yaw) = euler_from_quaternion(robotQuat)
        robotQuat = quaternion_from_euler(0.0, 0.0, yaw) 
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.mapFrame
        m.header.stamp = rospy.Time.now()
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
        m.pose.covariance = [0.1,  0,    0,     0,     0,     0,
                             0,    0.1,  0,     0,     0,     0,
                             0,    0,    0.1,   0,     0,     0,
                             0,    0,    0,     0.1,   0,     0,
                             0,    0,    0,     0,     0.1,   0,
                             0,    0,    0,     0,     0,     0.1]
        self.posePub.publish(m)
        self.threadLock.acquire()
        if self.odomFrame != "":
            t = self.lr.getLatestCommonTime(self.poseFrame, self.odomFrame)
            odomt, odomr = self.lr.lookupTransform(self.poseFrame, self.odomFrame, t)
            odom = numpy.dot(translation_matrix((odomt[0], odomt[1], odomt[2])),
                             quaternion_matrix((odomr[0], odomr[1], odomr[2], odomr[3])))
            pose = numpy.dot(pose, odom)
        robotXyz = numpy.array(translation_from_matrix(pose))[:3]
        robotQuat = numpy.array(quaternion_from_matrix(pose))
        (r, p, yaw) = euler_from_quaternion(robotQuat)
        robotQuat = quaternion_from_euler(0.0, 0.0, yaw) 
        self.pose = pose
        self.robotYaw = yaw
        self.robotQuat = robotQuat
        self.robotXyz = robotXyz
        self.threadLock.release()
        self.sendTransform()


    def sendTransform(self):
        self.threadLock.acquire()
        if not self.mapPublished:
            self.mapPublished = True
            print "publishing map"
            for fid in self.fiducials.keys():
               rospy.sleep(.1)
               self.publishMarker(fid)
            print "published map"
        if self.numFiducialsVisible == 0:
            for fid in self.visibleMarkers.keys():
                if not self.tfs.has_key(fid):
                    self.publishMarker(fid)
        if (not self.pose == None) and self.sendTf:
            if self.odomFrame != "":
                frame = self.odomFrame
            else:
                frame = self.poseFrame
            self.br.sendTransform(self.robotXyz, self.robotQuat,
                                  rospy.Time.now(),
                                  frame,
                                  self.mapFrame)
            self.lastTfPubTime = rospy.get_time()
        self.threadLock.release()

if __name__ == "__main__":
    node = FiducialSlam()
    rospy.loginfo("Fiducial Slam started")
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        age = rospy.get_time() - node.lastTfPubTime
        if age > 0.1:
            node.sendTransform()
        rate.sleep()
    node.close()
    rospy.loginfo("Fiducial Slam ended")
