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
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, \
                              TransformStamped
from visualization_msgs.msg import Marker

from fiducials_ros.msg import Fiducial
from ros_rpp.msg import FiducialTransform

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
       self.mapFile = rospy.get_param("map_file", "map.txt")
       self.tfs = {}
       self.currentSeq = None
       self.fiducials = {}
       #self.addOriginFiducial(543)
       self.br = tf.TransformBroadcaster()
       self.lr = tf.TransformListener()
       self.markerPub = rospy.Publisher("fiducials", Marker)
       self.loadMap()
       self.robotXyz = None
       self.robotQuat = None
       self.timer = rospy.Timer(rospy.Duration(1.0/20.0), self.sendTransform)
       rospy.Subscriber("/fiducial_transforms", FiducialTransform, self.newTf)


    """
    Save current map to file
    """
    def saveMap(self):
        print "** save map **"
        file = open(self.mapFile, "w")
        fids = self.fiducials.keys()
        fids.sort()
        for fid in fids:
            f = self.fiducials[fid]
            pos = f.position
            (r, p, y) = euler_from_quaternion(f.orientation)
            file.write("%d %lf %lf %lf %lf %lf %lf %lf %d\n" % \
              (f.id, pos[0], pos[1], pos[2],
               rad2deg(r), rad2deg(p), rad2deg(y),
               f.variance, f.observations))
        file.close()

    """
    Load map from file
    """
    def loadMap(self):
        file = open(self.mapFile, "r")
        for line in file.readlines():
            words = line.split()
            fid = int(words[0])
            f = Fiducial(fid)
            f.position = numpy.array((float(words[1]), float(words[2]), float(words[3])))
            f.orientation = quaternion_from_euler(deg2rad(words[4]), deg2rad(words[5]), deg2rad(words[6]))
            f.variance = float(words[7])
            f.observations = int(words[8])
            self.fiducials[fid] = f
            self.publishMarker(fid)
        file.close()

    """
    Called when a FiducialTransform is received
    """
    def newTf(self, m):
        seq = m.image_seq
        if seq != self.currentSeq:
            """
            If this is a new frame, process pairs tfs from the previous one
            """
            for t1 in self.tfs.keys():
                for t2 in self.tfs.keys():
                    if t1 == t2:
                        continue
                    if not self.fiducials.has_key(t1):
                        continue
                    self.updateMap(t1, t2)
            self.updatePose()
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
        self.tfs[id] = (mat, m.object_error, m.image_error)
    
        
    """
    Update the map with the new transform between the fiducial pair
    f1 and f2
    """
    def updateMap(self, f1, f2):
        if self.fiducials.has_key(f2):
            if self.fiducials[f2].variance == 0.0:
                return

        posef1 = self.fiducials[f1].pose44()

        (trans1, oerr1, ierr1) = self.tfs[f1]
        (trans2, oerr2, ierr2) = self.tfs[f2]

        # transform form fiducial f1 to f2
        trans = numpy.dot(numpy.linalg.inv(trans1), trans2)
        
        # pose of f1 transformed by trans
        posef2 = numpy.dot(posef1, trans)

        xyz = numpy.array(translation_from_matrix(posef2))[:3]
        quat = numpy.array(quaternion_from_matrix(posef2))
        (r, p, yaw) = euler_from_quaternion(quat)

        print "new %d from %d %d %.3f %.3f %.3f %f %f %f %f %f %f %f" % \
            (f2, f1, self.currentSeq, xyz[0], xyz[1], xyz[2], 
             rad2deg(r), rad2deg(p), rad2deg(yaw),
             oerr1, ierr1, oerr2, ierr2)
            
        addedNew = False
        if not self.fiducials.has_key(f2):
            self.fiducials[f2] = Fiducial(f2)
            addedNew = True

        # We use the max of the two object errors, scaled, as our variance
        variance = max(oerr1, oerr2) * 100.0
        # and take into account the variance of the reference fiducial
        variance = max(variance, self.fiducials[f1].variance)
        self.fiducials[f2].update(xyz, quat, variance)

        p = self.fiducials[f2].position
        print "%d updated to %.3f %.3f %.3f" % (f2, p[0], p[1], p[2])

        if addedNew:
            self.saveMap()

        self.publishMarker(f2)

    """ 
    Estimate the pose of the camera from the fiducial to camera
    transforms in self.tfs
    """
    def updatePose(self):
        position = None
        orientation = None
        
        for t in self.tfs.keys():
            if not self.fiducials.has_key(t):
                continue

            (trans, oerr, ierr) = self.tfs[t]
            posef1 = self.fiducials[t].pose44()

            txpose = numpy.dot(posef1, trans,)
            xyz = numpy.array(translation_from_matrix(txpose))[:3]
            quat = numpy.array(quaternion_from_matrix(txpose))

            (r, p, y) = euler_from_quaternion(quat)
            print "pose %d %f %f %f %f %f %f" % (t, xyz[0], xyz[1], xyz[2],
                                                 rad2deg(r), 
                                                 rad2deg(p),
                                                 rad2deg(y))
            thisvar = oerr * 100.0
            thisvar = max(thisvar, self.fiducials[t].variance)

            if position is None:
                position = xyz
                orientation = quat
                variance = thisvar
            else:
                position, v1 = updateLinear(position, variance,
                                       xyz, oerr)
                orientation, v2 = updateAngular(orientation, variance,
                                                quat, thisvar)
                self.variance = v1
        if not position is None:
            (r, p, y) = euler_from_quaternion(orientation)
            xyz = position
            print "pose ALL %f %f %f %f %f %f" % (xyz[0], xyz[1], xyz[2],
                                                  rad2deg(r), 
                                                  rad2deg(p),
                                                  rad2deg(y))
            self.publishTransform(position, orientation)

    def publishMarker(self, fiducialId):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        position = self.fiducials[fiducialId].position
        marker.pose = Pose(Point(position[0], position[1], 0.0),
                           Quaternion(0, 0, 0, 1))
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color = ColorRGBA(1, 0, 0, 1)
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

    """
    Until we have proper sensor fusion in place, we publish a correction to odom, and do this in a timer,
    so that we can ron on odometry if no fiducials are visible
    """
    def publishTransform(self, trans, rot):
        odomt, odomr = self.lr.lookupTransform("odom", "base_footprint",
                                               rospy.Time(0))
        camt, camr = self.lr.lookupTransform("pgr_camera_frame", "base_link", 
                                             rospy.Time(0))
        odom = numpy.dot(translation_matrix((odomt[0], odomt[1], odomt[2])),
                         quaternion_matrix((odomr[0], odomr[1], odomr[2], odomr[3])))
        camera = numpy.dot(translation_matrix((camt[0], camt[1], camt[2])),
                           quaternion_matrix((camr[0], camr[1], camr[2], camr[3])))
        pose = numpy.dot(translation_matrix((trans[0], trans[1], trans[2])),
                         quaternion_matrix((rot[0], rot[1], rot[2], rot[3])))

        odomCorrection = numpy.linalg.inv(numpy.dot(pose, numpy.linalg.inv(odom)))
        self.robotXyz = numpy.array(translation_from_matrix(odomCorrection))[:3]
        self.robotXyz[2] = 0
        self.robotQuat = numpy.array(quaternion_from_matrix(odomCorrection))
        print self.robotXyz
        #tf2::Transform odom_correction = (odom_tf * pose.inverse()).inverse();

    def sendTransform(self, arg):
        if self.robotXyz is None:
            return
        self.br.sendTransform(self.robotXyz, self.robotQuat,
                              rospy.Time.now(),
                              "odom",
                              "map")

if __name__ == "__main__":
    node = FiducialSlam()
    rospy.loginfo("Fiducial Slam started")
    rospy.spin()
    print "end spin"
    node.saveMap()
