#!/usr/bin/python

"""
Simultaneous location and mapping based on fiducial poses.
Receives FiducialTransform messages and builds a map and estimates the
camera pose from them
"""


import rospy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_slerp, \
                               translation_matrix, quaternion_matrix, \
                               translation_from_matrix, quaternion_from_matrix, \
                               quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from fiducials_ros.msg import Fiducial
from ros_rpp.msg import FiducialTransform
from json import dumps
import tf
from math import pi, sqrt
import rosbag
import sys
import os
import traceback
import math
import numpy


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
       rospy.Subscriber("/fiducial_transforms", FiducialTransform, self.newTf)
       self.mapFile = rospy.get_param("map_file", "map.txt")
       self.tfs = {}
       self.currentSeq = None
       self.fiducials = {}
       #self.addOriginFiducial(543)
       self.br = tf.TransformBroadcaster()
       self.loadMap()

    def addOriginFiducial(self, id):
       pose = numpy.zeros(3),
       fid = Fiducial(id)
       fid.position = numpy.zeros(3)
       fid.orientation = numpy.array((0.0, 0.0, 0.0, 1.0))
       fid.variance = 0.0
       fid.observations = 1
       self.fiducials[id] = fid

    def saveMap(self):
        print "** save map **"
        file = open(self.mapFile, "w")
        fids = self.fiducials.keys()
        fids.sort()
        for fid in fids:
            f = self.fiducials[fid]
            pos = f.position
            (r, p, y) = euler_from_quaternion(f.orientation)
            file.write("%d %d %lf %lf %lf %lf %lf %lf %lf\n" % \
              (f.id, f.observations, pos[0], pos[1], pos[2],
               rad2deg(r), rad2deg(p), rad2deg(y), f.variance))
        file.close()

    def loadMap(self):
        file = open(self.mapFile, "r")
        for line in file.readlines():
            words = line.split()
            print words
            fid = int(words[0])
            f = Fiducial(fid)
            f.position = numpy.array((float(words[1]), float(words[2]), float(words[3])))
            f.orientation = quaternion_from_euler(deg2rad(words[4]), deg2rad(words[5]), deg2rad(words[6]))
            f.variance = float(words[7])
            f.observations = int(words[8])
            self.fiducials[fid] = f
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
        self.tfs[id] = (mat, m.object_error, m.image_error, m.fiducial_area)
    
        
    """
    Update the map with the new transform between the fiducial pair
    f1 and f2
    """
    def updateMap(self, f1, f2):
        if self.fiducials.has_key(f2):
            if self.fiducials[f2].variance == 0.0:
                return

        posef1 = self.fiducials[f1].pose44()

        (trans1, oerr1, ierr1, area1) = self.tfs[f1]
        (trans2, oerr2, ierr2, area2) = self.tfs[f2]

        # transform form fiducial f1 to f2
        trans = numpy.dot(numpy.linalg.inv(trans1), trans2)
        
        # pose of f1 transformed by trans
        posef2 = numpy.dot(posef1, trans)

        xyz = numpy.array(translation_from_matrix(trans))[:3]
        quat = numpy.array(quaternion_from_matrix(trans))
        (r, p, yaw) = euler_from_quaternion(quat)

        print "new %d from %d %d %.3f %.3f %.3f %f %f %f %f %f %f %f %f %f" % \
            (f2, f1, self.currentSeq, xyz[0], xyz[1], xyz[2], 
             rad2deg(r), rad2deg(p), rad2deg(yaw),
             oerr1, ierr1, oerr2, ierr2, area1, area2)
            
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

            (trans, oerr, ierr, area) = self.tfs[t]
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
        # TODO: publish

if __name__ == "__main__":
    node = FiducialSlam()
    rospy.spin()
    print "end spin"
    node.saveMap()
