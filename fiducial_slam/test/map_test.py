#!/usr/bin/python

from __future__ import print_function

import sys
import threading
import time
import unittest
import os
import math

import rospy
import rostest

from fiducial_msgs.msg import FiducialMapEntryArray
from geometry_msgs.msg import PoseWithCovarianceStamped

NAME='map'
EPSILON=0.1

"""
A node to verify the map created by fiducial_slam. It will verify
that the specified map file contains at least min_lines lines,
and if the expect parameter is also given, it will check that
the position of the specfied fiducial. In addition, the pose
specified in expected_pose is verified
"""

def rad2deg(rad):
    return rad * 180.0 / math.pi

class MapTest(unittest.TestCase):
    def __init__(self, *args):
        super(MapTest, self).__init__(*args)
        self.mapEntries = -1
        self.map = {}
        self.pose = None

    def mapCallback(self, msg):
        self.mapEntries = len(msg.fiducials)
        for fid in msg.fiducials:
            self.map[fid.fiducial_id] = (fid.x, fid.y, fid.z,
               rad2deg(fid.rx), rad2deg(fid.ry), rad2deg(fid.rz))

    def poseCallback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.pose = (position.x, position.y, position.z,
                     orientation.x, orientation.y, orientation.z, orientation.w)

    def test_map(self):
        rospy.init_node('test_map')
        minLines = rospy.get_param("~min_lines", 1)
        expect = rospy.get_param("~expect", "")
        expectedPose = rospy.get_param("~expected_pose", "")

        rospy.Subscriber("/fiducial_map", FiducialMapEntryArray,
                         self.mapCallback)
        rospy.Subscriber("/fiducial_pose", PoseWithCovarianceStamped,
                         self.poseCallback)

        print("test_map");
        t = 0
        while True:
            if self.mapEntries >= minLines and self.pose != None:
                if expectedPose != "":
                    ep = expectedPose.split()
                    for i in range(len(ep)):
                        if abs(float(ep[i]) - float(self.pose[i])) > EPSILON:
                            self.fail("pose %s" % (self.pose,))
                if expect != "":
                    lines = expect.split(',')
                    for line in lines:
                        ex = line.split()
                        fid = int(ex[0])
                        ex = ex[1:]
                        if not self.map.has_key(fid):
                            self.fail("Fiducial %d not in map" % fid)
                        fiducial = self.map[fid]
                        for i in range(len(ex)):
                            if abs(float(ex[i]) - float(fiducial[i])) > EPSILON:
                                self.fail("fiducial %d %s expected %s" % \
                                  (fid, fiducial, ex,))
                return
	    time.sleep(0.5)

if __name__ == '__main__':
    try:
        rostest.run('rostest', NAME, MapTest, sys.argv)
    except KeyboardInterrupt:
        pass
    print("exiting")
