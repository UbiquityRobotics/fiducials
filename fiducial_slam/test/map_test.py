#!/usr/bin/python

from __future__ import print_function

import sys
import threading
import time
import unittest
import os
import math

import rclpy
from rclpy.node import Node

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
        self.node = Node("test_map")

        self.map_subscription = self.node.create_subscription(FiducialMapEntryArray, "/fiducial_map", self.mapCallback, 1)
        self.pose_subscription = self.node.create_subscription(PoseWithCovarianceStamped,"/fiducial_pose", self.poseCallback, 1)

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
        self.node.declare_parameter("min_lines", 1)
        minLines = self.node.get_parameter('min_lines').value

        expect = self.node.declare_parameter("expect", "")
        expect = self.node.get_parameter('expect').value

        self.node.declare_parameter("expected_pose", "")
        expectedPose = self.node.get_parameter('expected_pose').value

        t = 0
        while True:
            rclpy.spin_once(self.node) #Spin the node to receive callbacks
            if self.mapEntries >= minLines and self.pose != None: #If there are map entries and a pose is set
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
                        if not fid in self.map:
                            self.fail("Fiducial %d not in map" % fid)
                        fiducial = self.map[fid]
                        for i in range(len(ex)):
                            if abs(float(ex[i]) - float(fiducial[i])) > EPSILON:
                                self.fail("fiducial %d %s expected %s" % \
                                  (fid, fiducial, ex,))
                return
            #time.sleep(0.5)

def main(args=None):
    sys.argv = [sys.argv[0]]
    unittest.main()

if __name__ == "__main__":
    rclpy.init()
    main()
