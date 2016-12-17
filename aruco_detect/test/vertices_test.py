#!/usr/bin/python

from __future__ import print_function

import sys
import threading
import time
import unittest

import rospy
import rostest

from fiducial_pose.msg import Fiducial
#from sensor_msgs.msg import CameraInfo

NAME='vertices'
TIMEOUT=30
EPSILON=1.0

"""
A node to verify the vertices of a fiducial detected by fiducial_detect
"""

class VertexSet:
    def __init__(self, fid, x0, y0, x1, y1, x2, y2, x3, y3):
        self.fid = int(fid)
        vertices = []
        vertices.append((float(x0), float(y0)))
        vertices.append((float(x1), float(y1)))
        vertices.append((float(x2), float(y2)))
        vertices.append((float(x3), float(y3)))
        vertices.sort()
        self.vertices = vertices

    def __repr__(self):
        return "fid %d %s %s %s %s %s %s %s %s" % (self.fid, self.vertices[0][0], self.vertices[0][1],
                                                         self.vertices[1][0], self.vertices[1][1],
                                                         self.vertices[2][0], self.vertices[2][1],
                                                         self.vertices[3][0], self.vertices[3][1])

    def compare(self, other):
        for i in range(0, 4):
            if abs(self.vertices[i][0] - other.vertices[i][0]) > EPSILON or \
               abs(self.vertices[i][1] - other.vertices[i][1]) > EPSILON:
              return False
        return True 


class VerticesTest(unittest.TestCase):
    def __init__(self, *args):
        super(VerticesTest, self).__init__(*args)

    def callback(self, msg):
        print("Message received: fid %d" % msg.fiducial_id)
        if msg.fiducial_id == self.expected.fid:
            actual = VertexSet(msg.fiducial_id, msg.x0, msg.y0, msg.x1, msg.y1,
                                                msg.x2, msg.y2, msg.x2, msg.y3)
            print("actual", actual)
            self.received = True
            if actual.compare(self.expected):
                self.verified = True

    def test_vertices(self):
        rospy.init_node('test_vertices')
        self.received = False
        self.verified = False
        expected = rospy.get_param("~expected", "")
        print("expected", expected)
        if expected:
            e = expected.split()
            self.expected = VertexSet(e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8])
            print("expected")
            print(expected)
        print("test_vertices");
        sub = rospy.Subscriber('/fiducial_vertices', Fiducial, self.callback)
        t = 0
        while t < TIMEOUT and not self.verified:
            time.sleep(0.5)
            t += 1
        if self.verified:
            print("Success")
        elif self.received:
            self.fail("Vertices received but not equal to expected") 
        else:
            self.fail("Vertices not received")

        
if __name__ == '__main__':
    try:
        rostest.run('rostest', NAME, VerticesTest, sys.argv)
    except KeyboardInterrupt:
        pass
print("exiting")