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

"""
A node to verify the vertices of a fiducial detected by fiducial_detect
"""

class VerticesTest(unittest.TestCase):
    def __init__(self, *args):
        super(VerticesTest, self).__init__(*args)

    def callback(self, msg):
        self.message_received = True         
        print("Message received")
        print("Fiducial msg: image %d fiducial %d", msg.image_seq, msg.fiducial_id)
        # TODO: vertify the vertex coordinates

    def test_vertices(self):
        rospy.init_node('test_vertices')
        self.message_received = False
        print("test_vertices");
        sub = rospy.Subscriber('/fiducial_detect/vertices', Fiducial, self.callback)
        t = 0
        while t < TIMEOUT and not self.message_received:
            time.sleep(1)
            t += 1
        if self.message_received:
            print("Success")
        else:
            self.fail("No message recieved")
        
if __name__ == '__main__':
    try:
        rostest.run('rostest', NAME, VerticesTest, sys.argv)
    except KeyboardInterrupt:
        pass
print("exiting")
