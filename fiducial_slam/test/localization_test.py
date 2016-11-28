#!/usr/bin/python

from __future__ import print_function

import sys
import threading
import time
import unittest

import rospy
import rostest

from geometry_msgs.msg import PoseWithCovarianceStamped

NAME='localization'
TIMEOUT=30
EPSILON=0.1

"""
A node to verify the pose output of fiducial_slam
"""


class LocalizationTest(unittest.TestCase):
    def __init__(self, *args):
        super(LocalizationTest, self).__init__(*args)

    def callback(self, msg):
        print("Message received")
        print(msg)
        self.received = True
        if abs(msg.pose.pose.position.x - self.expectedx) < EPSILON and \
           abs(msg.pose.pose.position.y - self.expectedy) < EPSILON:
            self.verified = True

    def test_localization(self):
        rospy.init_node('test_localization')
        self.received = False
        self.verified = False
        self.expectedx = rospy.get_param("~expectedx", 0) 
        self.expectedy = rospy.get_param("~expectedy", 0) 
        print("test_localization");
        sub = rospy.Subscriber("/fiducial_pose", PoseWithCovarianceStamped, self.callback)
        t = 0
        while t < TIMEOUT and not self.verified:
            time.sleep(0.5)
            t += 1
        if self.verified:
            print("Success")
        elif self.received:
            self.fail("Localization received but not equal to expected") 
        else:
            self.fail("Localization not received")

        
if __name__ == '__main__':
    try:
        rostest.run('rostest', NAME, LocalizationTest, sys.argv)
    except KeyboardInterrupt:
        pass
print("exiting")
