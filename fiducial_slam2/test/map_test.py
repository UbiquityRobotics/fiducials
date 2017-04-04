#!/usr/bin/python

from __future__ import print_function

import sys
import threading
import time
import unittest
import os

import rospy
import rostest

from fiducial_msgs.msg import FiducialMapEntryArray

NAME='map'
EPSILON=0.2

"""
A node to verify the map created by fiducial_slam. It will verify that the specified map file contains
at least min_lines lines, and if the expect parameter is also given, it will check that the position 
of the specfied fiducial.
"""

class MapTest(unittest.TestCase):
    def __init__(self, *args):
        super(MapTest, self).__init__(*args)
        self.mapEntries = -1

    def mapCallback(self, msg):
        print (msg)
        self.mapEntries = len(msg.fiducials)
        

    def test_map(self):
        rospy.init_node('test_map')
        filename = rospy.get_param("~map_file", "")
        minLines = rospy.get_param("~min_lines", 1)
        expect = rospy.get_param("~expect", "")

        rospy.Subscriber("/fiducial_map", FiducialMapEntryArray, self.mapCallback)

        print("test_map");
        try:
            os.remove(filename)
        except:
            pass
        t = 0
        while True:
            if self.mapEntries >= minLines:
                return
	    time.sleep(0.5)
        
if __name__ == '__main__':
    try:
        rostest.run('rostest', NAME, MapTest, sys.argv)
    except KeyboardInterrupt:
        pass
    print("exiting")
