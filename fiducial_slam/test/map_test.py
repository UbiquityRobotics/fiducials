#!/usr/bin/python

from __future__ import print_function

import sys
import threading
import time
import unittest

import rospy
import rostest

NAME='map'
TIMEOUT=30

"""
A node to verify the map created by fiducial_slam
"""

class MapTest(unittest.TestCase):
    def __init__(self, *args):
        super(MapTest, self).__init__(*args)

    def countlines(self, filename):
        file = open(filename, "r")
        count = len(file.readlines())
        file.close()
        return count

    def test_map(self):
        rospy.init_node('test_map')
        filename = rospy.get_param("~map_file", "")
        minLines = rospy.get_param("~min_lines", 1)
        print("test_map");
        t = 0
        while t < TIMEOUT:
            lines = self.countlines(filename)
            print("map has %d lines %d needed", lines, minLines)
            if lines >= minLines:
                return 
        else:
            self.fail("Not enough map entries")
        
if __name__ == '__main__':
    try:
        rostest.run('rostest', NAME, MapTest, sys.argv)
    except KeyboardInterrupt:
        pass
print("exiting")
