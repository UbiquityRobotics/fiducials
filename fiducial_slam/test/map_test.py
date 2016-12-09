#!/usr/bin/python

from __future__ import print_function

import sys
import threading
import time
import unittest
import os

import rospy
import rostest

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

    def countlines(self, filename):
        try:
            file = open(filename, "r")
            self.lines = file.readlines()
            file.close()
            count = len(self.lines)
            print("Map has %d lines" % count)
            return count
        except:
            return 0

    def checkposition(self, fid, x, y, z):
        for line in self.lines:
            elems = line.split()
            if int(elems[0]) != int(fid):
                continue
            if abs(float(elems[1]) - float(x)) < EPSILON and \
               abs(float(elems[2]) - float(y)) < EPSILON and \
               abs(float(elems[3]) - float(z)) < EPSILON:
                print("Fiducial %s found in correct position" % fid)
                return True
            else:
                self.fail("Fiducial %s position incorrect: %s %s %s" % (fid, elems[1], elems[2], elems[3]))
                return False
        print("Fiducial %s not found" % fid)
        return False

    def test_map(self):
        rospy.init_node('test_map')
        filename = rospy.get_param("~map_file", "")
        minLines = rospy.get_param("~min_lines", 1)
        expect = rospy.get_param("~expect", "")
        print("test_map");
        try:
            os.remove(filename)
        except:
            pass
        t = 0
        while True:
            lines = self.countlines(filename)
            if lines >= minLines:
                if expect == "":
                    return 
                (fid, x, y, z) = expect.split()
                if self.checkposition(fid, x, y, z):
                    return
	    time.sleep(0.5)
        
if __name__ == '__main__':
    try:
        rostest.run('rostest', NAME, MapTest, sys.argv)
    except KeyboardInterrupt:
        pass
print("exiting")
