#!/usr/bin/python

"""
Copyright (c) 2015, Ubiquity Robotics
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of ubiquity_motor nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from tf.transformations import euler_from_quaternion, quaternion_slerp, \
                               translation_matrix, quaternion_matrix, \
                               translation_from_matrix, quaternion_from_matrix, \
                               quaternion_from_euler


from fiducial_msgs.srv import InitializeMap
from fiducial_msgs.msg import FiducialMapEntry, FiducialMapEntryArray

from fiducial_slam.fiducial import Fiducial
from fiducial_slam import mkdirnotex, rad2deg, deg2rad


import rospy
import tf2_ros

from math import pi, sqrt
import sys
import os
import traceback
import math
import numpy
import time
import threading
import copy


class Map:
    def __init__(self, mapFileName, initialMapFileName=None):
       self.fiducials = {}
       self.mapFileName = mapFileName
       if initialMapFileName:
           mkdirnotex(initialMapFileName)
       mkdirnotex(mapFileName)
       loaded = False
       if initialMapFileName:     
           loaded = self.load(initialMapFileName)
       else:
           loaded = self.load(mapFileName)
       if not loaded:
           rospy.logerr("could not load map %s", mapFileName)
       #self.mapPub = rospy.Publisher("/fiducial_map", FiducialMapEntryArray, queue_size=100) 
       #rospy.Service('initialize_fiducial_map', InitializeMap, self.initializeMap)


    """
    InitiailzeMap service
    """
    def initialize(self, msg):
        print "InitializeMap service call"
        self.fiducials = {}
        for fid in msg.fiducials:
            f = Fiducial(fid.fiducial_id)
            f.position = numpy.array(fid.x, fid.y, fid.z)
            f.orientation = quaternion_from_euler(fid.rx, fid.ry, fid.rz)
            self.fiducials[fid] = f

    """ 
    Publish map
    """
    def publish(self, topic):
        fmea = FiducialMapEntryArray()
        fids = self.fiducials.keys()
        fids.sort()
        for fid in fids:
            f = self.fiducials[fid]
            fme = FiducialMapEntry()
            fme.fiducial_id = fid
            fme.x = f.position[0]
            fme.y = f.position[1]
            fme.z = f.position[2]
            (r, p, y) = euler_from_quaternion(f.orientation)
            fme.rx = r
            fme.ry = p
            fme.rz = y
            fmea.fiducials.append(fme)
        topic.publish(fmea)

    """
    Save current map to file
    """
    def save(self, filename=None):
        if filename is None:
            filename = self.mapFileName
        print "** save map **"
        file = open(filename, "w")
        fids = self.fiducials.keys()
        fids.sort()
        for fid in fids:
            f = self.fiducials[fid]
            pos = f.position
            (r, p, y) = euler_from_quaternion(f.orientation)
            file.write("%d %r %r %r %r %r %r %r %d %s\n" % \
              (f.id, pos[0], pos[1], pos[2],
               rad2deg(r), rad2deg(p), rad2deg(y),
               f.variance, f.observations,
               ' '.join(map(str, f.links))))
        file.close()

    """
    Load map from file
    """
    def load(self, filename):
        try:
            file = open(filename, "r")
            for line in file.readlines():
                words = line.split()
                fid = int(words[0])
                f = Fiducial(fid)
                f.position = numpy.array((float(words[1]), float(words[2]), float(words[3])))
                f.orientation = quaternion_from_euler(deg2rad(words[4]), deg2rad(words[5]), deg2rad(words[6]))
                f.variance = float(words[7])
                f.observations = int(words[8])
                f.links = map(int, words[9:])
                self.fiducials[fid] = f
            file.close()
            return True
        except:
            traceback.print_exc()
            return False

    """
    Print out fiducial vertices
    """
    def showVertices(self):
        fids = self.fiducials.keys()
        fids.sort()
        for fid in fids:
            f = self.fiducials[fid]
            pos = f.position
            off = numpy.dot(translation_matrix(numpy.array((-1.0, -1.0, 0.0))), f.pose44())
            off = numpy.dot(translation_matrix(numpy.array((1.0, -1.0, 0.0))), f.pose44())
            off = numpy.dot(translation_matrix(numpy.array((1.0, 1.0, 0.0))), f.pose44())
            off = numpy.dot(translation_matrix(numpy.array((-1.0, 1.0, 0.0))), f.pose44())


    def __getitem__(self, id):
        return self.fiducials[id]

    def __setitem__(self, id, object):
        self.fiducials[id] = object

    def has_key(self, id):
        return self.fiducials.has_key(id)

    def keys(self):
        return self.fiducials.keys()
