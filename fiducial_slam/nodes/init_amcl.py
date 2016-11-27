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

"""
Initialize amcl from fiducial pose
"""


import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

from math import pi, sqrt
import sys
import os
import traceback
import math
import numpy
import time
import copy

class InitAMCL:
    def __init__(self):
       rospy.init_node('init_amcl')
       self.lastAmclPose = None
       self.isInitialized = False
       self.cov_thresh = rospy.get_param("~cov_thresh", 0.2)
       self.initPub = rospy.Publisher("/initialpose", 
                                        PoseWithCovarianceStamped, 
                                        queue_size=1)
       rospy.Subscriber("/amcl_pose",
                        PoseWithCovarianceStamped,
                        self.amclPose)
       rospy.Subscriber("/fiducial_pose",
                        PoseWithCovarianceStamped,
                        self.fiducialPose)

    def amclPose(self, m):
        print "amcl_pose", m.pose.covariance[0]
        self.lastAmclPose = m

    def fiducialPose(self, m):
        #print "fiducial_pose", m
        if self.lastAmclPose.pose.covariance[0] > self.cov_thresh \
        or not self.isInitialized:
            self.initPub.publish(m)
            self.isInitialized = True

if __name__ == "__main__":
    node = InitAMCL()
    rospy.spin()
