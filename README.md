
# Simultaneous Localization and Mapping Using Fiducial Markers
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)

## Overview

This package implements a system that uses ceiling mounted
fiducial markers (think QR Codes) to allow a robot to identify
its location and orientation.  It does this by constructing
a map of the ceiling fiducials.  The position of one fiducial
needs to be specified, then a map of the fiducials is built 
up from observing pairs of markers in the same image. 
Once the map has been constructed, the robot can identify
its location by locating itself relative to one or more 
ceiling fiducials.

Documentation is at [http://wiki.ros.org/fiducials](http://wiki.ros.org/fiducials).

## Recording A Bag File

Sometimes for trobleshooting purposes it is useful to record a bag 
file to capture the exact data on the topics going into and out of 
fiducials.

To do this, while the system is running, run `rosbag record -a`.
You can upload this bag file to a file sharing service like Google
Drive and link to it in your issue, this will help us diagnose 
the problem. 
