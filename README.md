
# Simultaneous Localization and Mapping Using Fiducial Markers
Travis:  [![Build Status](https://travis-ci.org/UbiquityRobotics/fiducials.svg?branch=kinetic-devel)](https://travis-ci.org/UbiquityRobotics/fiducials)

Jenkins: [![Build Status](http://build.ros.org/view/Kdev/job/Kdev__fiducials__ubuntu_xenial_amd64/badge/icon)](http://build.ros.org/view/Kdev/job/Kdev__fiducials__ubuntu_xenial_amd64/)

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

Documentation is at [http://wiki.ros.org/aruco_detect](http://wiki.ros.org/fiducials).
