
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


## Generating fiducials

The fiducials are typically generated from the laptop/desktop.

Fiducial tags can generated using the create_markers.py utility.
It takes as arguments the start and end fiducial numbers and an
output file name.  For example, following commands, generates
fiducials 100 to 200:

	$ cd /tmp
    $ rosrun fiducial_lib create_markers.py 100 200 tags.pdf

You can print the PDF files using using `evince`:

	$ sudo apt-get install evince
    $ evince tags.pdf

## Map Creation

To create an empty map file with fiducial 301 (the id of 301 is arbitrary) at the origin:

        $ rosrun fiducial_slam init_map.py 301

This assumes the fiducial is on the ceiling, and sets up the rotation of the fiducial
so that the pose of fiducials is determined in the co-ordinate system of the floor.  
Increased accuracy can be gained by specifying the height of the fiducial (z), 
and that this can also be determined from the output of `fiducial_slam.py`.  

It is important that the transform from base link to the camera frame is
correctly specified.  In the `magni_robot` package, this is specified in the URDF.

## Running Fiducials

1. First create an empty map as above.

2. In order to see the fiducials during map building, run rviz:

        roslaunch fiducial_slam fiducial_rviz.launch

Note that the robot node should be running (so that the transform from odom to base_link is published)
and at at least one of the fiducials that is in the the map should have been observed before rviz 
can display anything in the map frame.

        roslaunch fiducial_detect fiducial_detect.launch
        roslaunch fiducial_slam fiducial_slam.launch
		
At this point the robot should be capable of executing move_base and other navigation functions.		

## Visualization with rviz

While building the map, you should start to see something that looks as follows:

> ![rviz Showing Fiducials](fiducials_rviz.png "rviz Displaying Fiducials")

* Red cubes represent fiducials that are currently in view
  of the camera.

* Green cubes represent fiducials that are in the map, but
  not currently in the view of the camera.

* Blue lines connect pairs of fiducials that have shown
  up in the camera view at the same time.  The map is constructed
  by combining fiducial pairs.
  
	

