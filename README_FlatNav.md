
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

Documentation is at [http://wiki.ros.org/fiducials](http://wiki.ros.org/fiducials).

## Using the Flat Navigation Options

The original navigation mode which still exists is a very general mode.
It is the most flexible and general and allows for growing a map dynamically
as a robot navigates.   Because it is so general it can be used in 
more ways but at the same time is much more complex and prone to certain 
errors that can come about in recognition and processing of fiducials.

Starting in mid 2019 a decision/experiment was made to have some options that
were felt to lead to better navigation in a more constrained case of fiducials.

Flat Navigation is a term that forces the assumption that the floor and
the ceiling are parallel.   When this is done certain errors in the 
recognition of the fiducials do not impact the navigation scheme.

A second mode that can be forced is when in a mode where the map has
already been obtained we can enable read_only_map mode.  
The read only mode will not dynamically alter the map of ceiling fiducials
and this allows for a stable and more repeatable mode for a robot that
is always going to be navigating with a full map created previously.
This also allows for a fleet of robots to all be using the very same map.
The read_only_map mode is forced when just navigating after map creation
but of course cannot be used for map creation as a map MUST be written.

The Nav2D branches in this repository and in magni_robot repository
are both used to implement the ability to enable flat navigation techniques.
Launch files at an earlier time tried to call out specific options but
as of early 2020 all options are intended to be passed to the navigation
launch files.  A user of course can create their own launch files for 
specific mode by calling the standard launch files with options of choice.

Below are the Flat Navigation Options and basically what they change.
Keep in mind that X and Y are rotations on the floor or ceiling
and rotations about the Z axis indicate the way a fiducial is rotated
while remaining in a parallel plane to the plane of the floor.

    navigate_flat   Enables read only navigation using an existing
                    map of ceiling only fiducials.
                    The nav stack assumes no X or Y rotation of
                    the fiducials relative to the plane of the floor
                    so the name 'flat' means not at an angle to floor.
                    Set this option to true to enable (default is false)
     
    fiducials_flat  When making maps the ceiling fiducials X and Y
                    rotations are zeroed so fiducials are assumed to
                    be in a plane parallel to the floor, thus 'flat'.
                    Set this option to true to enable (default is false)

This mode of navigation is not yet released but has been coming along 
nicely.  Significant testing has not been done since the launch files have
been made to support entry of the flat modes.   Most of the prior testing
was done with other depricated flat launch files.

## Running Flat Navigation Mapping And Nav

Below is the command to run mapping using flat navigation. 
Leave off fiducials_flat specification for general solution navigation 

    roslaunch magni_demos simple_navigation.launch fiducials_flat:=True


Below is the command to run navigation after the map creation using flat navigation
leave off navigate_flat specification for general solution navigation

    roslaunch magni_demos simple_navigation.launch navigate_flat:=True 


## Recording A Bag File

Sometimes for trobleshooting purposes it is useful to record a bag 
file to capture the exact data on the topics going into and out of 
fiducials.

To do this, while the system is running, run `rosbag record -a`.
You can upload this bag file to a file sharing service like Google
Drive and link to it in your issue, this will help us diagnose 
the problem. 
