
# Simultaneous Localization and Mapping Using Fiducial Markers

## Overview

This package implements a system that uses ceiling mounted
fiducials (think QR Codes) to allow a robot to identify
its location and orientation.  It does this by constructing
a map of the ceiling fiducials.  Once the map has been
constructed, the robot can identify its location by locating
itself relative to a single ceiling fiducial.

A
  [more detailed document](https://docs.google.com/a/mrjim.com/document/d/1GsqAXgagWFZp891-5EDgfnYioPGjC1JdtXoIOecaQ-w)
is available that describes the system in more detail.

In order to use this system you will need to do the following:

* Mount Ceiling Fiducials.  You will need to print and install
  a number of fiduicals on your ceiling.  The fiduicals have to
  be big enough that your camera can resolve them.

* Camera Installation.  You will need to install and calibrate
  an upward pointing camera.

* Software Installation.  You will have to install the appropriate
  software.

## Installing Ceiling Fiducials

First you must install ROS (Robot Operating System) on both
your robot and some sort of laptop/desktop.

The fiducials are typically generated from the laptop/desktop.

Fiduical tags are generated using the following commands:

	cd /tmp
        rosrun fiducial_lib Tags num ...

Thus, the following command will generate `tag42.svg` and `tag43.svg`:

        rosrun fiducial_lib Tags 42 43

To generate a 100 at a time, use the following command:

        rosrun fiducial_lib Tags 17{0,1,2,3,4,5,6,7,8,9}{0,1,2,3,4,5,6,7,8,9}

This will generate `tag1700.svg` through `tag1799.svg` using
"glob curly bracket" magic.

To convert the `.svg` files to to .pdf files use the`inkscape`
program and the following commands:

        sudo apt-get install inkscape
        # Convert a single .svg to a single .pdf
        inkscape --without-gui --export-pdf=tag42.pdf tag42.svg
        # Convert a bunch at a time:
        for n in 17{0,1,2,3,4,5,6,7,8,9}{0,1,2,3,4,5,6,7,8,9} ; do \
           inkscape --without-gui --export-pdf=tag$n.pdf tag$n.svg ; \
           done
        # Merge the remaining pdf files:
        sudo apt-get install poppler-utils
        pdfunite tag17??.pdf tags17xx.pdf
	rm tag*.svg

You can print the `.pdf` using using `evince`:

	sudo apt-get install evince
        evince tags17xx.pdf

## Camera Installation:

You are going to need to install a camera and hook it into ROS.

### Raspberry Pi Camera

The
  [Raspberry Pi 2 Model B](https://www.raspberrypi.org/products/raspberry-pi-2-model-b/)
is a 900MHz quad core ARM7 single board computer with 1GB or RAM.
It has a variety of connectors including a CSI MIPI camera connector
for their
  [Rapsberry Pi Camera](https://www.raspberrypi.org/products/camera-module/)
which has some
  [nice specifications](https://www.raspberrypi.org/documentation/hardware/camera.md).

We currently recommend using the raspicam_node for this camera
  [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node)

> Currently, we install using source.  We are working on a binary
> install.

(The commands below do not work yet!!!)

When the binary package is ready, do the following command on your
robot:

        sudo apt-get install ubiquity-indigo-raspicam-node

In addition, you will need to install the fiducals package:

        sudo apt-get install ubiquity-indigo-fiducials

For testing purposes, you will also need to install an area
to testing.  First, create a ROS catkin workspace


## Calibrating a camera

Each time you select a new camera image size, it is necessary
to recalibrate the camera and generate a calibration `.yaml` file.

The
  [Monocular Camera Calibration tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
shows how to calibrate a single camera.

The
  [8x6 checkerboard](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check-108.pdf)
and the
  [7x6 checkerboard](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check_7x6_108mm.pdf)
are rather large and require specialized printers to print out at
full scale.  They can be printed on more common printer sizes
with auto scaling turned on.  Be sure to carefully measure the
square size in millimeters and convert to meters by dividing by 1000.

Assuming a checker board with 38mm squares, the following command can be used
to calibrate the camera:

        rosrun camera_calibration cameracalibrator.py camera:=pgr_camera_node image:=pgr_camera_node/image_raw --size 8x6 --square 0.038 --no-service-check

For the Raspberry Pi camera using gscam:

	# On Rasperry Pi:
	roslaunch fiducial_detect raspi_camera.launch
        # Note: This launch file refers to a calibration file that
        # probably does not exist.  The resulting error is OK.

	# On a laptop/desktop with ROS_MASTER_URI and ROS_HOSTNAME
	# env. variables set.
        rosrun camera_calibration cameracalibrator.py camera:=camera image:=camera_node/image_raw --size 8x6 --square 0.038 --no-service-check

Read the
  [Monocular Calibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
tutorial.  When the calibrator comes up, the [Calibrate], [Save],
and [Commit] buttons are dimmed out.  After the [Calibrate]
button shows up, click on it *once*.   After the [Save] and [Commit]
buttons show up, click on the [Save] button *once*.  Do not waste
your time clicking on [Commit], that feature is not supported by
`gscam`.

The resulting calibration is written into `/tmp/calibrabrationdata.tar.gz`.
Unpack the data as follows:

        cd /tmp
        mkdir camara_data
        cd camera_data
        gunzip -c ../calibrationdata.tar.gz | tar xvf -

Now you get to convert the file `ost.ini` to a `.yaml` file.

        rosrun camera_calibration_parsers ost.ini camera_WIDTHxHEIGHT.yaml
	mkdir -p ~/.ros/slam
	cp camera_WIDTHxHEIGHT.yaml ~/.ros/slam

where *WIDTH* is the image width and *HEIGHT* is the image height.
Make sure that `.../fiducials/fiducial_detect/launch/raspi.launch`
points to this file:

        <param name="camera_info_url"
               value="file:///home/ubuntu.ros/slam/raspi_WIDTHxHEIGHT.yaml"/>

After this step, when you run the camera, you should no longer
get the calibration file error.

## Running Fiducials

(The localization and navigation commands are the same.)

To run the localization:

        mkdir -p ~/.ros/fiducials	# Only do once
        roslaunch fiducial_slam fiducial_pgr_3d.launch

On a Raspberry PI with camera:

        roslaunch fiducial_slam fiducial_raspi_3d.launch

To generate a map quickly, run this:

        roslaunch fiducial_slam fiducial_raspi_3d_map.launch

To run the navigation:

        roslaunch fiducial_slam navigation_noscan.launch

## Map Creation

To create an empty map file with fiducial 301 at the origin:

        mkdir -p ~/.ros/slam
        echo '301 0.0 0.0 0.0 180.0 0.0 180.0 0.0 1' > ~/ros/slam/map.txt

The format of this file is id x y z pan tilt roll numObservations
This assumes the fiducial is on the ceiling, and the 180 degree rotations are used
so that the pose of fiducials is determined in the co-ordinate system of the floor.  Note that incresed accuracy can be gained by specifying the height of the fiducial (z), and that this can also be determined from the output of 
`fiducial_slamp.py`.

It is important that the transform from base link to the camera frame is
correctly specified.  This launch files are currently in transition, but
at the moment, `fiducial_detect/launch/raspi_pose.launch` is where this tf
is specified. 



## Nodes

### fiducial_detect fiducial_detect

This node finds fiducial markers in images stream and publishes their vertices
(corner points) and estimates 3D transforms from the camera to the fiducials.
It also has 2D SLAM built in.

#### Parameters


**tag_height** Name of the tag_height file (default `Tag_Heights.xml`).  This
file is used to specify the height of the fiducials for 2D slam.

**map_file** Name of the file where the generated 2D SLAM-based map should be
stored (default `ROS_MAP`).

**log_file** Name of the log file (default `fiducuals.log.txt`).

**data_directory** Name of the directory where tag_height and map_file reside,
relative to `~/.ros`.

**odom_frame** If this is set to a non-empty string, then the result of the
localization is published as a correction to odometry.
For example, the odometry publishes the tf from map to odom, and this node
publishes the tf from odom to base_link, with the tf from
map to odom removed. Default: not set.
 
**map_frame** The name of the map (world) frame.  Default `map`.

**pose_frame** The frame for our tf. Default `base_link`.

**publish_images** If `true`, images containing fiducials are published. Default
`false`.

**publish_interesting_images** If `true`, 'interesting' images containing fiducials are
published. Default `false`. This is for debug purposes.

**publish_tf** If `true`, transforms are published. Default `false`.

**publish_markers** If `true`, visualization markers are published. Default 
`false`.

**estimate_pose** If `true`, 3D pose estimation is performed and fiducial
transforms are published. Default `true`.

**fiducial_len** The length of one side of a fiducial in meters, used by the
pose estimation.  Default 0.146.

**undistort_points** If `false`, it is assumed that the input is an undistorted
image, and the vertices are used directly to calculate the fiducial transform.
If it is `true`, then the vertices are undistorted first. This is faster, but
less accurate.  Default `false`.

**fiducials_are_level** If `true`, it is assumed that all fiducials are level, as
would be the case on ceiling mounted fiducials. In this case only 3DOF are estimated.
Default `true`.

#### Published Topics

**fiducuals** A topic of `visualization_msgs/Marker` messages that can be viewed
in rviz for debugging, if that option is selected.

**vertices** A topic of `fiducial_detect/Fiducial*` messages with the detected
fiducial vertices.

**fiducials_images** An `ImageTransport*` of images containing fiducials, if that
option is selected.

**interesting_images** `ImageTransport*` of interesting images, if that option
is selected.

**fiducial_transforms** A topic of `fiducial_pose/FiducialTransform` messages 
with the computed fiducial pose.

#### Subscribed Topics

**camera** An `ImageTransport` of the images to be processed.

**camera_info** A topic of `sensor_msgs/CameraInfo` messages with the camera
intrinsic parameters.


### fiducial_slam fiducial_slam.py

This node performs 3D Simultaneous Localization and Mapping (SLAM) from the 
fiducial transforms. For the mapping part, pairs of transforms are combined
to determine the position of fiducials based on existing observations.
For the localization part, fiducial transforms are combined with fiducial poses
to estimate the camera pose (and hence the robot pose).

#### Parameters

**map_file** Path to the file containing the generated map (this must exist). Default `map.txt`.

**trans_file** Path to a file to store all detected fidicial transforms. Default `trans.txt`.

**obs_file** Path to a file to store all detected fidicial observations. Default `obs.txt`.

**odom_frame** If this is set to a non-empty string, then the result of the localization is published as a correction to odometry.
For example, the odometry publishes the tf from map to odom, and this node publishes the tf from odom to base_link, with the tf from
map to odom removed. Default: not set.
 
**camera_frame** The name of the camera frame.  Default `camera`.

**map_frame** The name of the map (world) frame.  Default `map`.

**pose_frame** The frame for our tf. Default `base_link`.

**publish_tf** If `true`, transforms are published. Default `true`.

**republish_tf** If `true`, transforms are republished until a new pose is calculated. Default `true`.

**mapping_mode** If `true` the map updated and saved more frequently.

**use_external_pose** If `true` then the node will attempt to use an external 
estimate of the robot pose (eg from AMCL) to estimate the pose of fiducials
if no known fiducials are observed.

**future** Amount of time (in seconds) to future-date published transforms.
Default 0.0.


#### Published Topics

**fiducuals** A topic of `visualization_msgs/Marker` messages that can be viewed
in rviz for debugging.

**fidicual_pose** a topic of `geometry_msgs/PoseWithCovarianceStamped` containing
the computed pose.

**tf** Transforms


#### Subscribed Topics

**fiducial_transforms** A topic of `fiducial_pose/FiducialTransform` messages with
fiducial pose.

**tf** Transforms

### fiducial_slam init_amcl.py

This node will reinitialize amcl by republishing the pose reported from 
fiducial_slamp.py to amcl as an initial pose.

#### Parameters

**cov_thresh** The threshold of covariance reported in *amcl_pose* for
reinitializing it.  Default 0.2.

#### Published Topics

**initial_pose** (geometry_msgs/PoseWithCovarianceStamped) The initial pose 
sent to AMCL

#### Subscribed Topics

**amcl_pose** (geometry_msgs/PoseWithCovarianceStamped) The pose from AMCL. The
covariance of this is examined to determine if AMCL needs re-initializing.

**fiducial_pose** (geometry_msgs/PoseWithCovarianceStamped) The pose from 
fiducial_slam.py


## Visualization with rviz

In order to see the fiducials during map building:

        roslaunch fiducial_detect fiducial_rviz.launch

You should start to see something that looks as follows:

> ![RViz Showing Fiducials](fiducials_rviz.png "RViz Displaying Fiducials")

* Red cubes represent fiducials that are currently in view
  of the camera.

* Green cubes represent fiducials that are in the map, but
  not currently in the view of the camera.

* Blue lines connect pairs of fiducials that have shown
  up in the camera view at the same time.  The map is constructed
  by stringing together fiducial pairs.

## File Formats:

### `map.txt` file format:

The format of `map.txt` is a series of lines of the form:

        id x y z pan tilt roll variance numObservations [neighbors ...]
=======
# Robot Localization Using Ceiling Fiducials

This code was developed to allow a robot to localize itself in an
indoor environment where fiducial tags are located on the ceiling.

## Installation

This code is destributed as a ROS package.  The new ROS packaging
format called catkin is used.  We are currently testing against the
"Groovy" release of ROS.  ROS currently only installs on the various
Ubuntu Linux distributions (e.g. Ubuntu, Kubuntu, etc.)

### Install ROS Groovy

The first step is to
[install ROS Groovy](http://wiki.ros.org/groovy/Installation/Ubuntu).

Make sure that you have edited your ~/.bashrc file to have:

        source /opt/ros/groovy/setup.bash

in it.  Then make sure you have run that script file:

        source ~/.bashrc

Now go to the ROS tutorials and play with
[catkin workspaces](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

### Install GCC 4.7 or Higher

The second step is to get to a version of the GCC compiler suite
that is at revision of 4.7 or higher.  The reason for this is because
the Fiducials code is written in the newer C11 (for 2011) revision of
the C programming language.  The GCC revisions 4.6 and below do not
support C11.

To figure out which version you have.  Type:

        sudo apt-get install -y build-essential

to make sure that you have a compiler.  Next, type:

        gcc --version

and you will get something that looks like:

        gcc (Ubuntu/Linaro 4.7.2-2ubuntu1) 4.7.2
        Copyright (C) 2012 Free Software Foundation, Inc.
        This is free software; see the source for copying conditions.  There is NO
        warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

In this particular example, GCC is at revision 4.7.2 which is what
you need.  If it says 4.6.x or less, you need to get a newer GCC compiler.

        sudo add-apt-repository ppa:ubuntu-toolchain-r/test
        sudo apt-get update
        sudo apt-get upgrade -y
        sudo apt-get dist-upgrade -y
        sudo apt-get install -y gcc-4.8
        sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 30

### Install InkScape and Doxygen

InkScape is used to convert fiducial tags from .svg Scalable Vector
Graphics format into .pdf Portable Document Format so that they can
be printed out on a laser printer.  To install InkScape:

        sudo apt-get install -y inkscape

Doxygen is the documentation generation program that reads the various
source files an prints more readable documenation.  To install Doxygen:

        sudo apt-get install -y doxygen

### Install Pt. Grey SDK

There are a bunch of Pt. Gray Flycapture-MV cameras kicking
around.  These are 648 x 480 USB monochrome cameras with a
global shutter.  The global shutter makes them much less immune
to motion blur.  For now, we are standardizing on this camera
for testing the fiducial software.

Evenutally, we will make the software build optionally with
or without the Pt. Grey device drivers.  However, until then,
it is necessary to download the Pt. Grey Software Develoment Kit.
Please follow the following steps:

* to go to the [Point Grey Web Site](www.ptgrey.com)

* go to [Support] => [Downloads] and click.  The following
  [downloads link](http://www.ptgrey.com/support/downloads/downloads_admin/Index.aspx)
  will work until the site gets reorganized.

* You can not have the software until you create an account.
  Their passwords must consist of letters and digits only.

* If you are lucky you should get to the
  [Product Support: Downloads](http://www.ptgrey.com/support/downloads/downloads_admin/Download.aspx)
  page.  As usual, the link will break when the web site is
  reorganized.

* Currently it brings you to a selection panel.  Just click the
  on [Software (24)] item.  It will expand into 24 possibilities.

* Scroll down until you get [FlyCapure 2.5 Release 4 - Linux].
  The are four options --
  1) [32-bit x86](http://www.ptgrey.com/support/downloads/downloads_admin/dlhelper.aspx?vp=flycapture2-2.5.3.4-i386-pkg.tgz&dld=180),
  2) [64-bit x86](http://www.ptgrey.com/support/downloads/downloads_admin/dlhelper.aspx?vp=flycapture2-2.5.3.4-amd64-pkg.tgz&dld=180),
  3) [ARM Hard Float](http://www.ptgrey.com/support/downloads/downloads_admin/dlhelper.aspx?vp=flycapture.2.5.3.4_armhf.tar.gz&dld=180), and
  4) [ARM Soft Float](http://www.ptgrey.com/support/downloads/downloads_admin/dlhelper.aspx?vp=flycapture.2.5.3.4_arm.tar.gz&dld=180).
  Download the correct one for your platform.

* Untar the tarball:

	cd {somwhere}
        gunzip -c {name_of_tar.gz} | tar xvf -

* Read the "readme.txt" file and install as much as you can.
  These instructions only go up to Ubuntu 10.04, which is getting
  pretty old.

* Finally run the install script:

        sudo sh install_flycapture.sh

Now it should be possible to download and build the fiducials package.

### Install and Build the Fiducials Code

We assume that you called your catkin workspace "catkin_ws" in the
installation steps below.

First you fetch the fiducials and fiducials_rviz catkin package:

    cd .../catkin_ws/src
    git clone https://github.com/waynegramlich/fiducials.git
    git clone https://github.com/waynegramlich/fiducials_rviz.git

Next you build it:

    cd ..
    catkin_make

Everything is now installed:

I only test on Linux/Ubuntu so you are on your own for other platforms.

### Installation Issues

If the instructions above do not work for you, please drop us
a line at [Wayne@Gramlich.Net](mailto:Wayne@Gramlich.Net) and
let us know what when wrong.

## Programs

The program should be present in your fiducials build directory:

        cd .../catkin_ws/build/fiducials

### Tags

The Tags program is used to generate .svg files for tags.  Runing Tags:

    Tags 41 42

will generate tag41.svg and tag42.svg.  To print:

    inkscape --without-gui --export-pdf=tag41.pdf tag41.svg
    lpr tag41.pdf

### Video_Capture

The Video_Capture program capture is used to display video from
a video camera and capture a sequence of images from the video
stream.  To use:

    Video_Capture camera_number [capture_base_name]

If the image does not come up, try again.  If comes up with
the image rotated horizontally.  If it keeps coming up screwy,
unplug the camera and try again.  Honest, it is unclear what
the issue is.

To use image capture, first click on the image to shift the
input focus to Video capture.  To capture an image, type the
[space] key.  To exit, type the [Esc] key.

### Fly_Capture

The Fly_Capture program capture is used to display video from
a Pt. Grey video camera and capture a sequence of images from
video stream.  To use:

    Fly_Capture camera_number [capture_base_name]

If the image does not come up, try again.  If comes up with
the image rotated horizontally.  If it keeps coming up screwy,
unplug the camera and try again.  Honest, it is unclear what
the issue is.

To use image capture, first click on the image to shift the
input focus to Video capture.  To capture an image, type the
[space] key.  To exit, type the [Esc] key.

### Demo

The Demo program is used to debug and show what is going
on under the covers with the Fiducials code:

    Demo dojo_3.6mm_6Oct2013/pg_3_6mm.txt dojo_3.6mm_6Oct2013/dojo_3.6mm-15.pnm

will load the dojo_3.6mm-15.pnm file and do fiducial recognition 
on it.  pg_3_6.txt is the lens correction coeeficients.  Move the
cursor over the window that pops up and click on the image.  This
moves the input focus to the Demo program.  Click on '+' to
increment one step through processing and '-' to decrement one
step through processing.

The steps are:

* Color to Gray
* Gaussian blur ['b' toggles the blur]
* Gray to Black and White
* Edge detect
* Edge simplify to polygons
* Select reasonable size quadralaterals
* Find corners to sub-pixel resolution
* Sample fiducial edges
* Sample fiducial bits
* Recognize fiducial id's (nothing visible yet)

>>>>>>> 896d7425d4d40c137a554d6916c37e9895645617
