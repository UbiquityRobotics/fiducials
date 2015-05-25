#Simultaneous localization and mapping using fiducial markers

## Overview

A document describing the system in more detail in
  [this document](https://docs.google.com/a/mrjim.com/document/d/1GsqAXgagWFZp891-5EDgfnYioPGjC1JdtXoIOecaQ-w)

## Creating a map

To create an empty map file with fiducial 301 at the origin:

        mkdir -p ~/.ros/slam
        echo '301 0.0 0.0 0.0 180.0 -0.0 0.0 0.0 1' > ~/ros/slam/map.txt


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

## Generating Fiducials

Fiduical tags are generated using:

        rosrun fiducial_lib Tags num ...

Thus, the following command will generate `tag42.svg` and `tagr43.svg`:

        rosrun fiducial_lib Tags 42 43

To generate a 100 at a time, try:

        rosrun fiducial_lib Tags 17{0,1,2,3,4,5,6,7,8,9}{0,1,2,3,4,5,6,7,8,9}

This will generate `tag1700.svg` through `tag1799.svg` using
"glob curly bracket" magic.

To convert the `.svg` files to to .pdf files use `inkscape`:

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

## Running Fiducials

(The localization and navigation commands are the same.)

To run the localization:

        mkdir -p ~/.ros/fiducials	# Only do once
        roslaunch fiducial_slam fiducial_pgr_nav_3d.launch

To run the navigation:

        roslaunch fiducial_slam navigation_noscan.launch
        rosrun map_server map_server fiducial_detect/map.yaml


## Nodes

### fiducial_detect fducial_detect

This node finds fiducial markers in an image stream and publishes their vertices (corner points),
and estimates their 3D pose and publishes that as a transform.  It also has 2D SLAM built in.

#### Parameters

**tag_height** Name of the tag_height file (default `Tag_Heights.xml`).  This file is used to specify the height of the fiducials for 2D slam,
but is required to exist even if 2D slam is not used.

**map_file** Name of the file where the generated 2D SLAM-based map should be stored (default `ROS_MAP`).

**log_file** Name of the log file (default `fiducuals.log.txt`).

**data_directory** Name of the directory where tag_height and map_file reside, relative to `~/.ros`.

**odom_frame** If this is set to a non-empty string, then the result of the localization is published as a correction to odometry.
For example, the odometry publishes the tf from map to odom, and this node publishes the tf from odom to base_link, with the tf from
map to odom removed. Default: not set.
 
**map_frame** The name of the map (world) frame.  Default `map`.

**pose_frame** The frame for our tf. Default `base_link`.

**publish_images** If `true`, images containing fiducials are published. Default `false`.

**publish_images** If `true`, 'interesting' images containing fiducials are published. Default `false`. This is for debug purposes.

**publish_tf** If `true`, transforms are published. Default `true`.

**publish_markers** If `true`, visualization markers are published. Default `true`.

#### Published Topics

**fiducuals** A topic of `visualization_msgs/Marker` messages that can be viewed in RViz for debugging, if that option is selected.

**vertices** A topic of `fiducial_detect/Fiducial*` messages with the detected fiducial vertices.

**fiducials_images** An `ImageTransport*` of images containing fiducials, if that option is selected.

**interesting_images** `ImageTransport*` of interesting images, if that option is selected.

**fiducial_len** The length of a fiducial, in meters. Default `0.146`.

**undisort_points** If `true`, then the detected points are undistorted. Default `false`. This option should only be set if the 
input image is not undistorted.

**fiducial_transforms** A topic of `fiducial_pose/FiducialTransform` messages with the computed fiducial pose.

#### Subscribed Topics

**camera** An `ImageTransport` of the images to be processed.

**camera_info** A topic of `sensor_msgs/CameraInfo` messages with the camera intrinsic parameters.


### fiducial_slam fiducial_slam.py

This node performs 3D Simultaneous Localization and Mapping (SLAM) from the fiducial transforms.
For the mapping part, pairs of transforms are combined to determine the position of fiducials based on existing observations.
For the localization part, fiducial transforms are combined with fiducial poses to estimate the camera pose (and hence the robot pose).

#### Parameters

**map_file** Path to the file containing the generated map (this must exist). Default `map.txt`.

**trans_file** Path to a file to store all detected fidicial transforms. Default `trans.txt`.

**obs_file** Path to a file to store all detected fidicial observations. Default `obs.txt`.

**odom_frame** If this is set to a non-empty string, then the result of the localization is published as a correction to odometry.
For example, the odometry publishes the tf from map to odom, and this node publishes the tf from odom to base_link, with the tf from
map to odom removed. Default: not set.
 
**camera_frame** The name of the camera frame.  Default `camera`. If the
transform from camera_frame to pose_frame cannot be looked up, then it is
assumed that the camera is at pose_frame.

**map_frame** The name of the map (world) frame.  Default `map`.

**pose_frame** The frame for our tf. Default `base_link`.

**publish_tf** If `true`, transforms are published. Default `true`.

**mapping_mode** If `true` the map updated and saved more frequently.

#### Published Topics

**fiducuals** A topic of `visualization_msgs/Marker` messages that can be viewed in RViz for debugging.

**fidicual_pose** a topic of `geometry_msgs/PoseWithCovarianceStamped` containing the computed pose.

#### Subscribed Topics

**fiducial_transforms** A topic of `fiducial_pose/FiducialTransform` messages with fiducial pose.

## RViz

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
