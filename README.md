
Simultaneous localization and mapping using fiducial markers
============================================================

Overview
--------

A document describing the system in more detail in
  [this document](https://docs.google.com/a/mrjim.com/document/d/1GsqAXgagWFZp891-5EDgfnYioPGjC1JdtXoIOecaQ-w)

Creating a map
--------------

To create an empty map file with fiducial 301 at the origin:

        mkdir -p ~/.ros/slam
        echo '301 0.0 0.0 0.0 180.0 -0.0 0.0 0.0 1' > ~/ros/slam/map.txt

The format of this file is id x y z pan tilt roll numObservations

Calibrating a camera
--------------------

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

        rosrun camera_calibration cameracalibrator.py camera:=pgr_camera_node image:=pgr_camera_node/image_raw --size 8x6 --square 0.038 --no-service_check

For the Raspberry Pi camera using gscam:

        rosrun camera_calibration cameracalibrator.py camera:=GSCamNodelet image:=camera_node/image_raw --size 8x6 --square 0.038 --no-service_check

The resulting calibration is stored in `~ros/camera_info`.

When done, `ost.txt` needs to be copied into `~/.ros/slam/ost.txt`.

Generating Fiducials
--------------------

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

Running Fiducials
-----------------

To run the localization:

        roslaunch fiducial_slam fiducials_pgr_nav_3d.launch

To run the navigation:

        roslaunch fiducial_slam fiducials_pgr_nav_3d.launch


Nodes
-----

### fiducial_detect fducial_localization

This node finds fiducial markers in an image stream and publishes there vertices (corner points).  It also has 2D SLAM built in.

#### Parameters

**tag_height** Name of the tag_height file (default `Tag_Heights.xml`).  This file is used to specify the height of the fiducials for 2D slam,
but is required to exist even if 2D slam is not used.

**map_file** Name of the file where the generated 2D SLAM-based map should be stored (default `ROS_MAP`).

**log_file** Name of the log file (default `fiducuals.log.txt`).

**data_directory** Name of the directory where tag_height and map_file reside, relative to `~/.ros`.

**odom_frame** If this is set to a non-empty string, then the result of the localization is published as a correction to odometry.
For example, the odometry publishes the tf from map to odom, and this node publishes the tf from odom to base_link, with the tf from
map to odom removed. 
 
**map_frame** The name of the map (world) frame.  Default `map`.

**pose_frame** The frame for our tf. Default `base_link`.

**publish_images** If `true`, images containing fiducials are published. Default `false`.

**publish_images** If `true`, 'interesting' images containing fiducials are published. Default `false`. This is for debug purposes.

**publish_tf** If `true`, transforms are published. Default `true`.

**publish_markers** If `true`, visualization markers are published. Default `true`.

#### Published Topics

**fiducuals** **visualization_msgs::Marker**

**vertices** **fiducial_detect::Fiducial**

**fiducials_images** **ImageTransport**

**interesting_images** **ImageTransport**

#### Subscribed Topics

**camera** **ImageTransport**
