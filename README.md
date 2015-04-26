
Simultaneous localization and mapping using fiducial markers
============================================================

Creating a map
--------------

To create an empty map file with fiducial 301 at the origin:

`$ mkdir ~/.ros/slam`
`$ echo '301 0.0 0.0 0.0 180.0 -0.0 0.0 0.0 1' > ~/ros/slam/map.txt`

The format of this file is id x y z pan tilt roll numObservations

Calibrating a camera
--------------------

The
  [Stereo Camera Calibration](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration)
information is also used for calibrating single lens cameras.

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



