
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

Assuming a checker board with 30mm squared, the following command can be used
to calibrate the camera:

`$ rosrun camera_calibration cameracalibrator.py camera:=pgr_camera_node image:=pgr_camera_node/image_raw --size 5x4 --square 0.030`

The resulting calibration is stored in `~ros/camera_info`.
