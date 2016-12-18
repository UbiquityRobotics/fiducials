## fiducial_detect fiducial_detect

This node finds fiducial markers in images stream and publishes their vertices
(corner points) and estimates 3D transforms from the camera to the fiducials.
It also has 2D SLAM built in.

### Parameters

* `estimate_pose` If `true`, 3D pose estimation is performed and fiducial
transforms are published. Default `true`.

* `fiducial_len` The length of one side of a fiducial in meters, used by the
pose estimation.  Default 0.146.

* `undistort_points` If `false`, it is assumed that the input is an undistorted
image, and the vertices are used directly to calculate the fiducial transform.
If it is `true`, then the vertices are undistorted first. This is faster, but
less accurate.  Default `false`.


### Published Topics


* `/fiducial_vertices`  A topic of type `fiducial_pose/Fiducial` messages with the detected
fiducial vertices.


* `/fiducial_transforms` A topic of type `fiducial_pose/FiducialTransform` messages 
with the computed fiducial pose.

### Subscribed Topics

* `camera` An `ImageTransport` of the images to be processed.

* `camera_info` A topic of `sensor_msgs/CameraInfo` messages with the camera
intrinsic parameters.