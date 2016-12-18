## aruco_detect aruco_detect

This node finds aruco markers in images stream and publishes their vertices
(corner points) and estimates 3D transforms from the camera to the fiducials.
It is based on the [Aruco](http://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html)
contributed module to OpenCV. It is an alternative to fiducial_detect

### Parameters

* `fiducial_len` The length of one side of a fiducial in meters, used by the
pose estimation.  Default 0.146.


### Published Topics


* `/fiducial_vertices` A topic of type `fiducial_pose/Fiducial` messages with the detected
fiducial vertices.


* `/fiducial_transforms` A topic of type `fiducial_pose/FiducialTransform` messages 
with the computed fiducial pose.

#### Subscribed Topics

* `camera` An `ImageTransport` of the images to be processed.

* `camera_info` A topic of type `sensor_msgs/CameraInfo` messages with the camera
intrinsic parameters.