## aruco_detect aruco_detect

This node finds aruco markers in images stream and publishes their vertices
(corner points) and estimates 3D transforms from the camera to the fiducials.
It is based on the [Aruco](http://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html)
contributed module to OpenCV. It is an alternative to fiducial_detect

Documentation is in [the ROS wiki page](http://wiki.ros.org/aruco_detect).

### Running the tests
```
colcon build --packages-select aruco_detect && colcon test --packages-select aruco_detect && colcon test-result
```

With output to the console
```
colcon build --packages-select aruco_detect && colcon test --packages-select aruco_detect --event-handlers console_direct+ && colcon test-result
```