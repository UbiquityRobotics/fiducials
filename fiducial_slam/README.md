## fiducial_slam fiducial_slam_node.py

This node performs 3D Simultaneous Localization and Mapping (SLAM) from the 
fiducial transforms. For the mapping part, pairs of transforms are combined
to determine the position of fiducials based on existing observations.
For the localization part, fiducial transforms are combined with fiducial poses
to estimate the camera pose (and hence the robot pose).

Documentation is [on the ROS wiki](http://wiki.ros.org/fiducial_slam).
