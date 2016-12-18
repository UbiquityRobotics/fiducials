## fiducial_slam fiducial_slam_node.py

This node performs 3D Simultaneous Localization and Mapping (SLAM) from the 
fiducial transforms. For the mapping part, pairs of transforms are combined
to determine the position of fiducials based on existing observations.
For the localization part, fiducial transforms are combined with fiducial poses
to estimate the camera pose (and hence the robot pose).

### Parameters

* `map_file` Path to the file containing the generated map (this must exist). Default `map.txt`.

* `trans_file` Path to a file to store all detected fiducial transforms. Default `trans.txt`.

* `obs_file` Path to a file to store all detected fiducial observations. Default `obs.txt`.

* `odom_frame` If this is set to a non-empty string, then the result of the localization is
published as a correction to odometry.  For example, the odometry publishes the tf from map
to odom, and this node publishes the tf from odom to base_link, with the tf from
map to odom removed. Default: not set.
 
* `map_frame` The name of the map (world) frame.  Default `map`.

* `pose_frame` The frame for our tf. Default `base_link`.

* `publish_tf` If `true`, transforms are published. Default `true`.

* `republish_tf` If `true`, transforms are republished until a new pose is calculated. Default `true`.

* `mapping_mode` If `true` the map updated and saved more frequently.

* `use_external_pose` If `true` then the node will attempt to use an external 
estimate of the robot pose (e.g. from AMCL) to estimate the pose of fiducials
if no known fiducials are observed.

* `future` Amount of time (in seconds) to future-date published transforms.
Default 0.0.

* `fiducials_are_level` If `true`, it is assumed that all fiducials are level, as would be the case on ceiling mounted fiducials. In this case only 3DOF are estimated.
Default `true`.

### Published Topics

* `/fiducials` A topic of `visualization_msgs/Marker` messages that can be viewed
in rviz for debugging.

* `/fiducial_pose` a topic of `geometry_msgs/PoseWithCovarianceStamped` containing
the computed pose.

*  `/fiducial_map`  a topic of `fiducial_slam/FiducialMapEntryArray` containing
the current state of the map.

* `tf` Transforms


### Subscribed Topics

* `/fiducial_transforms` A topic of `fiducial_pose/FiducialTransform` messages with
fiducial pose.

* `/tf` Transforms

### Services

*  /initialize_fiducial_map  Clears the map and reinitializes it with the contents of the specified `fiducial_slam/FiducialMapEntryArray`.
