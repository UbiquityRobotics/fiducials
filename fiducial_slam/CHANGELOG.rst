^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fiducial_slam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.0 (2019-05-09)
-------------------
* Compatibilty with Melodic, remove opencv3 dependancy
  opencv3 is no longer a package in melodic, so we use cv_bridge
  to get the correct opencv transitively
* New fusion with planar-based error estimation
  * Estimate variance of fiducial obsevations based on how well they place the robot on the floor upright
  * use Kalman + David method of fusing estimates
  * split out Transform with Variance, do some cleanup
  * Simple unit tests for core fusion code
  * remove hokey outlier rejection
* Add support for overriding covariance diagonal
* Add fiducial transform array stamp (`#156 <https://github.com/UbiquityRobotics/fiducials/issues/156>`_)
* Add publish_tf rosparam functionality and updated launch file accordingly (`#154 <https://github.com/UbiquityRobotics/fiducials/issues/154>`_)
  * Added publish_tf rosparam to disable publishing pose tf
  * Change default publish_tf to True and fix 'if' formatting
* Contributors: Jack Kilian, Jim Vaughan, Rohan Agrawal

0.10.0 (2018-10-13)
-------------------
* use sum in quadrature by default
* Disable multifiducial estimate by default
* Added a tool for fitting a plane
* Add map_read_only param, publish camera_pose
* Contributors: Jim Vaughan, Rohan Agrawal

0.9.0 (2018-09-12)
------------------
* Merge pull request `#112 <https://github.com/UbiquityRobotics/fiducials/issues/112>`_ from UbiquityRobotics/davecrawley-patch-1
  Added in a systematic error parameter
* fix missed rename alexy to sum in quadrature
* Suggested changes
* Updating the launch file to match the changes to map.cpp
* Added in a systematic error parameter
  Added in a systematic error parameter. This parameter is intended to take in to account systematic (non-random) errors in the measurement of fiducial position. Non-random errors can be very pernicious, and can generate highly unexpected and difficult to resolve issues. For example if **all** the fiducials are slightly the wrong size this will not show up as a random variance. Similar systematic errors include, camera mis-calibration, sampling defects, digitization errors, lens flaring and so forth. Measurement accuracy in-homogeneity can also be considered a systematic error and this will also help with that issue although it probably should be treated explicitly rather than through this "hack". Ultimately the correct approach is to rigorously compute all these systematic errors and add them in to the model - we are simply saying that we think the systematic errors are less than the specified value.
* Contributors: David Crawley, Rohan Agrawal

0.8.4 (2018-08-26)
------------------
* Don't publish pose if camera position is not known
* Update README.md
* Contributors: Jim Vaughan, Rohan Agrawal

0.8.3 (2018-02-26)
------------------
* Keep publishing map -> odom tf even if not fiducials are visible (`#103 <https://github.com/UbiquityRobotics/fiducials/issues/103>`_)
* Contributors: Jim Vaughan

0.8.2 (2018-02-14)
------------------
* Add service call to clear the map
* Contributors: Jim Vaughan

0.8.1 (2018-01-21)
------------------
  Fixed fiducial_pose topic's ``frame_id`` from camera to map.
* Contributors: Rohan Agrawal, nav-go

0.8.0 (2018-01-14)
------------------

0.7.5 (2017-12-06)
------------------

0.7.4 (2017-11-09)
------------------
* Fix typos
* Log full 6DOF pose for ALL and MUL
* Pass previous rvec and tvec to solvePnP()
* Added (multi) pose estimation to fiducial_slam (disabled by default)
* Contributors: Jim Vaughan

0.7.3 (2017-07-16)
------------------
* Install launch files and fiducials.rviz
* add test of auto init
* Print out 6DOF camera pose
* Fix multiplication order bug in autoInit()
* Renamed some variables to be more clear
* Add publish_6dof_pose param to disable squashing of estimated robot pose
* Contributors: Jim Vaughan

0.7.2 (2017-05-24)
------------------
* Using std::isnan() to stop Debian Jessie build errors
* Contributors: Jim Vaughan

0.7.1 (2017-05-22)
------------------

0.7.0 (2017-05-21)
------------------
* Fix dependencies
* Make sure that the variance sent to rviz doesn't truncate to 0
* Rewrite the code in C++, kill the python
* Fiducial transforms are always camera->fiducial
* Moved all service and message definitions to fiducial_msgs
* Contributors: Jim Vaughan, Rohan Agrawal

0.6.1 (2017-02-06)
------------------

0.6.0 (2017-02-04)
------------------
* Split Python into separate files
* Ddded median filter option
* Better exception/handling
* Contributors: Jim Vaughan, Rohan Agrawal

0.5.1 (2016-12-28)
------------------
* Map pub srv (`#36 <https://github.com/UbiquityRobotics/fiducials/issues/36>`_)
  * Added publishing of map (`#28 <https://github.com/UbiquityRobotics/fiducials/issues/28>`_) and reset service call (`#35 <https://github.com/UbiquityRobotics/fiducials/issues/35>`_)
  * Updated documentation
* Contributors: Jim Vaughan
