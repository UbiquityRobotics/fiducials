^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fiducial_slam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
