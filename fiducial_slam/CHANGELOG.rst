^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fiducial_slam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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