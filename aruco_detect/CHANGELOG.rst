^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aruco_detect
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add std=-c++-11 to build.
* Fixes for OpenCV 3.3.1
* handle invalid cameraInfo message in aruco_detect
* Added (multi) pose estimation to fiducial_slam (disabled by default)
* Contributors: Jim Vaughan, Rohan Agrawal

0.7.3 (2017-07-16)
------------------

0.7.2 (2017-05-24)
------------------

0.7.1 (2017-05-22)
------------------

0.7.0 (2017-05-21)
------------------
* Fix dependencies
* Added image and object error calculation. Renamed K and dist
* Moved all service and message definitions to fiducial_msgs
* Update copyright on aruco detect C++
* Contributors: Jim Vaughan, Rohan Agrawal

0.6.1 (2017-02-06)
------------------
* Fix dynamic_reconfigure build deps
* Contributors: Jim Vaughan

0.6.0 (2017-02-04)
------------------
* moved documentation to ROS wiki
* added utilities to generate PDF files of fiducials
* Expose aruco detection parameters
* Publish one set of fiducial_vertices per image
* Parameterized the dictionary used
* Contributors: Jim Vaughan, Rohan Agrawal

0.5.1 (2016-12-28)
------------------
* Install aruco_detect launch dir
* Use raw transport for aruco test
* Contributors: Rohan Agrawal
