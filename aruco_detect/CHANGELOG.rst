^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aruco_detect
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.0 (2019-05-09)
-------------------
* Compatibilty with Melodic
   * remove opencv3 dependancy, opencv3 is no longer a package in melodic, so we use cv_bridge
   * Fix build on OpenCV 3.2 
* Import empy with full path to avoid pip conflict
* Use subprocess for cairosvg to avoid lack of python2 support
* call genMarker with correct arguments with no joblib
* Check ignoreID's in TF publishing
* Added topic that enables/disables Aruco detections
* Added rosdeps for cairo and joblib
* Add params for determining weighting of observations; prevent compiler warnings
* add cli argument for paper size to create markers
* Contributors: Jack Kilian, Jim Vaughan, Rohan Agrawal, Tim Übelhör, jack

0.10.0 (2018-10-13)
-------------------
* Dramatically speed up create_markers using cairosvg and joblib
* Overhaul aruco marker generation (calculate svg values)
* Ddd new 2 tag test image
* Add params to ignore some fiducial Ids and override sizes
* Contributors: Jim Vaughan, Rohan Agrawal

0.9.0 (2018-09-12)
------------------

0.8.4 (2018-08-26)
------------------
* Use publish_images param as intended (`#106 <https://github.com/UbiquityRobotics/fiducials/issues/106>`_)
* Update README.md
* Contributors: Jim Vaughan

0.8.3 (2018-02-26)
------------------
* Merge pull request `#100 <https://github.com/UbiquityRobotics/fiducials/issues/100>`_ from alex-gee/kinetic-devel
* Add dictionary parameter to launch file
* Contributors: Alexander Gutenkunst, Rohan Agrawal

0.8.2 (2018-02-14)
------------------

0.8.1 (2018-01-21)
------------------

0.8.0 (2018-01-14)
------------------

0.7.5 (2017-12-06)
------------------
* Move create_markers.py to share directory
* Make corner detection on by default.
* Contributors: Jim Vaughan

0.7.4 (2017-11-09)
------------------
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
