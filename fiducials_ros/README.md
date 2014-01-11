# fiducials_ros

A ROS library to expose some of the fiducials functionality as a ROS node for integration with the ROS navigation stack.

# Setup

Follow the setup instructions for the fiducials package; until my changes changes are merged, you'll want to check out my modified version of the fiducials repository: https://github.com/trainman419/fiducials

Check this package out into your catkin workspace.

Build your catkin workspace with `catkin_make`

# Running the demo

Source your devel space (install space isn't supported yet)

    . catkin_ws/devel/setup.bash

cd to the fiducials directory:

    cd catkin_ws/src/fiducials

Run the demo:

    rosrun fiducials_ros fiducials_localization _lens_calibration:=calibration/pg_3_6mm.txt dojo_3.6mm_6Oct2013/*.pnm
