
# A node to detect aruco markers and send out their camera - marker transform

## Building with indigo

Building this node under indigo is tricky becaue indigo is compiled against
opencv 2.

This is discussed here
http://answers.ros.org/question/213925/opencv30-ros-indigo/?answer=213928#post-id-213928

In order to build it under indigo

cd catkin_ws/src
git clone https://github.com/ros-perception/vision_opencv.git
git clone https://github.com/ros-perception/image_common.git
git clone https://github.com/ros-perception/image_transport_plugins


and then build it.
