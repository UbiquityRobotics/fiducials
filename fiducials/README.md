# Robot Localization Using Ceiling Fiducials

This code was developed to allow a robot to localize itself in an
indoor environment where fiducial tags are located on the ceiling.

## Installation

This code is destributed as a ROS package.  The new ROS packaging
format called catkin is used.  We are currently testing against the
"Groovy" release of ROS.  ROS currently only installs on the various
Ubuntu Linux distributions (e.g. Ubuntu, Kubuntu, etc.)

### Install ROS Groovy

The first step is to
[install ROS Groovy](http://wiki.ros.org/groovy/Installation/Ubuntu).

Make sure that you have edited your ~/.bashrc file to have:

        source /opt/ros/groovy/setup.bash

in it.  Then make sure you have run that script file:

        source ~/.bashrc

Now go to the ROS tutorials and play with
[catkin workspaces](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

### Install GCC 4.7 or Higher

The second step is to get to a version of the GCC compiler suite
that is at revision of 4.7 or higher.  The reason for this is because
the Fiducials code is written in the newer C11 (for 2011) revision of
the C programming language.  The GCC revisions 4.6 and below do not
support C11.

To figure out which version you have.  Type:

        sudo apt-get install -y build-essential

to make sure that you have a compiler.  Next, type:

        gcc --version

and you will get something that looks like:

        gcc (Ubuntu/Linaro 4.7.2-2ubuntu1) 4.7.2
        Copyright (C) 2012 Free Software Foundation, Inc.
        This is free software; see the source for copying conditions.  There is NO
        warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

In this particular example, GCC is at revision 4.7.2 which is what
you need.  If it says 4.6.x or less, you need to get a newer GCC compiler.

        sudo add-apt-repository ppa:ubuntu-toolchain-r/test
        sudo apt-get update
        sudo apt-get upgrade -y
        sudo apt-get dist-upgrade -y
        sudo apt-get install -y build-essential

### Install InkScape and Doxygen

InkScape is used to convert fiducial tags from .svg Scalable Vector
Graphics format into .pdf Portable Document Format so that they can
be printed out on a laser printer.  To install InkScape:

        sudo apt-get install -y inkscape

Doxygen is the documentation generation program that reads the various
source files an prints more readable documenation.  To install Doxygen:

        sudo apt-get install -y doxygen

### Install and Build the Fiducials Code

We assume that you called your catkin workspace "catkin_ws" in the
installation steps below.

First you fetch the fiducials catkin package:

        cd .../catkin_ws/src
	git clone https://github.com/waynegramlich/fiducials.git

Next you build it:

        cd ..
	catkin_make

Everything is now installed:

I only test on Linux/Ubuntu so you are on your own for other platforms.

### Installation Issues

If the instructions above do not work for you, please drop us
a line at [Wayne@Gramlich.Net](mailto:Wayne@Gramlich.Net) and
let us know what when wrong.

## Programs

The program should be present in your fiducials build directory:

        cd .../catkin_ws/build/fiducials

### Tags

The Tags program is used to generate .svg files for tags.  Runing Tags:

    Tags 41 42

will generate tag41.svg and tag42.svg.  To print:

    inkscape --without-gui --export-pdf=tag41.pdf tag41.svg
    lpr tag41.pdf

### Video_Capture

The Video_Capture program capture is used to display video from
a video camera and capture a sequence of images from the video
stream.  To use:

    Video_Capture camera_number [capture_base_name]

If the image does not come up, try again.  If comes up with
the image rotated horizontally.  If it keeps coming up screwy,
unplug the camera and try again.  Honest, it is unclear what
the issue is.

To use image capture, first click on the image to shift the
input focus to Video capture.  To capture an image, type the
[space] key.  To exit, type the [Esc] key.

### Demo

The Demo program is used to debug and show what is going
on under the covers.

    Demo image-05.tga

will load the image-05.tga file and do fiducial recognition
on it.  Move the cursor over the window that pops up and
click on the image.  This moves the input focus to the Demo
program.  Click on '+' to increment one step through processing
and '-' to decrement one step through processing.

The steps are:

* Color to Gray
* Gaussian blur
* Gray to Black and White
* Edge detect
* Edge simplify to polygons
* Select reasonable size quadralaterals
* Find corners
* Sample fiducial edges
* Sample fiducial bits
* Recognize fiducial id's

