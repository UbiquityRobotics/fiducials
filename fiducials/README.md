# Robot Localization Using Ceiling Fiducials

This code was developed to allow a robot to localize
itself in an indoor environment where fiducial tags
are located on the ceiling.

## Compilation

Deprecated: Clone this repository and type "make".

This is now a catkin_package. Clone this repository into a directory under
the src directory in your catkin workspace and build by typing "catkin_make"
in the root of the workspace.

I only test on Linux/Ubuntu so you are on your own for other platforms.

The following are known dependencies:

* GCC 4.7 or higher (sudo apt-get install build-essential)
* InkScape (sudo apt-get install inkscape)
* doxygen (sudo apt-get install doxygen)
* ROS (groovy) ( [ROS Install](http://wiki.ros.org/groovy/Installation/Ubuntu) )

OpenCV shows up with the ROS install.  Your .bashrc must get
the ROS_ROOT environment variable set, or the Makefiles will
not work.

If you run across any additional dependencies, update this file
with your copy of the repository, and send me a pull request.

### Compiler upgrade

Danny upgraded his compiler as follows:

    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get upgrade -y
    sudo apt-get dist-upgrade -y
    sudo apt-get install -y build-essential libopencv-dev \
      inkscape doxygen

This may be more upgrading than you really want.

## Tags

The Tags program is used to generate .svg files for tags.

    Tags 41 42

will generate tag41.svg and tag42.svg.  To print:

    inkscape --without-gui --export-pdf=tag41.pdf tag41.svg
    lpr tag41.pdf

## Video_Capture

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

## Demo

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

