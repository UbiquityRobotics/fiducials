# Robot Localization Using Ceiling Fiducials

This code was developed to allow a robot to localize
itself in an indoor environment where fiducial tags
are located on the ceiling.

## Compilation

Clone this repository and type "make".  I only test on Linux
so you are on your own for other platforms.

The following are known dependencies:

* GCC (sudo apt-get install build-essential)
* OpenCV (sudo apt-get install libopencv-dev)
* InkScape (sudo apt-get install inkscape)

If you run across any additional dependencies, update this file
with your copy of the repository, and send me a pull request.

## Tags

The Tags program is used to generate .svg files for tags.

    Tags 41 42

will generate tag41.svg and tag42.svg.  To print:

    inkscape --without-gui --export-pdf=tag41.pdf tag41.svg
    lpr tag41.pdf

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

