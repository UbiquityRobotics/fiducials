# stag_detect

## Overview

This package consist of files for generating and launching STag marker related software.

The stag_detect node finds STag markers in images stream and estimates 3D transforms from the camera to the fiducials.

Based on:
- https://github.com/usrl-uofsc/stag_ros
- https://github.com/bbenligiray/stag

With added support for vision_msgs which are required for full integration.

#### Library HD

You can generate STag markers from any library you prefer. 
Make sure you set the corresponding `libraryHD` parameter in launch file.
We performed most of the test with library HD11, which suited our needs.

## Packs

Generally each pack should correspond to a standalone route, but you can generate yourself packs according to your preference.
Take note that each marker has a unique id, while a number is unique only to the pack.
