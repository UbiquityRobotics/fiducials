# stag_detect

## Overview

This package consist of files for generating and launching STag marker related software.

The stag_detect node finds STag markers in images stream and estimates 3D transforms from the camera to the fiducials.

## Per-marker size configuration

The node now supports an optional `config_file` parameter for per-marker size lookup.
The file is a YAML document with one default size and a list of marker-specific overrides:

```yaml
default_marker_size: 0.18
markers:
  - id: 8
    size: 0.08
  - id: 18
    size: 0.14
```

Launch example:

```bash
ros2 launch stag_detect stag_detect.launch.py marker_size:=0.18 config_file:=cfg/marker_sizes.yaml
```

If a detected marker id exists in the file, that size is used for pose estimation. Otherwise the node falls back to `marker_size`.

## TODO

- Extend the YAML config to optionally store world-frame tag positions for future global pose estimation.

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
