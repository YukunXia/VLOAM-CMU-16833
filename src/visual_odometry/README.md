# Prerequisites

OpenCV 4.5.1

# Troubleshooting

## 1. Catkin_Make fail with cv_bridge not found 

Solution: Modify `_include_dirs` in cv_bridgeConfig.cmake (at line 96)

The default `_include_dirs` is set by `set(_include_dirs "include;/usr/include/opencv4")`, while my opencv is installed under `/usr/local/include/opencv4`.

Reference: https://github.com/cipherdev/src/issues/13

# Kitti

## raw data usage

Reference: http://www.cvlibs.net/datasets/kitti/raw_data.php

There're 4 image folders under each timestamp dataset: "image_00", "image_01", "image_02", "image_03". They correspond to grey left camera, grey right camera, RGB left camera, RGB right camera.

## odometry data

From http://www.cvlibs.net/datasets/kitti/eval_odometry.php: 
- Download odometry data set (grayscale, 22 GB)
- Download odometry data set (color, 65 GB)
- Download odometry data set (velodyne laser data, 80 GB)
- Download odometry data set (calibration files, 1 MB)
- Download odometry ground truth poses (4 MB)
- Download odometry development kit (1 MB)

OpenCV load dataset: https://docs.opencv.org/3.4/dc/dfb/group__datasets__slam.html