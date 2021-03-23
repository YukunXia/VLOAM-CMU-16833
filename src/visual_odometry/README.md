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

NOTE: Currently, the running of the test code relies on that catkin_ws follows the following tree structure. And please run code at the root of catkin_ws.

- catkin_ws
    - build
    - data
        - 2011_09_26
            - 2011_09_26_drive_0001_sync
            - calib_cam_to_cam.txt
            - calib_velo_to_cam.txt
    - devel
    - src

## calibration file

Reference: https://github.com/yanii/kitti-pcl/blob/master/KITTI_README.TXT

>calib_cam_to_cam.txt: Camera-to-camera calibration
>--------------------------------------------------
>
>  - S_xx: 1x2 size of image xx before rectification
>  - K_xx: 3x3 calibration matrix of camera xx before rectification
>  - D_xx: 1x5 distortion vector of camera xx before rectification
>  - R_xx: 3x3 rotation matrix of camera xx (extrinsic)
>  - T_xx: 3x1 translation vector of camera xx (extrinsic)
>  - S_rect_xx: 1x2 size of image xx after rectification
>  - R_rect_xx: 3x3 rectifying rotation to make image planes co-planar
>  - P_rect_xx: 3x4 projection matrix after rectification
>
>Note: When using this dataset you will most likely need to access only
>P_rect_xx, as this matrix is valid for the rectified image sequences.
>
>calib_velo_to_cam.txt: Velodyne-to-camera registration
>------------------------------------------------------
>
>  - R: 3x3 rotation matrix
>  - T: 3x1 translation vector
>  - delta_f: deprecated
>  - delta_c: deprecated
>
>R|T takes a point in Velodyne coordinates and transforms it into the
>coordinate system of the left video camera. Likewise it serves as a
>representation of the Velodyne coordinate frame in camera coordinates.
>
>calib_imu_to_velo.txt: GPS/IMU-to-Velodyne registration
>-------------------------------------------------------
>
>  - R: 3x3 rotation matrix
>  - T: 3x1 translation vector
>
>R|T takes a point in GPS/IMU coordinates and transforms it into the
>coordinate system of the Velodyne scanner. Likewise it serves as a
>representation of the GPS/IMU coordinate frame in Velodyne coordinates.

From the paper "Vision meets robotics: The KITTI dataset", the `P_rect` is defined with element(1,3) and element(2,3) to be 0, but in real calibration file, these two entries are not necessarily exactly zero, especailly element(1,3). Anyway, the value would be 2 orders smaller than element(0,3).

## odometry data

From http://www.cvlibs.net/datasets/kitti/eval_odometry.php: 
- Download odometry data set (grayscale, 22 GB)
- Download odometry data set (color, 65 GB)
- Download odometry data set (velodyne laser data, 80 GB)
- Download odometry data set (calibration files, 1 MB)
- Download odometry ground truth poses (4 MB)
- Download odometry development kit (1 MB)

OpenCV load dataset: https://docs.opencv.org/3.4/dc/dfb/group__datasets__slam.html

## Processing KITTI data

### Image with Point Cloud

![Image with Point Cloud](figures/gray_image_with_depth.png)

### Image with Downsampled Point Cloud (grid size = 5)

![Image with Point Cloud](figures/gray_image_with_depth_dnsp.png)