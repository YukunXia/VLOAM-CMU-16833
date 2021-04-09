# Prerequisites

OpenCV 4.5.1
Eigen3 3.3
Ceres 2.0
PCL 1.2

# Data format

Place bag files under "src/vloam_main/bags/"

Note: current dataloader only support "synced" type dataset. 

# To-Do-List

- Add code to point cloud util to receive the transformations from ROS
- Add code to point cloud util to convert ROS PointCloud2 to Eigen MatrixXd
- Refactor and optimize VO (Maybe combine image_util and point_cloud_util into one class, and depth are directly assigned to the feature)