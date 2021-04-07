# Prerequisites

OpenCV 4.5.1
Eigen3 3.3
Ceres 2.0
PCL 1.2

# Data format

Place bag files under "src/data_loader/bags/"

Note: current dataloader only support "synced" type dataset. 

# To-Do-List

- Load transformations/calibrations from ROS, and set as ROS parameters
- Add code to point cloud util to receive the transformations from ROS
- Add code to convert ROS image to cv::Mat
- Add code to convert ROS PointCloud2 to Eigen MatrixXd
- Run VO continuously
- Publish odom from VO
- Visualize odom from VO