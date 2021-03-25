#include <visual_odometry/point_cloud_util.h>

using namespace vloam;

int main (int argc, char** argv) {
    const std::string file_path_prefix = "data/2011_09_26/";
    const std::string calib_cam_to_cam_file_path = file_path_prefix + "calib_cam_to_cam.txt";
    const std::string calib_velo_to_cam_file_path = file_path_prefix + "calib_velo_to_cam.txt";
    const std::string bin_file_path = file_path_prefix + "2011_09_26_drive_0001_sync/velodyne_points/data/0000000000.bin";
    const std::string image_file_path = file_path_prefix + "2011_09_26_drive_0001_sync/image_00/data/0000000000.png";

    PointCloudUtil point_cloud_util;

    point_cloud_util.print_result = true;
    point_cloud_util.downsample_grid_size = 5;

    point_cloud_util.loadTransformations(calib_cam_to_cam_file_path, calib_velo_to_cam_file_path);
    point_cloud_util.loadPointCloud(bin_file_path);
    point_cloud_util.projectPointCloud();
    point_cloud_util.downsamplePointCloud();
    point_cloud_util.visualizePointCloud(image_file_path);
    point_cloud_util.visualizeDepth(image_file_path);

    return 0;
}