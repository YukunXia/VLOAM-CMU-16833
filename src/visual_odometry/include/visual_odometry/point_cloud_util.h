#include <ros/package.h>
#include <ros/ros.h>

#include <boost/lexical_cast.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <vector>

#ifndef POINT_CLOUD_UTIL_H
#define POINT_CLOUD_UTIL_H

namespace vloam
{
class PointCloudUtil
{
public:
  PointCloudUtil()
  {
    cam_T_velo = Eigen::Matrix4f::Zero();
    rect0_T_cam = Eigen::Matrix4f::Zero();
    P_rect0 = Eigen::MatrixXf::Zero(3, 4);
    print_result = false;
    downsample_grid_size = 5;
  }
  void loadTransformations(const std::string& calib_cam_to_cam_file_path,
                           const std::string& calib_velo_to_cam_file_path);
  void loadPointCloud(const std::string& bin_file_path);
  void projectPointCloud();
  void printPointCloud2dStats() const;
  void downsamplePointCloud();
  void visualizePointCloud(const std::string image_file_path, const bool select_downsampled = true);
  float queryDepth(const float x, const float y, const int searching_radius = 2) const;
  void visualizeDepthCallBack(int event, int x, int y);
  void visualizeDepth(const std::string image_file_path);
  cv::Mat visualizeDepth(const cv::Mat& gray_image);

  // private:
  const int IMG_HEIGHT = 375;
  const int IMG_WIDTH = 1242;
  Eigen::Matrix4f cam_T_velo;   // T_velo^cam or cam_T_velo // TODO: rewrite related functions using Eigen::isometry or
                                // Eigen::Transforma; https://eigen.tuxfamily.org/dox/classEigen_1_1Transform.html
  Eigen::Matrix4f rect0_T_cam;  // T_cam0^rect0 or rect0_T_cam0 // NOTE: typedef Transform<float,3,Isometry> Isometry3f
  Eigen::MatrixXf P_rect0;      // TODO: combine the 3 transformations into 1 3x4 matrix

  Eigen::MatrixXf point_cloud_3d_tilde;  // row size is dynamic, and will be decided when load the point cloud; column
                                         // size is fixed as 4
  Eigen::MatrixXf point_cloud_2d;
  Eigen::MatrixXf point_cloud_2d_dnsp;

  int downsample_grid_size;

  Eigen::MatrixXf bucket_x;
  Eigen::MatrixXf bucket_y;
  Eigen::MatrixXf bucket_depth;
  Eigen::MatrixXi bucket_count;

  cv::Mat image_with_point_cloud;
  cv::Mat image_with_depth;

  bool print_result;
};

}  // namespace vloam

#endif