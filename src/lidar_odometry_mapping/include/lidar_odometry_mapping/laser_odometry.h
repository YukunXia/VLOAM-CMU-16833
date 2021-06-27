// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <lidar_odometry_mapping/common.h>
#include <lidar_odometry_mapping/tic_toc.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <vloam_tf/vloam_tf.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <lidar_odometry_mapping/lidarFactor.hpp>
#include <mutex>
#include <queue>

namespace vloam
{
class LaserOdometry
{
public:
  LaserOdometry() : q_last_curr(para_q), t_last_curr(para_t), nh("laser_odometry_node")
  {
  }

  void init(std::shared_ptr<VloamTF>& vloam_tf_);

  // void reset ();

  void input(const pcl::PointCloud<PointType>::Ptr& laserCloud_,
             const pcl::PointCloud<PointType>::Ptr& cornerPointsSharp_,
             const pcl::PointCloud<PointType>::Ptr& cornerPointsLessSharp_,
             const pcl::PointCloud<PointType>::Ptr& surfPointsFlat_,
             const pcl::PointCloud<PointType>::Ptr& surfPointsLessFlat_);
  void solveLO();
  void publish();
  void output(Eigen::Quaterniond& q_w_curr_, Eigen::Vector3d& t_w_curr,
              pcl::PointCloud<PointType>::Ptr& laserCloudCornerLast_,
              pcl::PointCloud<PointType>::Ptr& laserCloudSurfLast_, pcl::PointCloud<PointType>::Ptr& laserCloudFullRes_,
              bool& skip_frame);

  void TransformToStart(PointType const* const pi, PointType* const po);
  void TransformToEnd(PointType const* const pi, PointType* const po);

private:
  const bool DISTORTION = false;

  int corner_correspondence, plane_correspondence;
  const double SCAN_PERIOD = 0.1;
  const double DISTANCE_SQ_THRESHOLD = 25;
  const double NEARBY_SCAN = 2.5;

  int mapping_skip_frame;
  bool systemInited;

  double timeCornerPointsSharp;
  double timeCornerPointsLessSharp;
  double timeSurfPointsFlat;
  double timeSurfPointsLessFlat;
  double timeLaserCloudFullRes;

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast;

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudFullRes;

  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  // Transformation from current frame to world frame
  Eigen::Quaterniond q_w_curr;
  Eigen::Vector3d t_w_curr;

  // q_curr_last(x, y, z, w), t_curr_last
  double para_q[4];
  double para_t[3];

  Eigen::Map<Eigen::Quaterniond> q_last_curr;
  Eigen::Map<Eigen::Vector3d> t_last_curr;

  std::shared_ptr<VloamTF> vloam_tf;

  ros::NodeHandle nh;
  int verbose_level;
  bool detach_VO_LO;

  ros::Publisher pubLaserCloudCornerLast;
  ros::Publisher pubLaserCloudSurfLast;
  ros::Publisher pubLaserCloudFullRes;
  ros::Publisher pubLaserOdometry;
  ros::Publisher pubLaserPath;

  nav_msgs::Path laserPath;

  int frameCount;
};

}  // namespace vloam