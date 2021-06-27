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

#include <lidar_odometry_mapping/laser_mapping.h>
#include <lidar_odometry_mapping/laser_odometry.h>
#include <lidar_odometry_mapping/scan_registration.h>
#include <vloam_tf/vloam_tf.h>

namespace vloam
{
class LidarOdometryMapping
{
public:
  LidarOdometryMapping() : nh("lidar_odometry_mapping_node")
  {
  }

  void init(std::shared_ptr<VloamTF>& vloam_tf_);

  void reset();

  void scanRegistrationIO(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn);
  void laserOdometryIO();
  // void laserOdometryIO(const tf2::Transform& cam0_curr_T_cam0_last);
  void laserMappingIO();

private:
  std::shared_ptr<VloamTF> vloam_tf;

  ros::NodeHandle nh;
  int verbose_level;

  TicToc loam_timer;
  double frame_time;

  ScanRegistration scan_registration;
  pcl::PointCloud<PointType>::Ptr laserCloud;
  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

  LaserOdometry laser_odometry;
  Eigen::Quaterniond q_wodom_curr, q_w_curr;
  Eigen::Vector3d t_wodom_curr, t_w_curr;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
  bool skip_frame;

  LaserMapping laser_mapping;
};

}  // namespace vloam