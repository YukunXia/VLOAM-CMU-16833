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

#include <nav_msgs/Odometry.h>
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

#include <cmath>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "lidar_odometry_mapping/common.h"
#include "lidar_odometry_mapping/tic_toc.h"

using std::atan2;
using std::cos;
using std::sin;

namespace vloam
{
class ScanRegistration
{
public:
  ScanRegistration() : nh("scan_registration_node")
  {
  }

  void init();
  void reset();
  void input(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn_);
  void publish();
  void output(pcl::PointCloud<PointType>::Ptr& laserCloud_, pcl::PointCloud<PointType>::Ptr& cornerPointsSharp_,
              pcl::PointCloud<PointType>::Ptr& cornerPointsLessSharp_, pcl::PointCloud<PointType>::Ptr& surfPointsFlat_,
              pcl::PointCloud<PointType>::Ptr& surfPointsLessFlat_);

  // bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }
  template <typename PointT>
  void removeClosedPointCloud(const pcl::PointCloud<PointT>& cloud_in, pcl::PointCloud<PointT>& cloud_out, float thres);

private:
  const double scanPeriod = 0.1;

  const int systemDelay = 0;
  int systemInitCount;
  bool systemInited;
  int N_SCANS;
  float cloudCurvature[400000];
  int cloudSortInd[400000];
  int cloudNeighborPicked[400000];
  int cloudLabel[400000];

  ros::NodeHandle nh;
  int verbose_level;

  // ros::Subscriber subLaserCloud;
  ros::Publisher pubLaserCloud;
  ros::Publisher pubCornerPointsSharp;
  ros::Publisher pubCornerPointsLessSharp;
  ros::Publisher pubSurfPointsFlat;
  ros::Publisher pubSurfPointsLessFlat;
  // ros::Publisher pubRemovePoints;
  std::vector<ros::Publisher> pubEachScan;

  bool PUB_EACH_LINE;

  double MINIMUM_RANGE = 0.1;

  pcl::PointCloud<PointType>::Ptr laserCloud;
  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

  std::vector<pcl::PointCloud<PointType>> laserCloudScans;

  TicToc t_whole;
  TicToc t_prepare;
};

}  // namespace vloam