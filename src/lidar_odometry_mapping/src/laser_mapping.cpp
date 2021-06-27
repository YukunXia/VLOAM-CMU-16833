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

#include <lidar_odometry_mapping/laser_mapping.h>

namespace vloam
{
void LaserMapping::init(std::shared_ptr<VloamTF> &vloam_tf_)
{
  vloam_tf = vloam_tf_;

  if (!ros::param::get("loam_verbose_level", verbose_level))
    ROS_BREAK();

  frameCount = 0;

  // input: from odom
  laserCloudCornerLast = boost::make_shared<pcl::PointCloud<PointType>>();
  laserCloudSurfLast = boost::make_shared<pcl::PointCloud<PointType>>();

  // ouput: all visualble cube points
  laserCloudSurround = boost::make_shared<pcl::PointCloud<PointType>>();

  // surround points in map to build tree
  laserCloudCornerFromMap = boost::make_shared<pcl::PointCloud<PointType>>();
  laserCloudSurfFromMap = boost::make_shared<pcl::PointCloud<PointType>>();

  // input & output: points in one frame. local --> global
  laserCloudFullRes = boost::make_shared<pcl::PointCloud<PointType>>();

  // points in every cube
  for (int j = 0; j < laserCloudNum; ++j)
  {
    laserCloudCornerArray[j] = boost::make_shared<pcl::PointCloud<PointType>>();
    laserCloudSurfArray[j] = boost::make_shared<pcl::PointCloud<PointType>>();
  }

  // kd-tree
  kdtreeCornerFromMap = boost::make_shared<pcl::KdTreeFLANN<PointType>>();
  kdtreeSurfFromMap = boost::make_shared<pcl::KdTreeFLANN<PointType>>();

  parameters[0] = 0.0;
  parameters[1] = 0.0;
  parameters[2] = 0.0;
  parameters[3] = 1.0;
  parameters[4] = 0.0;
  parameters[5] = 0.0;
  parameters[6] = 0.0;

  new (&q_w_curr) Eigen::Map<Eigen::Quaterniond>(parameters);
  new (&t_w_curr) Eigen::Map<Eigen::Vector3d>(parameters + 4);

  // wmap_T_odom * odom_T_curr = wmap_T_curr;
  // transformation between odom's world and map's world frame
  q_wmap_wodom = Eigen::Quaterniond(1, 0, 0, 0);
  t_wmap_wodom = Eigen::Vector3d(0, 0, 0);

  q_wodom_curr = Eigen::Quaterniond(1, 0, 0, 0);
  t_wodom_curr = Eigen::Vector3d(0, 0, 0);

  lineRes = 0;
  planeRes = 0;
  if (!ros::param::get("mapping_line_resolution", lineRes))
    ROS_BREAK();
  if (!ros::param::get("mapping_plane_resolution", planeRes))
    ROS_BREAK();
  ROS_INFO("line resolution %f plane resolution %f \n", lineRes, planeRes);
  downSizeFilterCorner.setLeafSize(lineRes, lineRes, lineRes);
  downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

  pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);
  pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);
  pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);
  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
  // pubOdomAftMappedHighFrec = nh->advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);
  pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

  laserAfterMappedPath.poses.clear();

  // for (int i = 0; i < laserCloudNum; i++)
  // {
  // 	laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
  // 	laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
  // } // TOOD: check when this is needed

  laserCloudValidNum = 0;
  laserCloudSurroundNum = 0;

  if (!ros::param::get("mapping_skip_frame", mapping_skip_frame))
    ROS_BREAK();
  if (!ros::param::get("map_pub_number", map_pub_number))
    ROS_BREAK();
}

void LaserMapping::reset()
{
  laserCloudValidNum = 0;
  laserCloudSurroundNum = 0;
}

// // set initial guess
// void LaserMapping::transformAssociateToMap()
// {
//     q_w_curr = q_wmap_wodom * q_wodom_curr;
//     t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
// }

void LaserMapping::transformUpdate()
{
  q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
  t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}

void LaserMapping::pointAssociateToMap(PointType const *const pi, PointType *const po)
{
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
  po->x = point_w.x();
  po->y = point_w.y();
  po->z = point_w.z();
  po->intensity = pi->intensity;
  // po->intensity = 1.0;
}

void LaserMapping::pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
  Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
  po->x = point_curr.x();
  po->y = point_curr.y();
  po->z = point_curr.z();
  po->intensity = pi->intensity;
}

void LaserMapping::input(const pcl::PointCloud<PointType>::Ptr &laserCloudCornerLast_,
                         const pcl::PointCloud<PointType>::Ptr &laserCloudSurfLast_,
                         const pcl::PointCloud<PointType>::Ptr &laserCloudFullRes_,
                         const Eigen::Quaterniond &q_wodom_curr_, const Eigen::Vector3d &t_wodom_curr_,
                         const bool &skip_frame_)
{
  skip_frame = skip_frame_;

  if (!skip_frame)
  {
    laserCloudCornerLast = boost::make_shared<pcl::PointCloud<PointType>>(*laserCloudCornerLast_);
    laserCloudSurfLast = boost::make_shared<pcl::PointCloud<PointType>>(*laserCloudSurfLast_);
    laserCloudFullRes = boost::make_shared<pcl::PointCloud<PointType>>(*laserCloudFullRes_);
  }

  q_wodom_curr = q_wodom_curr_;
  t_wodom_curr = t_wodom_curr_;

  // transformAssociateToMap
  if (skip_frame)
  {
    q_w_curr_highfreq = q_wmap_wodom * q_wodom_curr;
    t_w_curr_highfreq = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
  }
  else
  {
    q_w_curr = q_wmap_wodom * q_wodom_curr;
    t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
  }
}

void LaserMapping::solveMapping()
{
  // this->reset();

  t_whole.tic();

  // transformAssociateToMap();

  TicToc t_shift;
  int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
  int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
  int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

  if (t_w_curr.x() + 25.0 < 0)
    centerCubeI--;
  if (t_w_curr.y() + 25.0 < 0)
    centerCubeJ--;
  if (t_w_curr.z() + 25.0 < 0)
    centerCubeK--;

  while (centerCubeI < 3)
  {
    for (int j = 0; j < laserCloudHeight; j++)
    {
      for (int k = 0; k < laserCloudDepth; k++)
      {
        int i = laserCloudWidth - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        for (; i >= 1; i--)
        {
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        }
        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeI++;
    laserCloudCenWidth++;
  }

  while (centerCubeI >= laserCloudWidth - 3)
  {
    for (int j = 0; j < laserCloudHeight; j++)
    {
      for (int k = 0; k < laserCloudDepth; k++)
      {
        int i = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        for (; i < laserCloudWidth - 1; i++)
        {
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        }
        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeI--;
    laserCloudCenWidth--;
  }

  while (centerCubeJ < 3)
  {
    for (int i = 0; i < laserCloudWidth; i++)
    {
      for (int k = 0; k < laserCloudDepth; k++)
      {
        int j = laserCloudHeight - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        for (; j >= 1; j--)
        {
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
        }
        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeJ++;
    laserCloudCenHeight++;
  }

  while (centerCubeJ >= laserCloudHeight - 3)
  {
    for (int i = 0; i < laserCloudWidth; i++)
    {
      for (int k = 0; k < laserCloudDepth; k++)
      {
        int j = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        for (; j < laserCloudHeight - 1; j++)
        {
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
        }
        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeJ--;
    laserCloudCenHeight--;
  }

  while (centerCubeK < 3)
  {
    for (int i = 0; i < laserCloudWidth; i++)
    {
      for (int j = 0; j < laserCloudHeight; j++)
      {
        int k = laserCloudDepth - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        for (; k >= 1; k--)
        {
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
        }
        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeK++;
    laserCloudCenDepth++;
  }

  while (centerCubeK >= laserCloudDepth - 3)
  {
    for (int i = 0; i < laserCloudWidth; i++)
    {
      for (int j = 0; j < laserCloudHeight; j++)
      {
        int k = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        for (; k < laserCloudDepth - 1; k++)
        {
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
        }
        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeK--;
    laserCloudCenDepth--;
  }

  for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
  {
    for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
    {
      for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
      {
        if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight && k >= 0 && k < laserCloudDepth)
        {
          laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
          laserCloudValidNum++;
          laserCloudSurroundInd[laserCloudSurroundNum] =
              i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
          laserCloudSurroundNum++;
        }
      }
    }
  }

  laserCloudCornerFromMap->clear();
  laserCloudSurfFromMap->clear();
  for (int i = 0; i < laserCloudValidNum; i++)
  {
    *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
    *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
  }
  int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
  int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

  pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
  downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
  downSizeFilterCorner.filter(*laserCloudCornerStack);
  int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

  pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
  downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
  downSizeFilterSurf.filter(*laserCloudSurfStack);
  int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

  if (verbose_level > 1)
  {
    ROS_INFO("map prepare time %f ms\n", t_shift.toc());
    ROS_INFO("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
  }

  if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
  {
    TicToc t_opt;
    TicToc t_tree;
    kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
    kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

    if (verbose_level > 1)
      ROS_INFO("build tree time %f ms \n", t_tree.toc());

    for (int iterCount = 0; iterCount < 2; iterCount++)
    {
      // ceres::LossFunction *loss_function = NULL;
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options problem_options;

      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(parameters, 4, q_parameterization);
      problem.AddParameterBlock(parameters + 4, 3);

      TicToc t_data;
      int corner_num = 0;

      for (int i = 0; i < laserCloudCornerStackNum; i++)
      {
        pointOri = laserCloudCornerStack->points[i];
        // double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
        pointAssociateToMap(&pointOri, &pointSel);
        kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        if (pointSearchSqDis[4] < 1.0)
        {
          std::vector<Eigen::Vector3d> nearCorners;
          Eigen::Vector3d center(0, 0, 0);
          for (int j = 0; j < 5; j++)
          {
            Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                laserCloudCornerFromMap->points[pointSearchInd[j]].z);
            center = center + tmp;
            nearCorners.push_back(tmp);
          }
          center = center / 5.0;

          Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
          for (int j = 0; j < 5; j++)
          {
            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
          }

          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

          // if is indeed line feature
          // note Eigen library sort eigenvalues in increasing order
          Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
          Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
          {
            Eigen::Vector3d point_on_line = center;
            Eigen::Vector3d point_a, point_b;
            point_a = 0.1 * unit_direction + point_on_line;
            point_b = -0.1 * unit_direction + point_on_line;

            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
            corner_num++;
          }
        }
        /*
        else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
        {
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                    laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                    laserCloudCornerFromMap->points[pointSearchInd[j]].z);
                center = center + tmp;
            }
            center = center / 5.0;
            Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
            ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
        }
        */
      }

      int surf_num = 0;
      for (int i = 0; i < laserCloudSurfStackNum; i++)
      {
        pointOri = laserCloudSurfStack->points[i];
        // double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
        pointAssociateToMap(&pointOri, &pointSel);
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 1.0)
        {
          for (int j = 0; j < 5; j++)
          {
            matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
            matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
            matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
            // ROS_INFO(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
          }
          // find the norm of plane
          Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
          double negative_OA_dot_norm = 1 / norm.norm();
          norm.normalize();

          // Here n(pa, pb, pc) is unit norm of plane
          bool planeValid = true;
          for (int j = 0; j < 5; j++)
          {
            // if OX * n > 0.2, then plane is not fit well
            if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                     norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                     norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
            {
              planeValid = false;
              break;
            }
          }
          Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
          if (planeValid)
          {
            ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
            surf_num++;
          }
        }
        /*
        else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
        {
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
                                    laserCloudSurfFromMap->points[pointSearchInd[j]].y,
                                    laserCloudSurfFromMap->points[pointSearchInd[j]].z);
                center = center + tmp;
            }
            center = center / 5.0;
            Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
            ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
        }
        */
      }

      if (verbose_level > 1)
      {
        // ROS_INFO("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
        // ROS_INFO("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

        ROS_INFO("mapping data assosiation time %f ms \n", t_data.toc());
      }

      TicToc t_solver;
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 4;
      options.minimizer_progress_to_stdout = false;
      options.check_gradients = false;
      options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      if (verbose_level > 1)
        ROS_INFO("mapping solver time %f ms \n", t_solver.toc());

      // ROS_INFO("time %f \n", timeLaserOdometry);
      // ROS_INFO("corner factor num %d surf factor num %d\n", corner_num, surf_num);
      // ROS_INFO("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1],
      // parameters[2], 	   parameters[4], parameters[5], parameters[6]);
    }

    if (verbose_level > 1)
      ROS_INFO("mapping optimization time %f \n", t_opt.toc());
  }
  else
  {
    if (verbose_level > 1)
      ROS_WARN("time Map corner and surf num are not enough");
  }
  transformUpdate();

  TicToc t_add;
  for (int i = 0; i < laserCloudCornerStackNum; i++)
  {
    pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

    if (pointSel.x + 25.0 < 0)
      cubeI--;
    if (pointSel.y + 25.0 < 0)
      cubeJ--;
    if (pointSel.z + 25.0 < 0)
      cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 &&
        cubeK < laserCloudDepth)
    {
      int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
      laserCloudCornerArray[cubeInd]->push_back(pointSel);
    }
  }

  for (int i = 0; i < laserCloudSurfStackNum; i++)
  {
    pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

    if (pointSel.x + 25.0 < 0)
      cubeI--;
    if (pointSel.y + 25.0 < 0)
      cubeJ--;
    if (pointSel.z + 25.0 < 0)
      cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 &&
        cubeK < laserCloudDepth)
    {
      int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
      laserCloudSurfArray[cubeInd]->push_back(pointSel);
    }
  }

  if (verbose_level > 1)
    ROS_INFO("add points time %f ms\n", t_add.toc());

  TicToc t_filter;
  for (int i = 0; i < laserCloudValidNum; i++)
  {
    int ind = laserCloudValidInd[i];

    pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
    downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
    downSizeFilterCorner.filter(*tmpCorner);
    laserCloudCornerArray[ind] = tmpCorner;

    pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
    downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
    downSizeFilterSurf.filter(*tmpSurf);
    laserCloudSurfArray[ind] = tmpSurf;
  }

  if (verbose_level > 1)
    ROS_INFO("filter time %f ms \n", t_filter.toc());

  frameCount++;
}

void LaserMapping::publish()
{  // TODO: take in global skip frame
  TicToc t_pub;

  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.header.frame_id = "map";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = ros::Time::now();  // TODO: globally config time stamp
  if (!skip_frame)
  {
    odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
    odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
    odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
    odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
    odomAftMapped.pose.pose.position.x = t_w_curr.x();
    odomAftMapped.pose.pose.position.y = t_w_curr.y();
    odomAftMapped.pose.pose.position.z = t_w_curr.z();

    vloam_tf->world_MOT_base_last.setOrigin(tf2::Vector3(t_w_curr.x(), t_w_curr.y(), t_w_curr.z()));
    vloam_tf->world_MOT_base_last.setRotation(tf2::Quaternion(q_w_curr.x(), q_w_curr.y(), q_w_curr.z(), q_w_curr.w()));

    // ROS_INFO(
    //     "lidar_mapping: q = %.4f, %.4f, %.4f, %.4f; t = %.4f, %.4f, %.4f",
    //     q_w_curr.x(),
    //     q_w_curr.y(),
    //     q_w_curr.z(),
    //     q_w_curr.w(),
    //     t_w_curr.x(),
    //     t_w_curr.y(),
    //     t_w_curr.z()
    // );
  }
  else
  {
    odomAftMapped.pose.pose.orientation.x = q_w_curr_highfreq.x();
    odomAftMapped.pose.pose.orientation.y = q_w_curr_highfreq.y();
    odomAftMapped.pose.pose.orientation.z = q_w_curr_highfreq.z();
    odomAftMapped.pose.pose.orientation.w = q_w_curr_highfreq.w();
    odomAftMapped.pose.pose.position.x = t_w_curr_highfreq.x();
    odomAftMapped.pose.pose.position.y = t_w_curr_highfreq.y();
    odomAftMapped.pose.pose.position.z = t_w_curr_highfreq.z();

    vloam_tf->world_MOT_base_last.setOrigin(
        tf2::Vector3(t_w_curr_highfreq.x(), t_w_curr_highfreq.y(), t_w_curr_highfreq.z()));
    vloam_tf->world_MOT_base_last.setRotation(
        tf2::Quaternion(q_w_curr_highfreq.x(), q_w_curr_highfreq.y(), q_w_curr_highfreq.z(), q_w_curr_highfreq.w()));
  }
  pubOdomAftMapped.publish(odomAftMapped);

  geometry_msgs::PoseStamped laserAfterMappedPose;
  laserAfterMappedPose.header = odomAftMapped.header;
  laserAfterMappedPose.pose = odomAftMapped.pose.pose;
  laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
  laserAfterMappedPath.header.frame_id = "map";
  laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
  pubLaserAfterMappedPath.publish(laserAfterMappedPath);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(t_w_curr(0), t_w_curr(1), t_w_curr(2)));
  q.setW(q_w_curr.w());
  q.setX(q_w_curr.x());
  q.setY(q_w_curr.y());
  q.setZ(q_w_curr.z());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "map", "aft_mapped"));

  if ((frameCount * mapping_skip_frame) % map_pub_number == 0)  // 0.5 Hz?
  {
    pcl::PointCloud<PointType> laserCloudMap;
    for (int i = 0; i < 4851; i++)
    {
      laserCloudMap += *laserCloudCornerArray[i];
      laserCloudMap += *laserCloudSurfArray[i];
    }
    sensor_msgs::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(laserCloudMap, laserCloudMsg);
    laserCloudMsg.header.stamp = ros::Time::now();  // TODO: globally config time stamp
    laserCloudMsg.header.frame_id = "velo_origin";
    pubLaserCloudMap.publish(laserCloudMsg);
    if (verbose_level > 1)
      ROS_INFO("publishing the map, with %ld points", laserCloudMap.size());
  }

  int laserCloudFullResNum = laserCloudFullRes->points.size();
  for (int i = 0; i < laserCloudFullResNum; i++)
  {
    pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
  }

  sensor_msgs::PointCloud2 laserCloudFullRes3;  // Q: what's the difference between laserCloudFullRes 1 2 and 3?
  pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = ros::Time::now();  // TODO: globally config time stamp
  laserCloudFullRes3.header.frame_id = "map";
  pubLaserCloudFullRes.publish(laserCloudFullRes3);

  if (verbose_level > 1)
  {
    ROS_INFO("mapping pub time %f ms \n", t_pub.toc());

    ROS_INFO("whole mapping time %f ms +++++\n",
             t_whole.toc());  // TODO check if t_whole tictoc obj can be shared in class
  }
}

}  // namespace vloam