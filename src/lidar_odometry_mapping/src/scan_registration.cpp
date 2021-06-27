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

#include <lidar_odometry_mapping/scan_registration.h>

namespace vloam
{
void ScanRegistration::init()
{
  ros::param::get("loam_verbose_level", verbose_level);

  systemInitCount = 0;
  systemInited = false;
  N_SCANS = 0;

  if (!ros::param::get("scan_line", N_SCANS))
    ROS_BREAK();

  if (!ros::param::get("minimum_range", MINIMUM_RANGE))
    ROS_BREAK();

  ROS_INFO("scan line number %d \n", N_SCANS);

  if (N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
  {
    ROS_ERROR("only support velodyne with 16, 32 or 64 scan line!");
  }

  // // NOTE: publishers and subscribers won't necessarily used, but are kept for forward compatibility
  // subLaserCloud = nh->subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100,
  // &ScanRegistration::laserCloudHandler, this);
  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);
  pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
  pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);
  pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);
  pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);
  // pubRemovePoints = nh->advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

  PUB_EACH_LINE = false;
  // // std::vector<ros::Publisher> pubEachScan;
  // // if(PUB_EACH_LINE)
  // // {
  // //     for(int i = 0; i < N_SCANS; i++)
  // //     {
  // //         ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
  // //         pubEachScan.push_back(tmp);
  // //     }
  // // }

  laserCloud = boost::make_shared<pcl::PointCloud<PointType>>();
  cornerPointsSharp = boost::make_shared<pcl::PointCloud<PointType>>();
  cornerPointsLessSharp = boost::make_shared<pcl::PointCloud<PointType>>();
  surfPointsFlat = boost::make_shared<pcl::PointCloud<PointType>>();
  surfPointsLessFlat = boost::make_shared<pcl::PointCloud<PointType>>();
}

void ScanRegistration::reset()
{
  laserCloud->clear();
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();

  laserCloudScans = std::vector<pcl::PointCloud<PointType>>(N_SCANS);  // N_SCANS = 64 for kitti
}

template <typename PointT>
void ScanRegistration::removeClosedPointCloud(const pcl::PointCloud<PointT>& cloud_in,
                                              pcl::PointCloud<PointT>& cloud_out, float thres)
{
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }

  size_t j = 0;

  for (size_t i = 0; i < cloud_in.points.size(); ++i)
  {
    if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y +
            cloud_in.points[i].z * cloud_in.points[i].z <
        thres * thres)
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    j++;
  }
  if (j != cloud_in.points.size())
  {
    cloud_out.points.resize(j);
  }

  cloud_out.height = 1;
  cloud_out.width = static_cast<uint32_t>(j);
  cloud_out.is_dense = true;
}

void ScanRegistration::input(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn_)
{
  if (!systemInited)
  {
    systemInitCount++;
    if (systemInitCount >= systemDelay)
    {
      systemInited = true;
    }
    else
      return;
  }
  if (verbose_level > 1)
  {
    ROS_INFO("scan registration starts");
    t_whole.tic();
  }

  std::vector<int> scanStartInd(N_SCANS, 0);
  std::vector<int> scanEndInd(N_SCANS, 0);

  pcl::PointCloud<pcl::PointXYZ> laserCloudIn = laserCloudIn_;
  // pcl::fromROSMsg(laserCloudMsg, laserCloudIn); // TODO: see if the input of the function should have another type
  // from vloam_main
  std::vector<int> indices;

  pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
  removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);

  if (verbose_level > 1)
  {
    ROS_INFO("end of closed point removal");
  }

  int cloudSize = laserCloudIn.points.size();
  float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
  float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

  if (endOri - startOri > 3 * M_PI)
  {
    endOri -= 2 * M_PI;
  }
  else if (endOri - startOri < M_PI)
  {
    endOri += 2 * M_PI;
  }

  if (verbose_level > 1)
  {
    ROS_INFO("end Ori %f\n", endOri);
  }

  bool halfPassed = false;
  int count = cloudSize;
  PointType point;
  for (int i = 0; i < cloudSize; i++)
  {
    point.x = laserCloudIn.points[i].x;
    point.y = laserCloudIn.points[i].y;
    point.z = laserCloudIn.points[i].z;

    float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
    int scanID = 0;

    if (N_SCANS == 16)
    {
      scanID = int((angle + 15) / 2 + 0.5);
      if (scanID > (N_SCANS - 1) || scanID < 0)
      {
        count--;
        continue;
      }
    }
    else if (N_SCANS == 32)
    {
      scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
      if (scanID > (N_SCANS - 1) || scanID < 0)
      {
        count--;
        continue;
      }
    }
    else if (N_SCANS == 64)
    {
      if (angle >= -8.83)
        scanID = int((2 - angle) * 3.0 + 0.5);
      else
        scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

      // use [0 50]  > 50 remove outlies
      if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
      {
        count--;
        continue;
      }
    }
    else
    {
      ROS_INFO("wrong scan number\n");
      ROS_BREAK();
    }
    // ROS_INFO("angle %f scanID %d \n", angle, scanID);

    float ori = -atan2(point.y, point.x);
    if (!halfPassed)
    {
      if (ori < startOri - M_PI / 2)
      {
        ori += 2 * M_PI;
      }
      else if (ori > startOri + M_PI * 3 / 2)
      {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI)
      {
        halfPassed = true;
      }
    }
    else
    {
      ori += 2 * M_PI;
      if (ori < endOri - M_PI * 3 / 2)
      {
        ori += 2 * M_PI;
      }
      else if (ori > endOri + M_PI / 2)
      {
        ori -= 2 * M_PI;
      }
    }

    float relTime = (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + scanPeriod * relTime;
    laserCloudScans[scanID].push_back(point);
  }

  cloudSize = count;

  if (verbose_level > 1)
  {
    ROS_INFO("points size %d \n", cloudSize);
  }

  for (int i = 0; i < N_SCANS; i++)
  {
    scanStartInd[i] = laserCloud->size() + 5;
    *laserCloud += laserCloudScans[i];
    scanEndInd[i] = laserCloud->size() - 6;
  }

  if (verbose_level > 1)
  {
    ROS_INFO("prepare time %f \n", t_prepare.toc());
  }

  for (int i = 5; i < cloudSize - 5; i++)
  {
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x +
                  laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x +
                  laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x +
                  laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y +
                  laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y +
                  laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y +
                  laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z +
                  laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z +
                  laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z +
                  laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;
  }

  TicToc t_pts;

  float t_q_sort = 0;
  for (int i = 0; i < N_SCANS; i++)
  {
    if (scanEndInd[i] - scanStartInd[i] < 6)
      continue;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
    for (int j = 0; j < 6; j++)
    {
      int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
      int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

      TicToc t_tmp;
      std::sort(cloudSortInd + sp, cloudSortInd + ep + 1,
                [&](const int& i, const int& j) { return cloudCurvature[i] < cloudCurvature[j]; });
      t_q_sort += t_tmp.toc();

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--)
      {
        int ind = cloudSortInd[k];

        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
        {
          largestPickedNum++;
          if (largestPickedNum <= 2)
          {
            cloudLabel[ind] = 2;
            cornerPointsSharp->push_back(laserCloud->points[ind]);
            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
          }
          else if (largestPickedNum <= 20)
          {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
          }
          else
          {
            break;
          }

          cloudNeighborPicked[ind] = 1;

          for (int l = 1; l <= 5; l++)
          {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
            {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--)
          {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
            {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++)
      {
        int ind = cloudSortInd[k];

        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
        {
          cloudLabel[ind] = -1;
          surfPointsFlat->push_back(laserCloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4)
          {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++)
          {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
            {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--)
          {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
            {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++)
      {
        if (cloudLabel[k] <= 0)
        {
          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }

    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    *surfPointsLessFlat += surfPointsLessFlatScanDS;
  }

  if (verbose_level > 1)
  {
    ROS_INFO("sort q time %f \n", t_q_sort);
    ROS_INFO("seperate points time %f \n", t_pts.toc());

    ROS_INFO("scan registration time %f ms *************\n", t_whole.toc());
  }
}

void ScanRegistration::publish()
{
  sensor_msgs::PointCloud2 laserCloudOutMsg;
  pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
  laserCloudOutMsg.header.stamp = ros::Time::now();
  laserCloudOutMsg.header.frame_id = "velo";
  pubLaserCloud.publish(laserCloudOutMsg);

  sensor_msgs::PointCloud2 cornerPointsSharpMsg;
  pcl::toROSMsg(*cornerPointsSharp, cornerPointsSharpMsg);
  cornerPointsSharpMsg.header.stamp = ros::Time::now();
  cornerPointsSharpMsg.header.frame_id = "velo";
  pubCornerPointsSharp.publish(cornerPointsSharpMsg);

  sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
  pcl::toROSMsg(*cornerPointsLessSharp, cornerPointsLessSharpMsg);
  cornerPointsLessSharpMsg.header.stamp = ros::Time::now();
  cornerPointsLessSharpMsg.header.frame_id = "velo";
  pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

  sensor_msgs::PointCloud2 surfPointsFlat2;
  pcl::toROSMsg(*surfPointsFlat, surfPointsFlat2);
  surfPointsFlat2.header.stamp = ros::Time::now();
  surfPointsFlat2.header.frame_id = "velo";
  pubSurfPointsFlat.publish(surfPointsFlat2);

  sensor_msgs::PointCloud2 surfPointsLessFlat2;
  pcl::toROSMsg(*surfPointsLessFlat, surfPointsLessFlat2);
  surfPointsLessFlat2.header.stamp = ros::Time::now();
  surfPointsLessFlat2.header.frame_id = "velo";
  pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

  // pub each scam
  if (PUB_EACH_LINE)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      sensor_msgs::PointCloud2 scanMsg;
      pcl::toROSMsg(laserCloudScans[i], scanMsg);
      scanMsg.header.stamp = ros::Time::now();
      scanMsg.header.frame_id = "map";
      pubEachScan[i].publish(scanMsg);
    }
  }

  // ROS_INFO("scan registration time %f ms *************\n", t_whole.toc());
  // if(t_whole.toc() > 100)
  //     ROS_WARN("scan registration process over 100ms");
}

void ScanRegistration::output(pcl::PointCloud<PointType>::Ptr& laserCloud_,
                              pcl::PointCloud<PointType>::Ptr& cornerPointsSharp_,
                              pcl::PointCloud<PointType>::Ptr& cornerPointsLessSharp_,
                              pcl::PointCloud<PointType>::Ptr& surfPointsFlat_,
                              pcl::PointCloud<PointType>::Ptr& surfPointsLessFlat_)
{
  laserCloud_ = this->laserCloud;
  cornerPointsSharp_ = this->cornerPointsSharp;
  cornerPointsLessSharp_ = this->cornerPointsLessSharp;
  surfPointsFlat_ = this->surfPointsFlat;
  surfPointsLessFlat_ = this->surfPointsLessFlat;
}

}  // namespace vloam