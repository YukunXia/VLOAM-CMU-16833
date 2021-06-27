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

#include <lidar_odometry_mapping/lidar_odometry_mapping.h>

namespace vloam
{
void LidarOdometryMapping::init(std::shared_ptr<VloamTF>& vloam_tf_)
{
  vloam_tf = vloam_tf_;

  if (!ros::param::get("loam_verbose_level", verbose_level))
    ROS_BREAK();

  scan_registration.init();
  laser_odometry.init(vloam_tf);
  laser_mapping.init(vloam_tf);

  // subLaserCloud = nh->subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100,
  // &LidarOdometryMapping::laserCloudHandler, this);

  laserCloud = boost::make_shared<pcl::PointCloud<PointType>>();
  cornerPointsSharp = boost::make_shared<pcl::PointCloud<PointType>>();
  cornerPointsLessSharp = boost::make_shared<pcl::PointCloud<PointType>>();
  surfPointsFlat = boost::make_shared<pcl::PointCloud<PointType>>();
  surfPointsLessFlat = boost::make_shared<pcl::PointCloud<PointType>>();

  laserCloudCornerLast = boost::make_shared<pcl::PointCloud<PointType>>();
  laserCloudSurfLast = boost::make_shared<pcl::PointCloud<PointType>>();
  laserCloudFullRes = boost::make_shared<pcl::PointCloud<PointType>>();
}

void LidarOdometryMapping::reset()
{
  // prepare: laserCloud, cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat
  scan_registration.reset();
  // laser_odometry.reset();
  laser_mapping.reset();
}

void LidarOdometryMapping::scanRegistrationIO(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn)
{
  loam_timer.tic();
  frame_time = 0.0;

  scan_registration.input(laserCloudIn);

  // scan_registration.publish(); // no need to publish the point cloud feature

  scan_registration.output(laserCloud,             // 10Hz
                           cornerPointsSharp,      // 10Hz
                           cornerPointsLessSharp,  // 10Hz
                           surfPointsFlat,         // 10Hz
                           surfPointsLessFlat      // 10Hz
  );

  if (verbose_level > 0)
  {
    ROS_INFO("Scan Registration takes %f ms \n", loam_timer.toc());
  }
  frame_time += loam_timer.toc();
}

void LidarOdometryMapping::laserOdometryIO()
{
  loam_timer.tic();

  laser_odometry.input(laserCloud,             // 10Hz
                       cornerPointsSharp,      // 10Hz
                       cornerPointsLessSharp,  // 10Hz
                       surfPointsFlat,         // 10Hz
                       surfPointsLessFlat      // 10Hz
  );

  laser_odometry.solveLO();  // 10Hz

  laser_odometry.publish();

  laser_odometry.output(q_wodom_curr,          // 10Hz
                        t_wodom_curr,          // 10Hz
                        laserCloudCornerLast,  // 2Hz // no change if skip_frame
                        laserCloudSurfLast,    // 2Hz // no change if skip_frame
                        laserCloudFullRes,     // 2Hz // no change if skip_frameee
                        skip_frame);

  if (verbose_level > 0)
  {
    ROS_INFO("Laser Odometry takes %f ms \n", loam_timer.toc());
  }
  frame_time += loam_timer.toc();
}

void LidarOdometryMapping::laserMappingIO()
{
  loam_timer.tic();

  laser_mapping.input(laserCloudCornerLast,  // 2Hz
                      laserCloudSurfLast,    // 2Hz
                      laserCloudFullRes,     // 2Hz
                      q_wodom_curr,          // 10Hz
                      t_wodom_curr,          // 10Hz
                      skip_frame);

  if (!skip_frame)
    laser_mapping.solveMapping();  // 2Hz

  laser_mapping.publish();

  // laser_odometry.output(

  // );

  if (verbose_level > 0)
  {
    ROS_INFO("Laser Mapping takes %f ms \n", loam_timer.toc());
  }
  frame_time += loam_timer.toc();
  if (frame_time > 100)
  {
    ROS_WARN("LOAM takes %f ms (>100 ms)", frame_time);
  }
}

// void LidarOdometryMapping::solveLOAM() {
//     laser_odometry.solveLO();
//     laser_mapping.solveMapping();

//     // TODO: check the running time constraint: 0.1s
// }

}  // namespace vloam