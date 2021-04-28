#pragma once

#include <cmath>
#include <vector>
#include <string>
#include "lidar_odometry_mapping/common.h"
#include "lidar_odometry_mapping/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using std::atan2;
using std::cos;
using std::sin;

namespace vloam {

    class ScanRegistration {
        public:
            ScanRegistration() :
                nh("scan_registration_node")
            {}
            
            void init ();
            void reset();
            void input(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn_);
            void publish();
            void output(
                pcl::PointCloud<PointType>::Ptr& laserCloud_, 
                pcl::PointCloud<PointType>::Ptr& cornerPointsSharp_,
                pcl::PointCloud<PointType>::Ptr& cornerPointsLessSharp_,
                pcl::PointCloud<PointType>::Ptr& surfPointsFlat_,
                pcl::PointCloud<PointType>::Ptr& surfPointsLessFlat_
            );

            // bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }
            template <typename PointT>
            void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                pcl::PointCloud<PointT> &cloud_out, float thres);

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

}