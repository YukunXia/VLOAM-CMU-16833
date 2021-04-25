#pragma once

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include "lidar_odometry_mapping/common.h"
#include "lidar_odometry_mapping/tic_toc.h"
#include "lidar_odometry_mapping/lidarFactor.hpp"

namespace vloam {

    class LaserOdometry {
        public:
            LaserOdometry() : q_last_curr(para_q), t_last_curr(para_t) {}

            void init (ros::NodeHandle* nh_);
            void reset ();
            void solveLO();

            void TransformToStart(PointType const *const pi, PointType *const po);
            void TransformToEnd(PointType const *const pi, PointType *const po);

            void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2);
            void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2);
            void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2);
            void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2);
            //receive all point cloud
            void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2);

        private:
            const bool DISTORTION = false;

            int corner_correspondence, plane_correspondence;
            const double SCAN_PERIOD = 0.1;
            const double DISTANCE_SQ_THRESHOLD = 25;
            const double NEARBY_SCAN = 2.5;

            int skipFrameNum;
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

            std::vector<int> count_receive;
            sensor_msgs::PointCloud2ConstPtr cornerSharpBuf;
            sensor_msgs::PointCloud2ConstPtr cornerLessSharpBuf;
            sensor_msgs::PointCloud2ConstPtr surfFlatBuf;
            sensor_msgs::PointCloud2ConstPtr surfLessFlatBuf;
            sensor_msgs::PointCloud2ConstPtr fullPointsBuf;

            ros::NodeHandle* nh;

            ros::Subscriber subCornerPointsSharp;
            ros::Subscriber subCornerPointsLessSharp;
            ros::Subscriber subSurfPointsFlat;
            ros::Subscriber subSurfPointsLessFlat;
            ros::Subscriber subLaserCloudFullRes;

            ros::Publisher pubLaserCloudCornerLast;
            ros::Publisher pubLaserCloudSurfLast;
            ros::Publisher pubLaserCloudFullRes;
            ros::Publisher pubLaserOdometry;
            ros::Publisher pubLaserPath;

            nav_msgs::Path laserPath;

            int frameCount;
    };

}