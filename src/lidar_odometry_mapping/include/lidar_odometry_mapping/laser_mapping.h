#pragma once

#include <math.h>
#include <vector>
#include <lidar_odometry_mapping/common.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
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
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>

// #include "lidarFactor.hpp"
#include "lidar_odometry_mapping/lidarFactor.hpp"
#include "lidar_odometry_mapping/common.h"
#include "lidar_odometry_mapping/tic_toc.h"

namespace vloam {

    class LaserMapping {
        public:
                LaserMapping () : 
                laserCloudCenWidth(10), 
                laserCloudCenHeight(10), 
                laserCloudCenDepth(5), 
                q_w_curr(parameters),
                t_w_curr(parameters + 4),
                nh("laser_mapping_node")
                {}

            void init ();

            void reset();
            void input(
                const pcl::PointCloud<PointType>::Ptr& laserCloudCornerLast_,
                const pcl::PointCloud<PointType>::Ptr& laserCloudSurfLast_,
                const pcl::PointCloud<PointType>::Ptr& laserCloudFullRes_,
                const Eigen::Quaterniond& q_wodom_curr_,
                const Eigen::Vector3d& t_wodom_curr_,
                const bool& skip_frame_
            );
            void publish();
            void solveMapping();
            void output();

            // set initial guess
            void transformAssociateToMap();
            void transformUpdate();
            void pointAssociateToMap(PointType const *const pi, PointType *const po);
            void pointAssociateTobeMapped(PointType const *const pi, PointType *const po);

        private:
            int frameCount = 0;

            int laserCloudCenWidth;
            int laserCloudCenHeight;
            int laserCloudCenDepth;
            static const int laserCloudWidth = 21;
            static const int laserCloudHeight = 21;
            static const int laserCloudDepth = 11;


            static const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851


            int laserCloudValidInd[125];
            int laserCloudSurroundInd[125];

            // input: from odom
            pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
            pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;

            // ouput: all visualble cube points
            pcl::PointCloud<PointType>::Ptr laserCloudSurround;

            // surround points in map to build tree
            pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
            pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;

            //input & output: points in one frame. local --> global
            pcl::PointCloud<PointType>::Ptr laserCloudFullRes;

            // points in every cube
            pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
            pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];

            //kd-tree
            pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
            pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

            double parameters[7];
            Eigen::Map<Eigen::Quaterniond> q_w_curr;
            Eigen::Map<Eigen::Vector3d> t_w_curr;
            Eigen::Quaterniond q_w_curr_highfreq;
            Eigen::Vector3d t_w_curr_highfreq;

            // wmap_T_odom * odom_T_curr = wmap_T_curr;
            // transformation between odom's world and map's world frame
            Eigen::Quaterniond q_wmap_wodom;
            Eigen::Vector3d t_wmap_wodom;
            Eigen::Quaterniond q_wodom_curr;
            Eigen::Vector3d t_wodom_curr;

            sensor_msgs::PointCloud2ConstPtr cornerLastBuf;
            sensor_msgs::PointCloud2ConstPtr surfLastBuf;
            sensor_msgs::PointCloud2ConstPtr fullResBuf;
            nav_msgs::Odometry::ConstPtr odometryBuf;

            pcl::VoxelGrid<PointType> downSizeFilterCorner;
            pcl::VoxelGrid<PointType> downSizeFilterSurf;

            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            PointType pointOri, pointSel;

            ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath;

            nav_msgs::Path laserAfterMappedPath;

            ros::NodeHandle nh;
            int verbose_level;

            float lineRes;
            float planeRes;

            int laserCloudValidNum;
            int laserCloudSurroundNum;

            TicToc t_whole;

            bool skip_frame;
            int skip_frame_number;
            int map_pub_number;

    };

}