#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/loss_function.h>
#include <pcl/point_types.h>

#include <visual_odometry/point_cloud_util.h>
#include <visual_odometry/image_util.h>
#include <visual_odometry/ceres_cost_function.h>

#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

namespace vloam {
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> VO_policy;

    class VisualOdometry {
        public:
            VisualOdometry ();

            void init();

            void setUpVO();

            void processImage(const cv::Mat& img00);

            void setUpPointCloud(const Eigen::Isometry3f& imu_eigen_T_cam0, const Eigen::Isometry3f& imu_eigen_T_velo, const sensor_msgs::CameraInfoConstPtr& camera_info_msg);

            void processPointCloud(const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg, const pcl::PointCloud<pcl::PointXYZ>& point_cloud_pcl, const bool& visualize_depth, const bool& publish_point_cloud);

            tf2::Transform solveVO();

            void publish (const tf2::Transform& world_T_base_last);

        // private: 
            int i, j, count;

            vloam::ImageUtil image_util;
            std::vector<cv::Mat> images;
            std::vector<std::vector<cv::KeyPoint>> keypoints;
            std::vector<cv::Mat> descriptors;
            std::vector<cv::DMatch> matches;

            std::vector<vloam::PointCloudUtil> point_cloud_utils;
            Eigen::MatrixXf point_cloud_3d_tilde;
            pcl::PointCloud<pcl::PointXYZ> point_cloud_pcl;

            float depth0, depth1;
            double angles_0to1[3];
            double t_0to1[3];
            Eigen::Vector3f point_3d_image0_0;
            Eigen::Vector3f point_3d_image0_1;
            Eigen::Vector3f point_3d_rect0_0;
            Eigen::Vector3f point_3d_rect0_1;
            ros::Publisher pub_point_cloud;

            ceres::Solver::Options options;    
            ceres::Solver::Summary summary;

            float angle;
            tf2::Transform cam0_curr_T_cam0_last;
            tf2::Quaternion cam0_curr_q_cam0_last;

        private:
            ros::NodeHandle nh;

            nav_msgs::Odometry visualOdometry;
            ros::Publisher pubvisualOdometry;
            nav_msgs::Path visualPath;
            ros::Publisher pubvisualPath;
    };
}

#endif