#pragma once

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace vloam {
    class VloamTF {
        public:

            void init();
            void processStaticTransform();
            geometry_msgs::TransformStamped VO2Base(const tf2::Transform& cam0_curr_T_cam0_last);

            tf2_ros::StaticTransformBroadcaster static_broadcaster;
            tf2_ros::TransformBroadcaster dynamic_broadcaster;

            std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener;

            geometry_msgs::TransformStamped imu_stamped_tf_velo, map_stamped_tf_velo_origin, imu_stamped_tf_cam0, base_stamped_tf_imu;
            tf2::Transform imu_T_velo, imu_T_cam0, base_T_imu, base_T_cam0, velo_T_cam0;

            float angle;
            geometry_msgs::TransformStamped world_stamped_tf_base; 
            tf2::Transform world_T_base_last, base_last_T_base_curr, cam0_curr_T_cam0_last, velo_last_T_velo_curr;

            Eigen::Isometry3f imu_eigen_T_velo, imu_eigen_T_cam0;

    };
}