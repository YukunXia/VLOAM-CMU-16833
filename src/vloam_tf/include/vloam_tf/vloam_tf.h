#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// #include <tf/transform_datatypes.h>

namespace vloam
{
class VloamTF
{
public:
  void init();
  void processStaticTransform();
  void VO2VeloAndBase(const tf2::Transform &cam0_curr_T_cam0_last);

  void VO2Cam0StartFrame(FILE *VOFilePtr, const int count);
  void LO2Cam0StartFrame(FILE *LOFilePtr, const int count);
  void MO2Cam0StartFrame(FILE *MOFilePtr, const int count);

  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  tf2_ros::TransformBroadcaster dynamic_broadcaster;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  // static transform
  geometry_msgs::TransformStamped imu_stamped_tf_velo, map_stamped_tf_velo_origin, imu_stamped_tf_cam0,
      base_stamped_tf_imu;
  tf2::Transform imu_T_velo, imu_T_cam0, base_T_imu, base_T_cam0, velo_T_cam0;
  Eigen::Isometry3f imu_eigen_T_velo, imu_eigen_T_cam0;

  // dynamic transform for VO
  geometry_msgs::TransformStamped world_stamped_VOtf_base;
  tf2::Transform world_VOT_base_last, base_last_VOT_base_curr, cam0_curr_VOT_cam0_last, velo_last_VOT_velo_curr;
  tf2::Transform cam0_init_VOT_cam0_last, cam0_init_VOT_cam0_start, cam0_start_VOT_cam0_last;
  Eigen::Isometry3f cam0_start_eigen_VOT_cam0_last;

  // dynamic transform for LO
  tf2::Transform world_LOT_base_last, cam0_init_LOT_cam0_last, cam0_init_LOT_cam0_start, cam0_start_LOT_cam0_last;
  Eigen::Isometry3f cam0_start_eigen_LOT_cam0_last;
  tf2::Transform base_prev_LOT_base_curr, cam0_curr_LOT_cam0_prev;

  // dynamic transform for MO
  tf2::Transform world_MOT_base_last, cam0_init_MOT_cam0_last, cam0_init_MOT_cam0_start, cam0_start_MOT_cam0_last;
  Eigen::Isometry3f cam0_start_eigen_MOT_cam0_last;
};
}  // namespace vloam