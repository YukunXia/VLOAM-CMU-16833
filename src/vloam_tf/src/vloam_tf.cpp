#include <vloam_tf/vloam_tf.h>

namespace vloam
{
void VloamTF::init()
{
  tf_buffer_ptr = std::make_shared<tf2_ros::Buffer>();
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr);

  world_VOT_base_last.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  world_VOT_base_last.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
  world_stamped_VOtf_base.header.frame_id = "map";
  world_stamped_VOtf_base.child_frame_id = "base";
  world_stamped_VOtf_base.transform = tf2::toMsg(world_VOT_base_last);
}

void VloamTF::processStaticTransform()
{
  // initial base to world is static as identity transform
  world_stamped_VOtf_base.header.stamp = ros::Time::now();
  dynamic_broadcaster.sendTransform(world_stamped_VOtf_base);

  imu_stamped_tf_velo = tf_buffer_ptr->lookupTransform("imu_link", "velo_link", ros::Time(0), ros::Duration(3.0));
  // NOTE: lookupTransform() will actually block until the transform between the two turtles becomes available (this
  // will usually take a few milliseconds)
  tf2::fromMsg(imu_stamped_tf_velo.transform, imu_T_velo);                      // TODO: later use, if not, remove
  imu_eigen_T_velo = tf2::transformToEigen(imu_stamped_tf_velo).cast<float>();  // for point cloud util internal use

  imu_stamped_tf_velo.header.frame_id = "imu";
  imu_stamped_tf_velo.child_frame_id = "velo";
  static_broadcaster.sendTransform(imu_stamped_tf_velo);

  map_stamped_tf_velo_origin =
      tf_buffer_ptr->lookupTransform("base_link", "velo_link", ros::Time(0), ros::Duration(3.0));
  map_stamped_tf_velo_origin.header.frame_id = "map";
  map_stamped_tf_velo_origin.child_frame_id = "velo_origin";
  static_broadcaster.sendTransform(map_stamped_tf_velo_origin);

  imu_stamped_tf_cam0 =
      tf_buffer_ptr->lookupTransform("imu_link", "camera_gray_left", ros::Time(0), ros::Duration(3.0));
  tf2::fromMsg(imu_stamped_tf_cam0.transform, imu_T_cam0);                      // TODO: later use, if not, remove
  imu_eigen_T_cam0 = tf2::transformToEigen(imu_stamped_tf_cam0).cast<float>();  // for point cloud util internal use

  imu_stamped_tf_cam0.header.frame_id = "imu";
  imu_stamped_tf_cam0.child_frame_id = "cam0";
  static_broadcaster.sendTransform(imu_stamped_tf_cam0);

  base_stamped_tf_imu = tf_buffer_ptr->lookupTransform("base_link", "imu_link", ros::Time(0), ros::Duration(3.0));
  tf2::fromMsg(base_stamped_tf_imu.transform, base_T_imu);  // TODO: later use, if not, remove

  base_stamped_tf_imu.header.frame_id = "base";
  base_stamped_tf_imu.child_frame_id = "imu";
  static_broadcaster.sendTransform(base_stamped_tf_imu);

  base_T_cam0 = base_T_imu * imu_T_cam0;
  velo_T_cam0 = imu_T_velo.inverse() * imu_T_cam0;
}

void VloamTF::VO2VeloAndBase(const tf2::Transform &cam0_curr_VOT_cam0_last)
{
  // get T_velo_last^velo_curr (from VO)
  velo_last_VOT_velo_curr =
      velo_T_cam0 * cam0_curr_VOT_cam0_last.inverse() * velo_T_cam0.inverse();  // odom for velodyne
  // get T_base_last^base_curr (from VO)
  base_last_VOT_base_curr = base_T_cam0 * cam0_curr_VOT_cam0_last.inverse() * base_T_cam0.inverse();

  // get T_world^curr = T_last^curr * T_world^last
  geometry_msgs::Transform temp = tf2::toMsg(base_last_VOT_base_curr);  // TODO: check better solution
  if (!std::isnan(temp.translation.x) and !std::isnan(temp.translation.y) and !std::isnan(temp.translation.z) and
      !std::isnan(temp.rotation.x) and !std::isnan(temp.rotation.y) and !std::isnan(temp.rotation.z) and
      !std::isnan(temp.rotation.w))                  // avoid nan at the first couple steps
    world_VOT_base_last *= base_last_VOT_base_curr;  // after update, last becomes the curr
  world_stamped_VOtf_base.header.stamp = ros::Time::now();
  world_stamped_VOtf_base.transform = tf2::toMsg(world_VOT_base_last);
}

void VloamTF::VO2Cam0StartFrame(FILE *VOFilePtr, const int count)
{
  if (count < 0)
    return;

  cam0_init_VOT_cam0_last = base_T_cam0.inverse() * world_VOT_base_last * base_T_cam0;

  if (count == 0)
    cam0_init_VOT_cam0_start = cam0_init_VOT_cam0_last;

  cam0_start_VOT_cam0_last = cam0_init_VOT_cam0_start.inverse() * cam0_init_VOT_cam0_last;

  cam0_start_eigen_VOT_cam0_last = tf2::transformToEigen(tf2::toMsg(cam0_start_VOT_cam0_last)).cast<float>();

  if (VOFilePtr != NULL)
  {
    fprintf(VOFilePtr, "%f %f %f %f %f %f %f %f %f %f %f %f\n", cam0_start_eigen_VOT_cam0_last(0, 0),
            cam0_start_eigen_VOT_cam0_last(0, 1), cam0_start_eigen_VOT_cam0_last(0, 2),
            cam0_start_eigen_VOT_cam0_last(0, 3), cam0_start_eigen_VOT_cam0_last(1, 0),
            cam0_start_eigen_VOT_cam0_last(1, 1), cam0_start_eigen_VOT_cam0_last(1, 2),
            cam0_start_eigen_VOT_cam0_last(1, 3), cam0_start_eigen_VOT_cam0_last(2, 0),
            cam0_start_eigen_VOT_cam0_last(2, 1), cam0_start_eigen_VOT_cam0_last(2, 2),
            cam0_start_eigen_VOT_cam0_last(2, 3));
  }
}

void VloamTF::LO2Cam0StartFrame(FILE *LOFilePtr, const int count)
{
  if (count < 0)
    return;

  cam0_init_LOT_cam0_last = base_T_cam0.inverse() * world_LOT_base_last * base_T_cam0;

  if (count == 0)
    cam0_init_LOT_cam0_start = cam0_init_LOT_cam0_last;

  cam0_start_LOT_cam0_last = cam0_init_LOT_cam0_start.inverse() * cam0_init_LOT_cam0_last;

  cam0_start_eigen_LOT_cam0_last = tf2::transformToEigen(tf2::toMsg(cam0_start_LOT_cam0_last)).cast<float>();

  if (LOFilePtr != NULL)
  {
    fprintf(LOFilePtr, "%f %f %f %f %f %f %f %f %f %f %f %f\n", cam0_start_eigen_LOT_cam0_last(0, 0),
            cam0_start_eigen_LOT_cam0_last(0, 1), cam0_start_eigen_LOT_cam0_last(0, 2),
            cam0_start_eigen_LOT_cam0_last(0, 3), cam0_start_eigen_LOT_cam0_last(1, 0),
            cam0_start_eigen_LOT_cam0_last(1, 1), cam0_start_eigen_LOT_cam0_last(1, 2),
            cam0_start_eigen_LOT_cam0_last(1, 3), cam0_start_eigen_LOT_cam0_last(2, 0),
            cam0_start_eigen_LOT_cam0_last(2, 1), cam0_start_eigen_LOT_cam0_last(2, 2),
            cam0_start_eigen_LOT_cam0_last(2, 3));
  }
}

void VloamTF::MO2Cam0StartFrame(FILE *MOFilePtr, const int count)
{
  if (count < 0)
    return;

  cam0_init_MOT_cam0_last = base_T_cam0.inverse() * world_MOT_base_last * base_T_cam0;

  if (count == 0)
    cam0_init_MOT_cam0_start = cam0_init_MOT_cam0_last;

  cam0_start_MOT_cam0_last = cam0_init_MOT_cam0_start.inverse() * cam0_init_MOT_cam0_last;

  cam0_start_eigen_MOT_cam0_last = tf2::transformToEigen(tf2::toMsg(cam0_start_MOT_cam0_last)).cast<float>();

  if (MOFilePtr != NULL)
  {
    fprintf(MOFilePtr, "%f %f %f %f %f %f %f %f %f %f %f %f\n", cam0_start_eigen_MOT_cam0_last(0, 0),
            cam0_start_eigen_MOT_cam0_last(0, 1), cam0_start_eigen_MOT_cam0_last(0, 2),
            cam0_start_eigen_MOT_cam0_last(0, 3), cam0_start_eigen_MOT_cam0_last(1, 0),
            cam0_start_eigen_MOT_cam0_last(1, 1), cam0_start_eigen_MOT_cam0_last(1, 2),
            cam0_start_eigen_MOT_cam0_last(1, 3), cam0_start_eigen_MOT_cam0_last(2, 0),
            cam0_start_eigen_MOT_cam0_last(2, 1), cam0_start_eigen_MOT_cam0_last(2, 2),
            cam0_start_eigen_MOT_cam0_last(2, 3));
  }
}
}  // namespace vloam