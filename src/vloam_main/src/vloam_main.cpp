#include <string>
#include <sstream>
// #include <fstream>
#include <iostream>
#include <cstdlib>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
// #include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// #include <visual_odometry/image_util.h>
// #include <visual_odometry/point_cloud_util.h>
// #include <visual_odometry/ceres_cost_function.h>
#include <visual_odometry/visual_odometry.h>
#include <lidar_odometry_mapping/lidar_odometry_mapping.h>

#include <vloam_main/vloam_mainAction.h>
#include <vloam_main/vloam_mainFeedback.h>
#include <vloam_main/vloam_mainResult.h>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/loss_function.h>

ros::NodeHandle* nh_ptr;

double rosbag_rate;
bool visualize_depth, publish_point_cloud;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_image00_ptr;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> sub_camera00_ptr;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_velodyne_ptr;

typedef actionlib::SimpleActionServer<vloam_main::vloam_mainAction> Server;
vloam_main::vloam_mainFeedback feedback;
vloam_main::vloam_mainResult result;

int count, i, j; // TODO: check if count will overflow

ros::Publisher pub_reset_path;

std::shared_ptr<vloam::VisualOdometry> VO;
cv_bridge::CvImagePtr cv_ptr;
pcl::PointCloud<pcl::PointXYZ> point_cloud_pcl;

std::shared_ptr<vloam::LidarOdometryMapping> LOAM;

std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
geometry_msgs::TransformStamped imu_stamped_tf_velo, imu_stamped_tf_cam0, base_stamped_tf_imu;
tf2::Transform imu_T_velo, imu_T_cam0, base_T_imu, base_T_cam0, velo_T_cam0;

std::string seq, cmd;
std::ostringstream ss;

float angle;
geometry_msgs::TransformStamped world_stamped_tf_base; 
tf2::Transform world_T_base_last, base_last_T_base_curr, cam0_curr_T_cam0_last, velo_last_T_velo_curr;

void init() {
    count = 0;

    VO->init();
    LOAM->init(nh_ptr);

    world_T_base_last.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    world_T_base_last.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    world_stamped_tf_base.header.frame_id = "map";
    world_stamped_tf_base.child_frame_id = "base";
    world_stamped_tf_base.transform = tf2::toMsg(world_T_base_last);
}

void callback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg, const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    static tf2_ros::TransformBroadcaster dynamic_broadcaster;

    i = count%2;
    VO->setUpVO();
    LOAM->reset();

    // Section 1: Process Image // takes ~34ms
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    VO->processImage(cv_ptr->image);

    // Section 2: Process Static Transformations and Camera Intrinsics
    if (count == 0) {
        world_stamped_tf_base.header.stamp = ros::Time::now();
        dynamic_broadcaster.sendTransform(world_stamped_tf_base);

        Eigen::Isometry3f imu_eigen_T_velo, imu_eigen_T_cam0;

        imu_stamped_tf_velo = tf_buffer_ptr->lookupTransform("imu_link", "velo_link", ros::Time::now(), ros::Duration(0.1));
        // NOTE: lookupTransform() will actually block until the transform between the two turtles becomes available (this will usually take a few milliseconds)
        tf2::fromMsg(imu_stamped_tf_velo.transform, imu_T_velo); // TODO: later use, if not, remove
        imu_eigen_T_velo = tf2::transformToEigen(imu_stamped_tf_velo).cast<float>(); // for point cloud util internal use
        
        imu_stamped_tf_velo.header.frame_id = "imu";
        imu_stamped_tf_velo.child_frame_id = "velo";
        static_broadcaster.sendTransform(imu_stamped_tf_velo);

        imu_stamped_tf_cam0 = tf_buffer_ptr->lookupTransform("imu_link", "camera_gray_left", ros::Time::now(), ros::Duration(0.1));
        tf2::fromMsg(imu_stamped_tf_cam0.transform, imu_T_cam0); // TODO: later use, if not, remove
        imu_eigen_T_cam0 = tf2::transformToEigen(imu_stamped_tf_cam0).cast<float>(); // for point cloud util internal use

        imu_stamped_tf_cam0.header.frame_id = "imu";
        imu_stamped_tf_cam0.child_frame_id = "cam0";
        static_broadcaster.sendTransform(imu_stamped_tf_cam0);

        base_stamped_tf_imu = tf_buffer_ptr->lookupTransform("base_link", "imu_link", ros::Time::now(), ros::Duration(0.1));
        tf2::fromMsg(base_stamped_tf_imu.transform, base_T_imu); // TODO: later use, if not, remove

        base_stamped_tf_imu.header.frame_id = "base";
        base_stamped_tf_imu.child_frame_id = "imu";
        static_broadcaster.sendTransform(base_stamped_tf_imu);
        
        base_T_cam0 = base_T_imu * imu_T_cam0;
        velo_T_cam0 = imu_T_velo.inverse() * imu_T_cam0;

        VO->setUpPointCloud(imu_eigen_T_cam0, imu_eigen_T_velo, camera_info_msg);
        
        std_msgs::String reset_msg;
        reset_msg.data = "reset";
        pub_reset_path.publish(reset_msg);
    }


    // Section 3: Process Point Cloud // takes ~2.6ms
    pcl::fromROSMsg(*point_cloud_msg, point_cloud_pcl); // optimization can be applied if pcl library is not necessarily
    // ROS_INFO("point cloud width=%d, height=%d", point_cloud_pcl.width, point_cloud_pcl.height); // typical output "point cloud width=122270, height=1053676" // TODO: check why height is so large
    VO->processPointCloud(point_cloud_msg, point_cloud_pcl, visualize_depth, publish_point_cloud);
    // double time = (double)cv::getTickCount();
    
    if (count > 1) {
        // Section 5: Publish VO

        // get T_cam0_last^cam0_curr
        cam0_curr_T_cam0_last = VO->solveVO();

        // get T_base_last^base_curr
        velo_last_T_velo_curr = velo_T_cam0 * cam0_curr_T_cam0_last.inverse() * velo_T_cam0.inverse(); // odom for velodyne
        base_last_T_base_curr = base_T_cam0 * cam0_curr_T_cam0_last.inverse() * base_T_cam0.inverse();

        // get T_world^curr = T_last^curr * T_world^last
        geometry_msgs::Transform temp = tf2::toMsg(base_last_T_base_curr); // TODO: check better solution
        if (!std::isnan(temp.translation.x) and 
            !std::isnan(temp.translation.y) and 
            !std::isnan(temp.translation.z) and
            !std::isnan(temp.rotation.x) and
            !std::isnan(temp.rotation.y) and
            !std::isnan(temp.rotation.z) and
            !std::isnan(temp.rotation.w)) // avoid nan at the first couple steps
            world_T_base_last *= base_last_T_base_curr; // after update, last becomes the curr
        world_stamped_tf_base.header.stamp = ros::Time::now();
        world_stamped_tf_base.transform = tf2::toMsg(world_T_base_last);

        dynamic_broadcaster.sendTransform(world_stamped_tf_base);

        // time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
        // std::cout << "Preprocessing 1 frames takes " << 1000 * time / 1.0 << " ms" << std::endl;
    }

    ROS_INFO("VLOAM MAIN: LOAM Starts; %d", count);
    LOAM->scanRegistrationIO(point_cloud_pcl);
    LOAM->laserOdometryIO();
    // LOAM->laserMappingIO();
    ROS_INFO("VLOAM MAIN: LOAM ends;   %d", count);

    ++count;
}

void execute(const vloam_main::vloam_mainGoalConstPtr& goal, Server* as) {
    result.loading_finished = false;

    // generate new message filter and tf_buffer
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), *sub_image00_ptr, *sub_camera00_ptr, *sub_velodyne_ptr);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    tf_buffer_ptr = std::make_shared<tf2_ros::Buffer>();
    tf2_ros::TransformListener tf_listener(*tf_buffer_ptr);

    init(); // prepare for a new set of estimation

    ss.clear(); ss.str("");
    ss << std::setw(4) << std::setfill('0') << goal->seq;
    seq = std::string(ss.str());
    cmd = "rosbag play " + ros::package::getPath("vloam_main") + "/bags/kitti_" + goal->date + "_drive_" + seq + "_synced.bag -r " + std::to_string(rosbag_rate); 
    // TODO: add one more entry of goal for different dataset type: 
    // In kitti2bag, kitti_types = ["raw_synced", "odom_color", "odom_gray"]
    // https://github.com/tomas789/kitti2bag/blob/bf0d46c49a77f5d5500621934ccd617d18cf776b/kitti2bag/kitti2bag.py#L264
    ROS_INFO("The command is %s", cmd.c_str());
    int sys_ret = system(cmd.c_str());

    result.loading_finished = true;
    as->setSucceeded(result);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "estimation_in_small_dataset");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    nh_private.getParam("rosbag_rate", rosbag_rate);
    nh_private.getParam("visualize_depth", visualize_depth);
    nh_private.getParam("publish_point_cloud", publish_point_cloud);

    ros::NodeHandle nh;
    nh_ptr = &nh;

    VO = std::make_shared<vloam::VisualOdometry>();
    LOAM = std::make_shared<vloam::LidarOdometryMapping>();

    sub_image00_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, "/kitti/camera_gray_left/image_raw", 5);
    sub_camera00_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(nh, "/kitti/camera_gray_left/camera_info", 5);
    sub_velodyne_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh, "/kitti/velo/pointcloud", 5); // TODO: change the buffer size to potentially reduce the data lost

    pub_reset_path = nh.advertise<std_msgs::String>("/syscommand", 5);

    Server server(nh, "load_small_dataset_action_server", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::Rate loop_rate(100);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}