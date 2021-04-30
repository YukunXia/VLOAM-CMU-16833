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
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <visual_odometry/visual_odometry.h>
#include <lidar_odometry_mapping/lidar_odometry_mapping.h>
#include <lidar_odometry_mapping/tic_toc.h>

#include <vloam_main/vloam_mainAction.h>
#include <vloam_main/vloam_mainFeedback.h>
#include <vloam_main/vloam_mainResult.h>

#include <vloam_tf/vloam_tf.h>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/loss_function.h>

double rosbag_rate;
bool visualize_depth, publish_point_cloud, detach_VO_LO;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> sub_image00_ptr;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> sub_camera00_ptr;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_velodyne_ptr;

typedef actionlib::SimpleActionServer<vloam_main::vloam_mainAction> Server;
vloam_main::vloam_mainFeedback feedback;
vloam_main::vloam_mainResult result;

int count, i, j; // TODO: check if count will overflow

std::shared_ptr<vloam::VisualOdometry> VO;
cv_bridge::CvImagePtr cv_ptr;
std::shared_ptr<vloam::LidarOdometryMapping> LOAM;
pcl::PointCloud<pcl::PointXYZ> point_cloud_pcl;
std::shared_ptr<vloam::VloamTF> vloam_tf; 

std::string seq, cmd;
std::ostringstream ss;

void init() {
    count = 0;

    vloam_tf = std::make_shared<vloam::VloamTF>();
    VO = std::make_shared<vloam::VisualOdometry>();
    LOAM = std::make_shared<vloam::LidarOdometryMapping>();

    vloam_tf->init();
    VO->init(vloam_tf);
    LOAM->init(vloam_tf);
}

void callback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg, const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    static tf2_ros::TransformBroadcaster dynamic_broadcaster;

    i = count%2;
    
    VO->reset();
    LOAM->reset();

    // Section 1: Process Image // takes ~34ms
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    VO->processImage(cv_ptr->image);

    // Section 2: Process Static Transformations and Camera Intrinsics
    if (count == 0) {
        vloam_tf->processStaticTransform();
        VO->setUpPointCloud(camera_info_msg);
    }


    // Section 3: Process Point Cloud // takes ~2.6ms
    pcl::fromROSMsg(*point_cloud_msg, point_cloud_pcl); // optimization can be applied if pcl library is not necessarily
    // ROS_INFO("point cloud width=%d, height=%d", point_cloud_pcl.width, point_cloud_pcl.height); // typical output "point cloud width=122270, height=1053676" // TODO: check why height is so large
    VO->processPointCloud(point_cloud_msg, point_cloud_pcl, visualize_depth, publish_point_cloud);
    
    // Section 4: Solve and Publish VO
    if (count > 0) {
        VO->solve();
    }
    VO->publish();

    // Section 5: Solve and Publish LO MO
    LOAM->scanRegistrationIO(point_cloud_pcl);
    LOAM->laserOdometryIO();
    LOAM->laserMappingIO();

    ++count;
    ROS_INFO("time stamp = %f, %f, %f, count = %d", image_msg->header.stamp.toSec(), camera_info_msg->header.stamp.toSec(), point_cloud_msg->header.stamp.toSec(), count);
}

void execute(const vloam_main::vloam_mainGoalConstPtr& goal, Server* as) {
    result.loading_finished = false;

    // generate new message filter and tf_buffer
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), *sub_image00_ptr, *sub_camera00_ptr, *sub_velodyne_ptr);
    sync.setInterMessageLowerBound(ros::Duration(0.09));
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    init(); // prepare for a new set of estimation

    ss.clear(); ss.str("");
    ss << std::setw(4) << std::setfill('0') << goal->seq;
    seq = std::string(ss.str());
    cmd = "rosbag play " + ros::package::getPath("vloam_main") + "/bags/kitti_" + goal->date + "_drive_" + seq + "_synced.bag -d 2 -r " + std::to_string(rosbag_rate); 
    // TODO: add one more entry of goal for different dataset type: 
    // In kitti2bag, kitti_types = ["raw_synced", "odom_color", "odom_gray"]
    // https://github.com/tomas789/kitti2bag/blob/bf0d46c49a77f5d5500621934ccd617d18cf776b/kitti2bag/kitti2bag.py#L264
    ROS_INFO("The command is %s", cmd.c_str());
    int sys_ret = system(cmd.c_str());

    result.loading_finished = true;
    as->setSucceeded(result);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "vloam_main_node");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    nh_private.getParam("rosbag_rate", rosbag_rate);
    nh_private.getParam("visualize_depth", visualize_depth);
    nh_private.getParam("publish_point_cloud", publish_point_cloud);

    ros::NodeHandle nh;

    sub_image00_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, "/kitti/camera_gray_left/image_raw", 10);
    sub_camera00_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo>>(nh, "/kitti/camera_gray_left/camera_info", 10);
    sub_velodyne_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh, "/kitti/velo/pointcloud", 10); // TODO: change the buffer size to potentially reduce the data lost

    Server server(nh, "load_small_dataset_action_server", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::Rate loop_rate(100);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}