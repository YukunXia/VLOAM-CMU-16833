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
// #include <tf2_msgs/TFMessage.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <actionlib/server/simple_action_server.h>

#include <visual_odometry/image_util.h>
#include <visual_odometry/point_cloud_util.h>
#include <visual_odometry/ceres_cost_function.h>

#include <vloam_main/vloam_mainAction.h>
#include <vloam_main/vloam_mainFeedback.h>
#include <vloam_main/vloam_mainResult.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/loss_function.h>

typedef actionlib::SimpleActionServer<vloam_main::vloam_mainAction> Server;
vloam_main::vloam_mainFeedback feedback;
vloam_main::vloam_mainResult result;

cv_bridge::CvImagePtr cv_ptr;

int count, i, j;
vloam::ImageUtil image_util;
std::vector<cv::Mat> images;
std::vector<std::vector<cv::KeyPoint>> keypoints;
std::vector<cv::Mat> descriptors;
std::vector<cv::DMatch> matches;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped transformStamped;

std::vector<vloam::PointCloudUtil> point_cloud_utils;
pcl::PointCloud<pcl::PointXYZ> point_cloud_pcl;


float depth0, depth1;
double angles_0to1[3];
double t_0to1[3];
Eigen::Vector3f point_3d_image0_0;
Eigen::Vector3f point_3d_image0_1;
Eigen::Vector3f point_3d_rect0_0;
Eigen::Vector3f point_3d_rect0_1;

ceres::Solver::Options options;    
ceres::Solver::Summary summary;

std::string seq, cmd;
std::ostringstream ss;

void init() {
    count = 0;

    image_util.print_result = false;
    image_util.visualize_result = false;

    images.clear();
    images.resize(2);
    keypoints.clear();
    keypoints.resize(2);
    descriptors.clear();
    descriptors.resize(2);

    point_cloud_utils.clear();
    point_cloud_utils.resize(2);
    point_cloud_utils[0].print_result = false;
    point_cloud_utils[0].downsample_grid_size = 5;
    point_cloud_utils[1].print_result = false;
    point_cloud_utils[1].downsample_grid_size = 5;
}

void callback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg, const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
    i = count%2;

    // Section 1: Process Image // takes ~34ms
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8); // TODO: check the encoding
    images[i] = cv_ptr->image;
    keypoints[i] = image_util.detKeypoints(images[i]);
    descriptors[i] = image_util.descKeypoints(keypoints[i], images[i]);
    if (count > 1)
        matches = image_util.matchDescriptors(descriptors[1-i], descriptors[i]); // first one is prev image, second one is curr image

    
    // process static transformations and camera intrinsics
    if (count == 0) {
        // from velodyne to unrectified camera 00 (gray left)
        Eigen::Isometry3f imu_T_velo, imu_T_cam0;

        transformStamped = tfBuffer.lookupTransform("imu_link", "velo_link", ros::Time::now(), ros::Duration(0.1));
        // NOTE: lookupTransform() will actually block until the transform between the two turtles becomes available (this will usually take a few milliseconds)
        imu_T_velo = tf2::transformToEigen(transformStamped).cast<float>();

        transformStamped = tfBuffer.lookupTransform("imu_link", "camera_gray_left", ros::Time::now(), ros::Duration(0.1));
        imu_T_cam0 = tf2::transformToEigen(transformStamped).cast<float>();

        point_cloud_utils[0].T_velo_cam = imu_T_cam0.matrix().inverse() * imu_T_velo.matrix();
        point_cloud_utils[1].T_velo_cam = imu_T_cam0.matrix().inverse() * imu_T_velo.matrix();

        // from unrectified camera 00 to rectified camera 00
        for (j=0; j<9; ++j) {
            point_cloud_utils[0].T_cam_rect0(j/3, j%3) = camera_info_msg->R[j]; // TODO: optimize this code later
            point_cloud_utils[1].T_cam_rect0(j/3, j%3) = camera_info_msg->R[j]; // assume P doesn't change
        }

        // from rectified camera 00 to image coordinate
        for (j=0; j<12; ++j) {
            point_cloud_utils[0].P_rect0(j/4, j%4) = camera_info_msg->P[j]; // TODO: optimize this code later
            point_cloud_utils[1].P_rect0(j/4, j%4) = camera_info_msg->P[j]; // assume P doesn't change
        }
        // std::cout << "\nP_rect0 = \n" << point_cloud_utils[0].P_rect0 << std::endl; 
    }


    // Section 2: Process Point Cloud // takes ~2.6ms
    pcl::fromROSMsg(*point_cloud_msg, point_cloud_pcl); // optimization can be applied if pcl library is not necessarily
    // ROS_INFO("point cloud width=%d, height=%d", point_cloud_pcl.width, point_cloud_pcl.height); // typical output "point cloud width=122270, height=1053676"
    Eigen::MatrixXf point_cloud_3d_tilde = Eigen::MatrixXf::Ones(point_cloud_pcl.size(), 4);
    for (j=0; j<point_cloud_pcl.size(); ++j) {
        point_cloud_3d_tilde(j, 0) = point_cloud_pcl.points[j].x;
        point_cloud_3d_tilde(j, 1) = point_cloud_pcl.points[j].y;
        point_cloud_3d_tilde(j, 2) = point_cloud_pcl.points[j].z;
    }
    point_cloud_utils[i].point_cloud_3d_tilde = point_cloud_3d_tilde;
    point_cloud_utils[i].projectPointCloud();
    point_cloud_utils[i].downsamplePointCloud();    
    // point_cloud_utils[i].visualizeDepth(cv_ptr->image);
    
    double time = (double)cv::getTickCount();

    // Section 3: Solve VO // takes ~11ms
    ceres::Problem problem;   

    for (j=0; j<3; ++j) {
        angles_0to1[j] = 0.0;
        t_0to1[j] = 0.0;
    }
    // int counter33 = 0, counter32 = 0, counter23 = 0, counter22 = 0;
    for (const auto& match:matches) { // ~ n=1400 matches
        depth0 = point_cloud_utils[1-i].queryDepth(keypoints[1-i][match.queryIdx].pt.x, keypoints[1-i][match.queryIdx].pt.y);
        depth1 = point_cloud_utils[i].queryDepth(keypoints[i][match.trainIdx].pt.x, keypoints[i][match.trainIdx].pt.y);
        if (depth0 > 0 and depth1 > 0) {
            point_3d_image0_0 << keypoints[1-i][match.queryIdx].pt.x*depth0, keypoints[1-i][match.queryIdx].pt.y*depth0, depth0;
            point_3d_image0_1 << keypoints[i][match.trainIdx].pt.x*depth1, keypoints[i][match.trainIdx].pt.y*depth1, depth1;

            point_3d_rect0_0 = (point_cloud_utils[1-i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_0 - point_cloud_utils[1-i].P_rect0.col(3));
            point_3d_rect0_1 = (point_cloud_utils[i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_1 - point_cloud_utils[i].P_rect0.col(3));

            assert(std::abs(point_3d_rect0_0(2) - depth0) < 0.0001);
            assert(std::abs(point_3d_rect0_1(2) - depth1) < 0.0001);

            ceres::CostFunction* cost_function = vloam::CostFunctor33::Create(
                    static_cast<double>(point_3d_rect0_0(0)), 
                    static_cast<double>(point_3d_rect0_0(1)), 
                    static_cast<double>(point_3d_rect0_0(2)), 
                    static_cast<double>(point_3d_rect0_1(0)), 
                    static_cast<double>(point_3d_rect0_1(1)), 
                    static_cast<double>(point_3d_rect0_1(2))
            );
            problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), angles_0to1, t_0to1);
            // ++counter33;
        }
        else if (depth0 > 0 and depth1 <= 0) {
            point_3d_image0_0 << keypoints[1-i][match.queryIdx].pt.x*depth0, keypoints[1-i][match.queryIdx].pt.y*depth0, depth0;
            point_3d_rect0_0 = (point_cloud_utils[1-i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_0 - point_cloud_utils[1-i].P_rect0.col(3));

            assert(std::abs(point_3d_rect0_0(2) - depth0) < 0.0001);

            ceres::CostFunction* cost_function = vloam::CostFunctor32::Create(
                    static_cast<double>(point_3d_rect0_0(0)), 
                    static_cast<double>(point_3d_rect0_0(1)), 
                    static_cast<double>(point_3d_rect0_0(2)), 
                    static_cast<double>(keypoints[i][match.trainIdx].pt.x * 2.0) / static_cast<double>(point_cloud_utils[i].IMG_WIDTH), // normalize xbar ybar to have mean value = 1
                    static_cast<double>(keypoints[i][match.trainIdx].pt.y * 2.0) / static_cast<double>(point_cloud_utils[i].IMG_HEIGHT)
            );
            problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), angles_0to1, t_0to1);
            // ++counter32;
        }
        else if (depth0 <= 0 and depth1 > 0) {
            point_3d_image0_1 << keypoints[i][match.trainIdx].pt.x*depth1, keypoints[i][match.trainIdx].pt.y*depth1, depth1;
            point_3d_rect0_1 = (point_cloud_utils[i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_1 - point_cloud_utils[i].P_rect0.col(3));

            assert(std::abs(point_3d_rect0_1(2) - depth1) < 0.0001);

            ceres::CostFunction* cost_function = vloam::CostFunctor23::Create(
                    static_cast<double>(keypoints[1-i][match.queryIdx].pt.x * 2.0) / static_cast<double>(point_cloud_utils[1-i].IMG_WIDTH), // normalize xbar ybar to have mean value = 1
                    static_cast<double>(keypoints[1-i][match.queryIdx].pt.y * 2.0) / static_cast<double>(point_cloud_utils[1-i].IMG_HEIGHT),
                    static_cast<double>(point_3d_rect0_1(0)), 
                    static_cast<double>(point_3d_rect0_1(1)), 
                    static_cast<double>(point_3d_rect0_1(2))
            );
            problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), angles_0to1, t_0to1);
            // ++counter23;
        }
        else {
            ceres::CostFunction* cost_function = vloam::CostFunctor22::Create(
                    static_cast<double>(keypoints[1-i][match.queryIdx].pt.x * 2.0) / static_cast<double>(point_cloud_utils[1-i].IMG_WIDTH), // normalize xbar ybar to have mean value = 1
                    static_cast<double>(keypoints[1-i][match.queryIdx].pt.y * 2.0) / static_cast<double>(point_cloud_utils[1-i].IMG_HEIGHT),
                    static_cast<double>(keypoints[i][match.trainIdx].pt.x * 2.0) / static_cast<double>(point_cloud_utils[i].IMG_WIDTH), // normalize xbar ybar to have mean value = 1
                    static_cast<double>(keypoints[i][match.trainIdx].pt.y * 2.0) / static_cast<double>(point_cloud_utils[i].IMG_HEIGHT)
            );
            problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), angles_0to1, t_0to1);
            // ++counter22;
        }
    }

    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << "\n";

    ROS_INFO("angles_0to1 = (%f, %f, %f)", angles_0to1[0], angles_0to1[1], angles_0to1[2]); 
    ROS_INFO("t_0to1 = (%f, %f, %f)", t_0to1[0], t_0to1[1], t_0to1[2]); 

    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "Preprocessing 1 frames takes " << 1000 * time / 1.0 << " ms" << std::endl;

    ++count;
}

void execute(const vloam_main::vloam_mainGoalConstPtr& goal, Server* as) {
    result.loading_finished = false;

    init(); // prepare for a new set of estimation

    ss.clear(); ss.str("");
    ss << std::setw(4) << std::setfill('0') << goal->seq;
    seq = std::string(ss.str());
    cmd = "rosbag play " + ros::package::getPath("vloam_main") + "/bags/kitti_" + goal->date + "_drive_" + seq + "_synced.bag -r 0.05"; 
    // TODO: add one more entry of goal for different dataset type: 
    // In kitti2bag, kitti_types = ["raw_synced", "odom_color", "odom_gray"]
    // https://github.com/tomas789/kitti2bag/blob/bf0d46c49a77f5d5500621934ccd617d18cf776b/kitti2bag/kitti2bag.py#L264
    ROS_INFO("The command is %s", cmd.c_str());
    system(cmd.c_str());

    result.loading_finished = true;
    as->setSucceeded(result);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "load_small_dataset");

    ros::NodeHandle nh;

    tf2_ros::TransformListener tfListener(tfBuffer);

    message_filters::Subscriber<sensor_msgs::Image> sub_image00(nh, "/kitti/camera_gray_left/image_raw", 1);
    // message_filters::Subscriber<tf2_msgs::TFMessage> sub_tf(nh, "/tf_static", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera00(nh, "/kitti/camera_gray_left/camera_info", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_velodyne(nh, "/kitti/velo/pointcloud", 1);

    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR; // TODO: check the best solver
    // Reference: http://ceres-solver.org/nnls_solving.html#linearsolver. For small problems (a couple of hundred parameters and a few thousand residuals) with relatively dense Jacobians, DENSE_QR is the method of choice
    // In our case, residual num is 1000~2000, but num of param is only 6
    // options.minimizer_progress_to_stdout = true;

    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_image00, sub_camera00, sub_velodyne);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    Server server(nh, "load_small_dataset_action_server", boost::bind(&execute, _1, &server), false);
    server.start();

    ros::Rate loop_rate(100);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}