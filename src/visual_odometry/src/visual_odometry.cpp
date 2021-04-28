#include <visual_odometry/visual_odometry.h>

namespace vloam {
    VisualOdometry::VisualOdometry() {
        pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_follow_VO", 5);
    }

    void VisualOdometry::init(std::shared_ptr<VloamTF>& vloam_tf_) {
        vloam_tf = vloam_tf_;

        count = 0;

        image_util.print_result = false;
        image_util.visualize_result = false;

        images.clear();
        images.resize(2);
        keypoints.clear();
        keypoints.resize(2);
        descriptors.clear();
        descriptors.resize(2);
        matches.clear();

        point_cloud_utils.clear();
        point_cloud_utils.resize(2);
        point_cloud_utils[0].print_result = false;
        point_cloud_utils[0].downsample_grid_size = 5;
        point_cloud_utils[1].print_result = false;
        point_cloud_utils[1].downsample_grid_size = 5;

        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR; // TODO: check the best solver
        // Reference: http://ceres-solver.org/nnls_solving.html#linearsolver. For small problems (a couple of hundred parameters and a few thousand residuals) with relatively dense Jacobians, DENSE_QR is the method of choice
        // In our case, residual num is 1000~2000, but num of param is only 6
        // options.minimizer_progress_to_stdout = true;

        pubvisualOdometry = nh.advertise<nav_msgs::Odometry>("/visual_odom_to_init", 100);
        pubvisualPath = nh.advertise<nav_msgs::Path>("/visual_odom_path", 100);
        visualPath.poses.clear();
    }

    void VisualOdometry::reset() {
        ++count;
        i = count%2;
    }

    void VisualOdometry::processImage(const cv::Mat& img00) {
        images[i] = img00;
        keypoints[i] = image_util.detKeypoints(images[i]);
        descriptors[i] = image_util.descKeypoints(keypoints[i], images[i]);    
        
        if (count > 1)
            matches = image_util.matchDescriptors(descriptors[1-i], descriptors[i]); // first one is prev image, second one is curr image
    }

    void VisualOdometry::setUpPointCloud(const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
        point_cloud_utils[0].cam_T_velo = vloam_tf->imu_eigen_T_cam0.matrix().inverse() * vloam_tf->imu_eigen_T_velo.matrix();
        point_cloud_utils[1].cam_T_velo = vloam_tf->imu_eigen_T_cam0.matrix().inverse() * vloam_tf->imu_eigen_T_velo.matrix();

        // point from unrectified camera 00 to rectified camera 00
        for (j=0; j<9; ++j) {
            point_cloud_utils[0].rect0_T_cam(j/3, j%3) = camera_info_msg->R[j]; // TODO: optimize this code later
            point_cloud_utils[1].rect0_T_cam(j/3, j%3) = camera_info_msg->R[j]; // assume P doesn't change
        }

        // point from rectified camera 00 to image coordinate
        for (j=0; j<12; ++j) {
            point_cloud_utils[0].P_rect0(j/4, j%4) = camera_info_msg->P[j]; // TODO: optimize this code later
            point_cloud_utils[1].P_rect0(j/4, j%4) = camera_info_msg->P[j]; // assume P doesn't change
        }
        // std::cout << "\nP_rect0 = \n" << point_cloud_utils[0].P_rect0 << std::endl; 
    }

    void VisualOdometry::processPointCloud(const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg, const pcl::PointCloud<pcl::PointXYZ>& point_cloud_pcl, const bool& visualize_depth, const bool& publish_point_cloud) {
        point_cloud_3d_tilde = Eigen::MatrixXf::Ones(point_cloud_pcl.size(), 4);
        for (j=0; j<point_cloud_pcl.size(); ++j) {
            point_cloud_3d_tilde(j, 0) = point_cloud_pcl.points[j].x;
            point_cloud_3d_tilde(j, 1) = point_cloud_pcl.points[j].y;
            point_cloud_3d_tilde(j, 2) = point_cloud_pcl.points[j].z;
        }
        point_cloud_utils[i].point_cloud_3d_tilde = point_cloud_3d_tilde;
        point_cloud_utils[i].projectPointCloud();
        point_cloud_utils[i].downsamplePointCloud();    
        if (visualize_depth)
            point_cloud_utils[i].visualizeDepth(images[i]); // uncomment this for depth visualization, but remember to reduce the bag playing speed too
        if (publish_point_cloud) {
            sensor_msgs::PointCloud2 point_cloud_in_VO_msg = *point_cloud_msg;
            // ROS_INFO("point cloud frame id was %s", point_cloud_msg->header.frame_id.c_str());
            point_cloud_in_VO_msg.header.frame_id = "velo";
            point_cloud_in_VO_msg.header.stamp = ros::Time::now();
            pub_point_cloud.publish(point_cloud_in_VO_msg);
        }
    }

    void VisualOdometry::solve() {
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

                // assert(std::abs(point_3d_rect0_0(2) - depth0) < 0.0001);
                // assert(std::abs(point_3d_rect0_1(2) - depth1) < 0.0001);

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

                // assert(std::abs(point_3d_rect0_0(2) - depth0) < 0.0001);

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

                // assert(std::abs(point_3d_rect0_1(2) - depth1) < 0.0001);

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

        // ROS_INFO("angles_0to1 = (%f, %f, %f)", angles_0to1[0], angles_0to1[1], angles_0to1[2]); 
        // ROS_INFO("t_0to1 = (%f, %f, %f)", t_0to1[0], t_0to1[1], t_0to1[2]); 

        cam0_curr_T_cam0_last.setOrigin(tf2::Vector3(t_0to1[0], t_0to1[1], t_0to1[2]));
        angle = std::sqrt(std::pow(angles_0to1[0], 2) + std::pow(angles_0to1[1], 2) + std::pow(angles_0to1[2], 2));
        cam0_curr_q_cam0_last.setRotation(tf2::Vector3(angles_0to1[0]/angle, angles_0to1[1]/angle, angles_0to1[2]/angle), angle);
        cam0_curr_T_cam0_last.setRotation(cam0_curr_q_cam0_last);

        // return cam0_curr_T_cam0_last;
    }

    void VisualOdometry::publish () {
        vloam_tf->VO2BaseVelo(cam0_curr_T_cam0_last);
        vloam_tf->dynamic_broadcaster.sendTransform(vloam_tf->world_stamped_tf_base);

        visualOdometry.header.frame_id = "map";
        visualOdometry.child_frame_id = "visual_odom";
        visualOdometry.header.stamp = ros::Time::now();//image_msg->header.stamp;
        Eigen::Quaterniond q_wodom_curr(vloam_tf->world_T_base_last.getRotation()); // wodom to cam
        Eigen::Vector3d t_wodom_curr(vloam_tf->world_T_base_last.getOrigin()); // wodom to cam
        visualOdometry.pose.pose.orientation.x = q_wodom_curr.x();
        visualOdometry.pose.pose.orientation.y = q_wodom_curr.y();
        visualOdometry.pose.pose.orientation.z = q_wodom_curr.z();
        visualOdometry.pose.pose.orientation.w = q_wodom_curr.w();
        visualOdometry.pose.pose.position.x = t_wodom_curr.x();
        visualOdometry.pose.pose.position.y = t_wodom_curr.y();
        visualOdometry.pose.pose.position.z = t_wodom_curr.z();
        pubvisualOdometry.publish(visualOdometry);

        geometry_msgs::PoseStamped visualPose;
        visualPose.header = visualOdometry.header;
        visualPose.pose = visualOdometry.pose.pose;
        visualPath.header.stamp = visualOdometry.header.stamp;
        visualPath.header.frame_id = "map";
        visualPath.poses.push_back(visualPose);
        pubvisualPath.publish(visualPath);
    }

}