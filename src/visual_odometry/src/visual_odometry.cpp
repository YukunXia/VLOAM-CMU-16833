#include <visual_odometry/visual_odometry.h>

namespace vloam {
    VisualOdometry::VisualOdometry() {
        pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_follow_VO", 5);
    }

    void VisualOdometry::init(std::shared_ptr<VloamTF>& vloam_tf_) {
        vloam_tf = vloam_tf_;

        if (!ros::param::get("loam_verbose_level", verbose_level))
            ROS_BREAK();
        if (!ros::param::get("reset_VO_to_identity", reset_VO_to_identity))
            ROS_BREAK();
        if (!ros::param::get("remove_VO_outlier", remove_VO_outlier))
            ROS_BREAK();
        if (!ros::param::get("keypoint_NMS", keypoint_NMS))
            ROS_BREAK();
        if (!ros::param::get("CLAHE", CLAHE))
            ROS_BREAK();
        if (!ros::param::get("visualize_optical_flow", visualize_optical_flow))
            ROS_BREAK();
        if (!ros::param::get("optical_flow_match", optical_flow_match))
            ROS_BREAK();

        count = -1;

        clahe = cv::createCLAHE(2.0);
        image_util.print_result = false;
        image_util.visualize_result = false;
        image_util.detector_type = DetectorType::ShiTomasi;
        image_util.descriptor_type = DescriptorType::ORB;
        image_util.matcher_type = MatcherType::BF;
        image_util.selector_type = SelectType::KNN;
        image_util.remove_VO_outlier = remove_VO_outlier;
        image_util.optical_flow_match = optical_flow_match;

        images.clear();
        images.resize(2);
        keypoints.clear();
        keypoints.resize(2);
        descriptors.clear();
        descriptors.resize(2);
        matches.clear();
        if (optical_flow_match) {
            keypoints_2f.clear();
            keypoints_2f.resize(2);
        }

        point_cloud_utils.clear();
        point_cloud_utils.resize(2);
        point_cloud_utils[0].print_result = false;
        point_cloud_utils[0].downsample_grid_size = 5;
        point_cloud_utils[1].print_result = false;
        point_cloud_utils[1].downsample_grid_size = 5;

        for (j=0; j<3; ++j) {
            angles_0to1[j] = 0.0;
            t_0to1[j] = 0.0;
        }

        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR; // TODO: check the best solver
        // Reference: http://ceres-solver.org/nnls_solving.html#linearsolver. For small problems (a couple of hundred parameters and a few thousand residuals) with relatively dense Jacobians, DENSE_QR is the method of choice
        // In our case, residual num is 1000~2000, but num of param is only 6
        // options.minimizer_progress_to_stdout = true;

        cam0_curr_T_cam0_last.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
        cam0_curr_T_cam0_last.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

        pubvisualOdometry = nh.advertise<nav_msgs::Odometry>("/visual_odom_to_init", 100);
        pubvisualPath = nh.advertise<nav_msgs::Path>("/visual_odom_path", 100);
        visualPath.poses.clear();

        image_transport::ImageTransport it(nh);
        pub_matches_viz = it.advertise("/visual_odometry/matches_visualization", 10);
        pub_depth_viz = it.advertise("/visual_odometry/depth_visualization", 10);
        pub_optical_flow_viz = it.advertise("/visual_odometry/optical_flow_visualization", 10);
    }

    void VisualOdometry::reset() {
        ++count;
        i = count%2;
    }

    void VisualOdometry::processImage(const cv::Mat& img00) {
        TicToc t_process_image;

        // ROS_INFO("image type = %d", img00.type());
        if (CLAHE)
            clahe->apply(img00, images[i]);
        else
            images[i] = img00;

        // if (keypoint_NMS)
        //     keypoints[i] = image_util.keyPointsNMS(image_util.detKeypoints(images[i]));
        // else
        keypoints[i] = image_util.detKeypoints(images[i]); // TODO: might be optimizd
        
        if (!optical_flow_match)
            descriptors[i] = image_util.descKeypoints(keypoints[i], images[i]);

        if (verbose_level > 0) {
            ROS_INFO("keypoint number = %ld \n", keypoints[i].size());
        }
        
        if (count > 0) {
            if (!optical_flow_match)
                matches = image_util.matchDescriptors(descriptors[1-i], descriptors[i]); // first one is prev image, second one is curr image
            else
                std::tie(
                    keypoints_2f[1-i], keypoints_2f[i], optical_flow_status
                ) = image_util.calculateOpticalFlow(images[1-i], images[i], keypoints[i]);
            if (verbose_level > 0) {
                ROS_INFO("match number = %ld \n", matches.size());
            }
        }

        ROS_INFO("Processing image took %f ms +++++\n", t_process_image.toc()); 
    }

    void VisualOdometry::setUpPointCloud(const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
        TicToc t_set_up_point_cloud;

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

        ROS_INFO("Setting up point cloud took %f ms +++++\n", t_set_up_point_cloud.toc()); 
    }

    void VisualOdometry::processPointCloud(const sensor_msgs::PointCloud2ConstPtr &point_cloud_msg, const pcl::PointCloud<pcl::PointXYZ>& point_cloud_pcl, const bool& visualize_depth, const bool& publish_point_cloud) {
        TicToc t_process_point_cloud;

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

        ROS_INFO("Processing point cloud took %f ms +++++\n", t_process_point_cloud.toc()); 
    }

    void VisualOdometry::solveRANSAC() { // TODO: consider KLT case =>  it's not working with KLT 
        TicToc t_ransac;

        std::vector<cv::Point2f> prev_pts, curr_pts;
        prev_pts.resize(matches.size());
        curr_pts.resize(matches.size());
        int j = 0;
        for (const auto& match:matches) { // ~ n=1400 matches 
            prev_pts[j] = keypoints[1-i][match.queryIdx].pt;
            curr_pts[j] = keypoints[i][match.trainIdx].pt;
            ++j;
        }
        
        cv::Mat camera_matrix;
        cv::eigen2cv(point_cloud_utils[0].P_rect0, camera_matrix);
        // std::cout << camera_matrix << std::endl;
        camera_matrix = camera_matrix.colRange(0, 3);
        cv::Mat E = cv::findEssentialMat(prev_pts, curr_pts, camera_matrix, cv::RANSAC, 0.999, 1.0);
        // cv::Mat E = cv::findEssentialMat(prev_pts, curr_pts, camera_matrix, cv::LMEDS);
        cv::Mat R, t;
        cv::Mat mask, triangulated_points;
        int recover_result = cv::recoverPose(E, prev_pts, curr_pts, camera_matrix, R, t, 200.0f);
        // ROS_INFO("recover result is = %d", recover_result);
        // // ROS_INFO("det of R is %.4f", cv::determinant(R));
        // ROS_INFO("R is  \n %.4f, %.4f, %.4f\n %.4f, %.4f, %.4f \n %.4f, %.4f, %.4f \n",
        //                 R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        //                 R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        //                 R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));

        tf2::Transform T;
        tf2::Matrix3x3 R_tf2(
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2)
        );
        T.setBasis(R_tf2);
        T.setOrigin(tf2::Vector3(
            t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)
        ));
        tf2::Quaternion q_tf2 = T.getRotation();

        // ROS_INFO("R quaternion is %.4f, %.4f, %.4f, %.4f", q_tf2.getX(), q_tf2.getY(), q_tf2.getZ(), q_tf2.getW());

        // ROS_INFO("From RANSAC, axis angle = %.4f, %.4f, %.4f", 
        //     q_tf2.getAxis().getX() * q_tf2.getAngle(),
        //     q_tf2.getAxis().getY() * q_tf2.getAngle(),
        //     q_tf2.getAxis().getZ() * q_tf2.getAngle()
        // );
        ROS_INFO("From RANSAC axis = %.4f, %.4f, %.4f, and angle = %.4f", 
            q_tf2.getAxis().getX(),
            q_tf2.getAxis().getY(),
            q_tf2.getAxis().getZ(),
            q_tf2.getAngle()
        );
        ROS_INFO("Froma RANSAC, t = %.4f, %.4f, %.4f", t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0) );
        // std::cout << "t = " << t << std::endl;

        // cam0_curr_T_cam0_last.setRotation(q_tf2);
        // float f2f_distance = std::sqrt(
        //     std::pow(cam0_curr_T_cam0_last.getOrigin().getX(), 2) + 
        //     std::pow(cam0_curr_T_cam0_last.getOrigin().getY(), 2) + 
        //     std::pow(cam0_curr_T_cam0_last.getOrigin().getZ(), 2)
        // );
        // cam0_curr_T_cam0_last.setOrigin(tf2::Vector3(
        //     f2f_distance * t.at<double>(0,0),
        //     f2f_distance * t.at<double>(1,0),
        //     f2f_distance * t.at<double>(2,0)
        // ));
        
        ROS_INFO("RANSAC VO took %f ms +++++\n", t_ransac.toc()); 
    }

    void VisualOdometry::solveNlsAll() {
        TicToc t_nls_all;

        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::Problem problem;   
        
        if (reset_VO_to_identity) {
            for (j=0; j<3; ++j) {
                angles_0to1[j] = 0.0;
                t_0to1[j] = 0.0;
            }
        }
        else {
            // init from LO
            t_0to1[0] = vloam_tf->cam0_curr_LOT_cam0_prev.getOrigin().getX();
            t_0to1[1] = vloam_tf->cam0_curr_LOT_cam0_prev.getOrigin().getY();
            t_0to1[2] = vloam_tf->cam0_curr_LOT_cam0_prev.getOrigin().getZ();
            angles_0to1[0] = vloam_tf->cam0_curr_LOT_cam0_prev.getRotation().getAxis().getX() * vloam_tf->cam0_curr_LOT_cam0_prev.getRotation().getAngle();
            angles_0to1[1] = vloam_tf->cam0_curr_LOT_cam0_prev.getRotation().getAxis().getY() * vloam_tf->cam0_curr_LOT_cam0_prev.getRotation().getAngle();
            angles_0to1[2] = vloam_tf->cam0_curr_LOT_cam0_prev.getRotation().getAxis().getZ() * vloam_tf->cam0_curr_LOT_cam0_prev.getRotation().getAngle();
        }
        
        int prev_pt_x, prev_pt_y, curr_pt_x, curr_pt_y;
        int counter33 = 0, counter32 = 0, counter23 = 0, counter22 = 0;
        int match_num = (optical_flow_match) ? keypoints_2f[i].size() : matches.size();
        for (int j=0; j<match_num; ++j) { // ~ n=1400 matches

            if (!optical_flow_match) {
                prev_pt_x = keypoints[1-i][matches[j].queryIdx].pt.x;
                prev_pt_y = keypoints[1-i][matches[j].queryIdx].pt.y;
                curr_pt_x = keypoints[i][matches[j].trainIdx].pt.x;
                curr_pt_y = keypoints[i][matches[j].trainIdx].pt.y;
            }
            else {
                if (optical_flow_status[j] == 1) {
                    prev_pt_x = keypoints_2f[1-i][j].x;
                    prev_pt_y = keypoints_2f[1-i][j].y;
                    curr_pt_x = keypoints_2f[i][j].x;
                    curr_pt_y = keypoints_2f[i][j].y;
                }
                else
                    continue;
            }
            

            if (remove_VO_outlier > 0) {
                if (std::pow(prev_pt_x - curr_pt_x, 2) + std::pow(prev_pt_y - curr_pt_y, 2) > remove_VO_outlier*remove_VO_outlier)
                    continue;
            }

            depth0 = point_cloud_utils[1-i].queryDepth(prev_pt_x, prev_pt_y);
            depth1 = point_cloud_utils[i].queryDepth(curr_pt_x, curr_pt_y);

            // if (std::abs(depth0 - depth1) > 3.0)
            //     continue;

            // if (depth0 > 0.5 and depth1 > 0.5) {
            //     point_3d_image0_0 << prev_pt_x*depth0, prev_pt_y*depth0, depth0;
            //     point_3d_image0_1 << curr_pt_x*depth1, curr_pt_y*depth1, depth1;

            //     point_3d_rect0_0 = (point_cloud_utils[1-i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_0);
            //     point_3d_rect0_1 = (point_cloud_utils[i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_1);

            //     // assert(std::abs(point_3d_rect0_0(2) - depth0) < 0.0001);
            //     // assert(std::abs(point_3d_rect0_1(2) - depth1) < 0.0001);

            //     ceres::CostFunction* cost_function = vloam::CostFunctor33::Create(
            //             static_cast<double>(point_3d_rect0_0(0)), 
            //             static_cast<double>(point_3d_rect0_0(1)), 
            //             static_cast<double>(point_3d_rect0_0(2)), 
            //             static_cast<double>(point_3d_rect0_1(0)), 
            //             static_cast<double>(point_3d_rect0_1(1)), 
            //             static_cast<double>(point_3d_rect0_1(2))
            //     );
            //     problem.AddResidualBlock(cost_function, loss_function, angles_0to1, t_0to1);
            //     ++counter33;
            // }
            // else if (depth0 > 0.5 and depth1 <= 0.5) {
            if (depth0 > 0) {
                point_3d_image0_0 << prev_pt_x*depth0, prev_pt_y*depth0, depth0;
                point_3d_image0_1 << curr_pt_x, curr_pt_y, 1.0f;

                point_3d_rect0_0 = (point_cloud_utils[1-i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_0); // assume point_cloud_utils[i].P_rect0.col(3) is zero
                point_3d_rect0_1 = (point_cloud_utils[i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_1); // assume point_cloud_utils[i].P_rect0.col(3) is zero

                // assert(std::abs(point_3d_rect0_0(2) - depth0) < 0.0001);

                ceres::CostFunction* cost_function = vloam::CostFunctor32::Create(
                        static_cast<double>(point_3d_rect0_0(0)), 
                        static_cast<double>(point_3d_rect0_0(1)), 
                        static_cast<double>(point_3d_rect0_0(2)), 
                        static_cast<double>(point_3d_rect0_1(0)) / static_cast<double>(point_3d_rect0_1(2)),
                        static_cast<double>(point_3d_rect0_1(1)) / static_cast<double>(point_3d_rect0_1(2))
                );
                problem.AddResidualBlock(cost_function, loss_function, angles_0to1, t_0to1);
                ++counter32;
            }

            // else if (depth0 <= 0.5 and depth1 > 0.5) {
            // // if (depth1 > 0) {
            //     point_3d_image0_0 << prev_pt_x, prev_pt_y, 1.0f;
            //     point_3d_image0_1 << curr_pt_x*depth1, curr_pt_y*depth1, depth1;

            //     point_3d_rect0_0 = (point_cloud_utils[1-i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_0); // assume point_cloud_utils[i].P_rect0.col(3) is zero
            //     point_3d_rect0_1 = (point_cloud_utils[i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_1); // assume point_cloud_utils[i].P_rect0.col(3) is zero

            // //     // assert(std::abs(point_3d_rect0_1(2) - depth1) < 0.0001);

            //     ceres::CostFunction* cost_function = vloam::CostFunctor23::Create(
            //             static_cast<double>(point_3d_rect0_0(0)) / static_cast<double>(point_3d_rect0_0(2)),
            //             static_cast<double>(point_3d_rect0_0(1)) / static_cast<double>(point_3d_rect0_0(2)),
            //             static_cast<double>(point_3d_rect0_1(0)), 
            //             static_cast<double>(point_3d_rect0_1(1)), 
            //             static_cast<double>(point_3d_rect0_1(2))
            //     );
            //     problem.AddResidualBlock(cost_function, loss_function, angles_0to1, t_0to1);
            //     ++counter23;
            // }
            else {
            // if (depth0 < 0 and depth1 < 0) {
                point_3d_image0_0 << prev_pt_x, prev_pt_y, 1.0f;
                point_3d_image0_1 << curr_pt_x, curr_pt_y, 1.0f;

                point_3d_rect0_0 = (point_cloud_utils[1-i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_0); // assume point_cloud_utils[i].P_rect0.col(3) is zero
                point_3d_rect0_1 = (point_cloud_utils[i].P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_1); // assume point_cloud_utils[i].P_rect0.col(3) is zero

                ceres::CostFunction* cost_function = vloam::CostFunctor22::Create(
                        static_cast<double>(point_3d_rect0_0(0)) / static_cast<double>(point_3d_rect0_0(2)),
                        static_cast<double>(point_3d_rect0_0(1)) / static_cast<double>(point_3d_rect0_0(2)),
                        static_cast<double>(point_3d_rect0_1(0)) / static_cast<double>(point_3d_rect0_1(2)),
                        static_cast<double>(point_3d_rect0_1(1)) / static_cast<double>(point_3d_rect0_1(2))
                );
                problem.AddResidualBlock(cost_function, loss_function, angles_0to1, t_0to1);
                ++counter22;
            }
        }

        // ROS_INFO("counter33 = %d", counter33);
        ROS_INFO("counter32 = %d", counter32);
        // ROS_INFO("counter23 = %d", counter33);
        ROS_INFO("counter22 = %d", counter22);

        ceres::Solve(options, &problem, &summary);
        // std::cout << summary.FullReport() << "\n";

        cam0_curr_T_cam0_last.setOrigin(tf2::Vector3(t_0to1[0], t_0to1[1], t_0to1[2]));
        angle = std::sqrt(std::pow(angles_0to1[0], 2) + std::pow(angles_0to1[1], 2) + std::pow(angles_0to1[2], 2));
        cam0_curr_q_cam0_last.setRotation(tf2::Vector3(angles_0to1[0]/angle, angles_0to1[1]/angle, angles_0to1[2]/angle), angle);
        cam0_curr_T_cam0_last.setRotation(cam0_curr_q_cam0_last);
    

        ROS_INFO("angles_0to1 = (%.4f, %.4f, %.4f)", angles_0to1[0], angles_0to1[1], angles_0to1[2]); 
        // ROS_INFO("q_0to1 = (%.4f, %.4f, %.4f, %.4f)", cam0_curr_q_cam0_last.getX(), cam0_curr_q_cam0_last.getY(), cam0_curr_q_cam0_last.getZ(), cam0_curr_q_cam0_last.getW()); 
        ROS_INFO("From nllsq axis = %.4f, %.4f, %.4f, and angle = %.4f", 
            cam0_curr_q_cam0_last.getAxis().getX(),
            cam0_curr_q_cam0_last.getAxis().getY(),
            cam0_curr_q_cam0_last.getAxis().getZ(),
            cam0_curr_q_cam0_last.getAngle()
        );
        ROS_INFO("t_0to1 = (%.4f, %.4f, %.4f)", t_0to1[0], t_0to1[1], t_0to1[2]); 
    
        // ROS_INFO("From LM axis = %.4f, %.4f, %.4f, and angle = %.4f", 
        //     cam0_curr_q_cam0_last.getAxis().getX(),
        //     cam0_curr_q_cam0_last.getAxis().getY(),
        //     cam0_curr_q_cam0_last.getAxis().getZ(),
        //     cam0_curr_q_cam0_last.getAngle()
        // );

        // return cam0_curr_T_cam0_last;

        ROS_INFO("Nonlinear Least square (ALL) VO took %f ms +++++\n", t_nls_all.toc()); 
    }

    void VisualOdometry::publish () {

        visualOdometry.header.frame_id = "map";
        visualOdometry.child_frame_id = "visual_odom";
        visualOdometry.header.stamp = ros::Time::now();//image_msg->header.stamp;
        Eigen::Quaterniond q_wodom_curr(vloam_tf->world_VOT_base_last.getRotation()); // wodom to cam
        Eigen::Vector3d t_wodom_curr(vloam_tf->world_VOT_base_last.getOrigin()); // wodom to cam
        visualOdometry.pose.pose.orientation.x = q_wodom_curr.x();
        visualOdometry.pose.pose.orientation.y = q_wodom_curr.y();
        visualOdometry.pose.pose.orientation.z = q_wodom_curr.z();
        visualOdometry.pose.pose.orientation.w = q_wodom_curr.w();
        visualOdometry.pose.pose.position.x = t_wodom_curr.x();
        visualOdometry.pose.pose.position.y = t_wodom_curr.y();
        visualOdometry.pose.pose.position.z = t_wodom_curr.z();
        pubvisualOdometry.publish(visualOdometry);
        // ROS_INFO("publish visualOdometry x = %f and %f", vloam_tf->world_VOT_base_last.getOrigin().x(), t_wodom_curr.x());

        // vloam_tf->world_VOT_base_last.setOrigin(tf2::Vector3(
        //     t_wodom_curr.x(),
        //     t_wodom_curr.y(),
        //     t_wodom_curr.z()
        // ));
        // vloam_tf->world_VOT_base_last.setRotation(tf2::Quaternion(
        //     q_wodom_curr.x(),
        //     q_wodom_curr.y(),
        //     q_wodom_curr.z(),
        //     q_wodom_curr.w()
        // ));


        geometry_msgs::PoseStamped visualPose;
        visualPose.header = visualOdometry.header;
        visualPose.pose = visualOdometry.pose.pose;
        visualPath.header.stamp = visualOdometry.header.stamp;
        visualPath.header.frame_id = "map";
        visualPath.poses.push_back(visualPose);
        pubvisualPath.publish(visualPath);

        if (verbose_level > 0 and count > 1) {
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            cv::Mat match_image = image_util.visualizeMatches(images[1-i], images[i], keypoints[1-i], keypoints[i], matches);
            matches_viz_cvbridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, match_image);
            pub_matches_viz.publish(matches_viz_cvbridge.toImageMsg());
        }

        if (verbose_level > 0 and count > 0) {
            std_msgs::Header header;
            header.stamp = ros::Time::now();
            cv::Mat depth_image = point_cloud_utils[i].visualizeDepth(images[i]);
            depth_viz_cvbridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, depth_image);
            pub_depth_viz.publish(depth_viz_cvbridge.toImageMsg());
        }

        if (verbose_level > 0 and count > 0 and visualize_optical_flow) {
            std_msgs::Header header;
            header.stamp = ros::Time::now();

            cv::Mat optical_flow_image;
            if (optical_flow_match)
                optical_flow_image = image_util.visualizeOpticalFlow(images[i], keypoints_2f[1-i], keypoints_2f[i], optical_flow_status);
            else
                optical_flow_image = image_util.visualizeOpticalFlow(images[i], keypoints[1-i], keypoints[i], matches);

            optical_flow_viz_cvbridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, optical_flow_image);
            pub_optical_flow_viz.publish(optical_flow_viz_cvbridge.toImageMsg());
        }
    }

}