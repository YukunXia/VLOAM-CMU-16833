#include <iostream>
#include <sstream>

#include <visual_odometry/image_util.h>
#include <visual_odometry/point_cloud_util.h>
#include <visual_odometry/ceres_cost_function.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/loss_function.h>
#include <Eigen/Dense>

using namespace vloam;

int main (int argc, char** argv) {
    const std::string file_path_prefix = "data/2011_09_26/";
    const std::string calib_cam_to_cam_file_path = file_path_prefix + "calib_cam_to_cam.txt";
    const std::string calib_velo_to_cam_file_path = file_path_prefix + "calib_velo_to_cam.txt";
    std::ostringstream ss0, ss1;
    if (argc == 3) {
        ss0 << std::setw(10) << std::setfill('0') << argv[1]; 
        ss1 << std::setw(10) << std::setfill('0') << argv[2]; 
    }
    else {
        ss0 << std::setw(10) << std::setfill('0') << 0; 
        ss1 << std::setw(10) << std::setfill('0') << 1; 
    }
    std::string img_index0(ss0.str()), img_index1(ss1.str());
    const std::string bin_file_path0 = file_path_prefix + "2011_09_26_drive_0113_sync/velodyne_points/data/" + img_index0 + ".bin";
    const std::string bin_file_path1 = file_path_prefix + "2011_09_26_drive_0113_sync/velodyne_points/data/" + img_index1 + ".bin";
    const std::string image_file_path0 = file_path_prefix + "2011_09_26_drive_0113_sync/image_00/data/" + img_index0 + ".png";
    const std::string image_file_path1 = file_path_prefix + "2011_09_26_drive_0113_sync/image_00/data/" + img_index1 + ".png";

    ImageUtil image_util;
    image_util.print_result = false;
    image_util.visualize_result = false;

    PointCloudUtil point_cloud_util0, point_cloud_util1;
    point_cloud_util0.print_result = false;
    point_cloud_util0.downsample_grid_size = 5;
    point_cloud_util0.loadTransformations(calib_cam_to_cam_file_path, calib_velo_to_cam_file_path);
    point_cloud_util1.print_result = false;
    point_cloud_util1.downsample_grid_size = 5;
    point_cloud_util1.loadTransformations(calib_cam_to_cam_file_path, calib_velo_to_cam_file_path);

    ceres::Problem problem;

    // the following code block takes ~10 ms

    cv::Mat image0 = cv::imread(image_file_path0, cv::IMREAD_GRAYSCALE);
    cv::Mat image1 = cv::imread(image_file_path1, cv::IMREAD_GRAYSCALE);

    point_cloud_util0.loadPointCloud(bin_file_path0);
    point_cloud_util1.loadPointCloud(bin_file_path1);


    // the following code block takes ~63 ms
    // double time = (double)cv::getTickCount();

    std::vector<cv::KeyPoint> keypoints0 = image_util.detKeypoints(image0);
    std::vector<cv::KeyPoint> keypoints1 = image_util.detKeypoints(image1);
    
    cv::Mat descriptors0 = image_util.descKeypoints(keypoints0, image0);
    cv::Mat descriptors1 = image_util.descKeypoints(keypoints1, image1);

    std::vector<cv::DMatch> matches = image_util.matchDescriptors(descriptors0, descriptors1);

    point_cloud_util0.projectPointCloud();
    point_cloud_util0.downsamplePointCloud();
    point_cloud_util1.projectPointCloud();
    point_cloud_util1.downsamplePointCloud();

    if (image_util.visualize_result) {
        point_cloud_util0.visualizeDepth(image_file_path0);
        point_cloud_util1.visualizeDepth(image_file_path1);
        image_util.visualizeMatches(point_cloud_util0.image_with_depth, point_cloud_util1.image_with_depth, keypoints0, keypoints1, matches);
    }

    // time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    // std::cout << "Preprocessing two frames takes " << 1000 * time / 1.0 << " ms" << std::endl;

    // the following code block takes 5.3 ms

    float depth0, depth1;
    double angles_0to1[3] = {0.0, 0.0, 0.0};
    double t_0to1[3] = {0.0, 0.0, 0.0};
    Eigen::Vector3f point_3d_image0_0;
    Eigen::Vector3f point_3d_image0_1;
    Eigen::Vector3f point_3d_rect0_0;
    Eigen::Vector3f point_3d_rect0_1;
    int counter33 = 0, counter32 = 0, counter23 = 0, counter22 = 0;
    for (const auto& match:matches) { // ~ n=1400 matches
        depth0 = point_cloud_util0.queryDepth(keypoints0[match.queryIdx].pt.x, keypoints0[match.queryIdx].pt.y);
        depth1 = point_cloud_util1.queryDepth(keypoints1[match.trainIdx].pt.x, keypoints1[match.trainIdx].pt.y);
        if (depth0 > 0 and depth1 > 0) {
            point_3d_image0_0 << keypoints0[match.queryIdx].pt.x*depth0, keypoints0[match.queryIdx].pt.y*depth0, depth0;
            point_3d_image0_1 << keypoints1[match.trainIdx].pt.x*depth1, keypoints1[match.trainIdx].pt.y*depth1, depth1;

            point_3d_rect0_0 = (point_cloud_util0.P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_0 - point_cloud_util0.P_rect0.col(3));
            point_3d_rect0_1 = (point_cloud_util1.P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_1 - point_cloud_util1.P_rect0.col(3));

            assert(std::abs(point_3d_rect0_0(2) - depth0) < 0.0001);
            assert(std::abs(point_3d_rect0_1(2) - depth1) < 0.0001);

            ceres::CostFunction* cost_function = CostFunctor33::Create(
                    static_cast<double>(point_3d_rect0_0(0)), 
                    static_cast<double>(point_3d_rect0_0(1)), 
                    static_cast<double>(point_3d_rect0_0(2)), 
                    static_cast<double>(point_3d_rect0_1(0)), 
                    static_cast<double>(point_3d_rect0_1(1)), 
                    static_cast<double>(point_3d_rect0_1(2))
            );
            problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), angles_0to1, t_0to1);
            ++counter33;
        }
        else if (depth0 > 0 and depth1 <= 0) {
            point_3d_image0_0 << keypoints0[match.queryIdx].pt.x*depth0, keypoints0[match.queryIdx].pt.y*depth0, depth0;
            point_3d_rect0_0 = (point_cloud_util0.P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_0 - point_cloud_util0.P_rect0.col(3));

            assert(std::abs(point_3d_rect0_0(2) - depth0) < 0.0001);

            ceres::CostFunction* cost_function = CostFunctor32::Create(
                    static_cast<double>(point_3d_rect0_0(0)), 
                    static_cast<double>(point_3d_rect0_0(1)), 
                    static_cast<double>(point_3d_rect0_0(2)), 
                    static_cast<double>(keypoints1[match.trainIdx].pt.x * 2.0) / static_cast<double>(point_cloud_util1.IMG_WIDTH), // normalize xbar ybar to have mean value = 1
                    static_cast<double>(keypoints1[match.trainIdx].pt.y * 2.0) / static_cast<double>(point_cloud_util1.IMG_HEIGHT)
            );
            problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), angles_0to1, t_0to1);
            ++counter32;
        }
        else if (depth0 <= 0 and depth1 > 0) {
            point_3d_image0_1 << keypoints1[match.trainIdx].pt.x*depth1, keypoints1[match.trainIdx].pt.y*depth1, depth1;
            point_3d_rect0_1 = (point_cloud_util1.P_rect0.leftCols(3)).colPivHouseholderQr().solve(point_3d_image0_1 - point_cloud_util1.P_rect0.col(3));

            assert(std::abs(point_3d_rect0_1(2) - depth1) < 0.0001);

            ceres::CostFunction* cost_function = CostFunctor23::Create(
                    static_cast<double>(keypoints0[match.queryIdx].pt.x * 2.0) / static_cast<double>(point_cloud_util1.IMG_WIDTH), // normalize xbar ybar to have mean value = 1
                    static_cast<double>(keypoints0[match.queryIdx].pt.y * 2.0) / static_cast<double>(point_cloud_util1.IMG_HEIGHT),
                    static_cast<double>(point_3d_rect0_1(0)), 
                    static_cast<double>(point_3d_rect0_1(1)), 
                    static_cast<double>(point_3d_rect0_1(2))
            );
            problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), angles_0to1, t_0to1);
            ++counter23;
        }
        else {
            ceres::CostFunction* cost_function = CostFunctor22::Create(
                    static_cast<double>(keypoints0[match.queryIdx].pt.x * 2.0) / static_cast<double>(point_cloud_util1.IMG_WIDTH), // normalize xbar ybar to have mean value = 1
                    static_cast<double>(keypoints0[match.queryIdx].pt.y * 2.0) / static_cast<double>(point_cloud_util1.IMG_HEIGHT),
                    static_cast<double>(keypoints1[match.trainIdx].pt.x * 2.0) / static_cast<double>(point_cloud_util1.IMG_WIDTH), // normalize xbar ybar to have mean value = 1
                    static_cast<double>(keypoints1[match.trainIdx].pt.y * 2.0) / static_cast<double>(point_cloud_util1.IMG_HEIGHT)
            );
            problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), angles_0to1, t_0to1);
            ++counter22;
        }
    }

    std::cout   <<   "Num of 3d to 2d matching = " << counter33 
                << "\nnum of 3d to 2d matching = " << counter32 
                << "\nnum of 2d to 3d matching = " << counter23 
                << "\nnum of 2d to 2d matching = " << counter22 
                << "\n" << std::endl;

    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR; // TODO: check the best solver
    // Reference: http://ceres-solver.org/nnls_solving.html#linearsolver. For small problems (a couple of hundred parameters and a few thousand residuals) with relatively dense Jacobians, DENSE_QR is the method of choice
    // In our case, residual num is 1000~2000, but num of param is only 6
    // options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // std::cout << summary.FullReport() << "\n";

    std::cout << angles_0to1[0] << ", " << angles_0to1[1] << ", " << angles_0to1[2] << std::endl; 
    std::cout << t_0to1[0] << ", " << t_0to1[1] << ", " << t_0to1[2] << std::endl; 

    return 0;
}