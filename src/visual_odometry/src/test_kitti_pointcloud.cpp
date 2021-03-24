#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include "ros/ros.h"
#include "ros/package.h"

// #include <visual_odometry/nanoflann.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

using std::cout;
using std::endl;

const std::string file_path_prefix = "data/2011_09_26/";
const std::string calib_cam_to_cam_file_path = file_path_prefix + "calib_cam_to_cam.txt";
const std::string calib_velo_to_cam_file_path = file_path_prefix + "calib_velo_to_cam.txt";
const std::string bin_file_path = file_path_prefix + "2011_09_26_drive_0001_sync/velodyne_points/data/0000000000.bin";
const std::string image_file_path = file_path_prefix + "2011_09_26_drive_0001_sync/image_00/data/0000000000.png";

const int IMG_HEIGHT = 375;
const int IMG_WIDTH = 1242;

void loadTransformations(Eigen::Matrix4f& T_velo_cam, Eigen::Matrix4f& T_cam_rect0, Eigen::MatrixXf& P_rect0) {
    std::string line;
    std::string delim = " ";
    float value;
    std::string curr;

    std::fstream input1(calib_velo_to_cam_file_path.c_str(), std::ios::in);
    if(!input1.good()){
        std::cerr << "Could not read file: " << calib_velo_to_cam_file_path << endl;
        exit(EXIT_FAILURE);
    }

    while(getline(input1, line)){ //read data from file object and put it into string.
        // std::cout << line << "\n"; //print the data of the string
        int start = 0;
        int end = line.find(delim);
        curr = line.substr(start, end - start);
        // std::cout << curr << ", "; 
        if (curr == "R:") {
            int index = 0;
            while (end != std::string::npos) {
                start = end + delim.length();
                end = line.find(delim, start);
                curr = line.substr(start, end - start);
                value = boost::lexical_cast<float>(line.substr(start, end - start));
                // std::cout << value << ", "; 
                T_velo_cam(index/3, index%3) = value;
                ++index;
            }
        }
        if (curr == "T:") {
            int index = 0;
            while (end != std::string::npos) {
                start = end + delim.length();
                end = line.find(delim, start);
                curr = line.substr(start, end - start);
                value = boost::lexical_cast<float>(line.substr(start, end - start));
                // std::cout << value << ", "; 
                T_velo_cam(index, 3) = value;
                ++index;
            }
        }
    }
    T_velo_cam(3,3) = 1;
    std::cout << "T_velo_cam = \n" << T_velo_cam << "\n" << std::endl;

    std::fstream input2(calib_cam_to_cam_file_path.c_str(), std::ios::in);
    if(!input2.good()){
        std::cerr << "Could not read file: " << calib_cam_to_cam_file_path << endl;
        exit(EXIT_FAILURE);
    }

    while(getline(input2, line)){ //read data from file object and put it into string.
        // std::cout << line << "\n"; //print the data of the string
        int start = 0;
        int end = line.find(delim);
        curr = line.substr(start, end - start);
        // std::cout << curr << ", "; 
        if (curr == "R_rect_00:") {
            int index = 0;
            while (end != std::string::npos) {
                start = end + delim.length();
                end = line.find(delim, start);
                curr = line.substr(start, end - start);
                value = boost::lexical_cast<float>(line.substr(start, end - start));
                // std::cout << value << ", "; 
                T_cam_rect0(index/3, index%3) = value;
                ++index;
            }
            T_cam_rect0(3,3) = 1;
        }
        if (curr == "P_rect_00:") {
            int index = 0;
            while (end != std::string::npos) {
                start = end + delim.length();
                end = line.find(delim, start);
                curr = line.substr(start, end - start);
                value = boost::lexical_cast<float>(line.substr(start, end - start));
                // std::cout << value << ", "; 
                P_rect0(index/4, index%4) = value;
                ++index;
            }
        }
    }

    std::cout << "T_cam_rect0 = \n" << T_cam_rect0 << "\n" << std::endl;
    std::cout << "P_rect0 = \n" << P_rect0 << "\n" << std::endl;
    
    input1.close(); 
    input2.close(); 
}

void loadPointCloud(Eigen::MatrixXf& point_cloud_3d_tilde) {
// void loadPointCloud(std::vector<Eigen::Vector4f>& point_cloud_3d_tilde) {
    // nanoflann supports std::vector and Eigen matrix
    // One official KITTI repo has sample code to read point cloud to pcd: https://github.com/yanii/kitti-pcl/blob/master/src/kitti2pcd.cpp
    // OpenCV also has a projection function: http://docs.ros.org/en/hydro/api/image_geometry/html/c++/classimage__geometry_1_1PinholeCameraModel.html#a30b3761aadfa4b91c7fedb97442c2f13

	// std::fstream input(bin_file_path.c_str(), std::ios::in | std::ios::binary);
	// if(!input.good()){
	// 	std::cerr << "Could not read file: " << bin_file_path << endl;
	// 	exit(EXIT_FAILURE);
	// }

    // // input.seekg (0, input.end);
    // // int length = input.tellg();
    // // input.seekg (0, input.beg);
 
    // // std::cout << length << std::endl; // length \approx point num * 16, eg. 1936240/121016 \approx 15.9998677861

	// input.seekg(0, std::ios::beg);
 
	// // Eigen::MatrixXd point_cloud_3d;
 
	// int i;
    // float x, y, z, intensity;
    
    // Eigen::Vector4f point_3d_tilde;
    // point_3d_tilde(3) = 1; // homogeneous coordinate
	// for (i=0; input.good() && !input.eof(); ++i) {
	// 	input.read((char *) &x, sizeof(float));
	// 	input.read((char *) &y, sizeof(float));
	// 	input.read((char *) &z, sizeof(float));
	// 	input.read((char *) &intensity, sizeof(float)); // TODO: check what to do with intensity. Thresholding?
		
    //     point_3d_tilde(0) = x;
    //     point_3d_tilde(1) = y;
    //     point_3d_tilde(2) = z;

    //     point_cloud_3d_tilde.push_back(point_3d_tilde);

    //     // if (i%100 == 0) {
    //     //     std::cout << i << "-th point is (" << x << ", " << y << ", " << z << ")" << std::endl;
    //     // }
	// }
    // // std::cout << "total num of point is " << point_cloud_3d_tilde.size() << std::endl;
	// input.close();

    int32_t num = 1000000; // about 10 times larger than the real point cloud size
    float *data = (float*)malloc(num*sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen(bin_file_path.c_str(), "rb");
    num = fread(data,sizeof(float), num, stream)/4;
    point_cloud_3d_tilde = Eigen::MatrixXf::Ones(num, 4);
    // std::cout << num << std::endl;
    for (int32_t i=0; i<num; i++) {
        point_cloud_3d_tilde(i, 0) = *px;
        point_cloud_3d_tilde(i, 1) = *py;
        point_cloud_3d_tilde(i, 2) = *pz;
        px+=4; py+=4; pz+=4; pr+=4;
    }

    fclose(stream);
    free(data);
}

void projectPointCloud(const Eigen::MatrixXf& point_cloud_3d_tilde, const Eigen::Matrix4f& T_velo_cam, const Eigen::Matrix4f& T_cam_rect0, const Eigen::MatrixXf& P_rect0, Eigen::MatrixXf& point_cloud_2d) {
    // Projection
    Eigen::MatrixXf point_cloud_3d = point_cloud_3d_tilde * T_velo_cam.transpose() * T_cam_rect0.transpose() * P_rect0.transpose(); // shape = point_cloud_3d_tilde.rows(), 3; in rect cam0 coordinate; last column is the depth

    // Screen out back points
    Eigen::VectorXi select_front = (point_cloud_3d.col(2).array() > 0.1).cast<int>(); // depth should not just be positive, but greater than a threshold
    Eigen::MatrixXf point_cloud_3d_front(select_front.sum(), 3);
    int i, j = 0;
    for (i = 0; i < point_cloud_3d.rows(); ++i) {
        if (select_front(i)) {
            point_cloud_3d_front.row(j) = point_cloud_3d.row(i);
            ++j;
        }
    }

    // From homogeneous to normal coordiante, last column is still depth
    point_cloud_2d = point_cloud_3d_front;
    point_cloud_2d.leftCols(2) = point_cloud_2d.leftCols(2).array().colwise() * Eigen::inverse(point_cloud_2d.col(2).array());
}

void printPointCloud2dStats(const Eigen::MatrixXf& point_cloud_2d) {
    std::cout << "min x = " << point_cloud_2d.col(0).minCoeff() << std::endl;
    std::cout << "max x = " << point_cloud_2d.col(0).maxCoeff() << std::endl;
    std::cout << "mean x = " << point_cloud_2d.col(0).mean() << std::endl;
    std::cout << "std x = " << std::sqrt(
        (point_cloud_2d.col(0).array() - point_cloud_2d.col(0).mean()).square().sum() / (point_cloud_2d.rows() - 1)
    ) << std::endl;

    std::cout << "\nmin y = " << point_cloud_2d.col(1).minCoeff() << std::endl;
    std::cout << "max y = " << point_cloud_2d.col(1).maxCoeff() << std::endl;
    std::cout << "mean y = " << point_cloud_2d.col(1).mean() << std::endl;
    std::cout << "std y = " << std::sqrt(
        (point_cloud_2d.col(1).array() - point_cloud_2d.col(1).mean()).square().sum() / (point_cloud_2d.rows() - 1)
    ) << std::endl;

    std::cout << "\nmin depth = " << point_cloud_2d.col(2).minCoeff() << std::endl;
    std::cout << "max depth = " << point_cloud_2d.col(2).maxCoeff() << std::endl;
    std::cout << "mean depth = " << point_cloud_2d.col(2).mean() << std::endl;
    std::cout << "std depth = " << std::sqrt(
        (point_cloud_2d.col(2).array() - point_cloud_2d.col(2).mean()).square().sum() / (point_cloud_2d.rows() - 1)
    ) << "\n" << std::endl;
    // min depth = 0.100293, max depth = 78.3935
}

std::tuple<Eigen::MatrixXf, Eigen::MatrixXf, Eigen::MatrixXf, Eigen::MatrixXi> downsamplePointCloud(const Eigen::MatrixXf& point_cloud_2d, const int grid_size, Eigen::MatrixXf& point_cloud_2d_dnsp) {
    const int new_width = std::ceil(static_cast<float>(IMG_WIDTH)/static_cast<float>(grid_size));
    const int new_height = std::ceil(static_cast<float>(IMG_HEIGHT)/static_cast<float>(grid_size));
    Eigen::MatrixXf bucket_x = Eigen::MatrixXf::Zero(new_width, new_height); 
    Eigen::MatrixXf bucket_y = Eigen::MatrixXf::Zero(new_width, new_height); 
    Eigen::MatrixXf bucket_depth = Eigen::MatrixXf::Zero(new_width, new_height); Eigen::MatrixXi bucket_count = Eigen::MatrixXi::Zero(new_width, new_height); 

    int index_x, index_y, i, j, global_count = 0;
    for (i=0; i<point_cloud_2d.rows(); ++i) {
        index_x = static_cast<int>(point_cloud_2d(i, 0)/grid_size);
        index_y = static_cast<int>(point_cloud_2d(i, 1)/grid_size);
        if (index_x >=0 and index_x < new_width and index_y >= 0 and index_y < new_height) {
            if (bucket_count(index_x, index_y) == 0) {
                bucket_x(index_x, index_y) = point_cloud_2d(i, 0);
                bucket_y(index_x, index_y) = point_cloud_2d(i, 1);
                bucket_depth(index_x, index_y) = point_cloud_2d(i, 2);
                ++global_count;
            }
            else { // incremental averaging -> TODO: check better averaging method
                bucket_x(index_x, index_y) += (point_cloud_2d(i, 0) - bucket_x(index_x, index_y))/bucket_count(index_x, index_y);
                bucket_y(index_x, index_y) += (point_cloud_2d(i, 1) - bucket_y(index_x, index_y))/bucket_count(index_x, index_y);
                bucket_depth(index_x, index_y) += (point_cloud_2d(i, 2) - bucket_depth(index_x, index_y))/bucket_count(index_x, index_y);
            }
            ++bucket_count(index_x, index_y);
        }
    }

    std::cout << "Point number in FOV after downsample = " << global_count << "\n" << std::endl;
    // grid size = 5 => print \approx 9.5k => ~10 times less points

    point_cloud_2d_dnsp = Eigen::MatrixXf(global_count, 3);
    for (i=0; i<new_width; ++i) {
        for (j=0; j<new_height; ++j) {
            if (bucket_count(i, j) > 0) {
                --global_count;
                point_cloud_2d_dnsp(global_count, 0) = bucket_x(i, j);
                point_cloud_2d_dnsp(global_count, 1) = bucket_y(i, j);
                point_cloud_2d_dnsp(global_count, 2) = bucket_depth(i, j);
            }
        }
    }
    assert(global_count == 0);

    return std::make_tuple(bucket_x, bucket_y, bucket_depth, bucket_count);
}

void visualizePointCloud(cv::Mat& image0_depth, const Eigen::MatrixXf& point_cloud_2d) {
    // cv::Mat point_cloud_2d_image(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
    // TODO: remove points outside of Cam0 FOV
    assert(image0_depth.size().height == IMG_HEIGHT);
    assert(image0_depth.size().width == IMG_WIDTH);

    cv::cvtColor(image0_depth, image0_depth, cv::COLOR_GRAY2BGR);

    float depth, depth_min = 0.1f, depth_max = 50.0f, ratio;
    int i = 0;
    for (i = 0; i < point_cloud_2d.rows(); ++i) {
        depth = point_cloud_2d(i, 2);
        ratio = std::max(std::min((depth-depth_min)/(depth_max-depth_min), 1.0f), 0.0f);
        if (ratio < 0.5) {
            cv::circle(
                image0_depth, 
                cv::Point(point_cloud_2d(i, 0), point_cloud_2d(i, 1)), // x, y
                1,
                cv::Scalar(0, 255*ratio*2, 255*(1-ratio*2)),
                cv::FILLED,
                cv::LINE_8
            );
        }
        else {
            cv::circle(
                image0_depth, 
                cv::Point(point_cloud_2d(i, 0), point_cloud_2d(i, 1)),
                1,
                cv::Scalar(255*(ratio-0.5)*2, 255*(1-(ratio-0.5)*2), 0),
                cv::FILLED,
                cv::LINE_8
            );
        }
    }

    cv::namedWindow("Display Kitti Sample Image With Depth", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Kitti Sample Image With Depth", image0_depth);

    cv::waitKey(0); // wait for key to be pressed
}

float queryDepth(const float x, const float y, const Eigen::MatrixXf& bucket_x, const Eigen::MatrixXf& bucket_y, const Eigen::MatrixXf& bucket_depth, const Eigen::MatrixXi& bucket_count, const int grid_size, const int searching_radius) {
    // grid size and searching radius are respectively recommended to be 5 and 2
    assert(std::ceil(static_cast<float>(IMG_WIDTH)/static_cast<float>(grid_size)) == bucket_x.rows());
    assert(std::ceil(static_cast<float>(IMG_HEIGHT)/static_cast<float>(grid_size)) == bucket_x.cols());

    // float x = static_cast<float>(c);
    // float y = static_cast<float>(IMG_HEIGHT - r);
    int index_x = static_cast<int>(x/grid_size);
    int index_y = static_cast<int>(y/grid_size);
    const int new_width = bucket_x.rows(); // cautious, bucket axis0 is x, axis1 is y => different from image array
    const int new_height = bucket_x.cols();

    // select all neighbors in a certain local block
    int index_x_, index_y_;
    std::vector<Eigen::Vector3f> neighbors;
    Eigen::Vector3f neighbor;
    for (index_x_ = index_x-searching_radius; index_x_ <= index_x+searching_radius; ++index_x_) {
        for (index_y_ = index_y-searching_radius; index_y_ <= index_y+searching_radius; ++index_y_) {
            if (index_x_ >=0 and index_x_ < new_width and index_y_ >= 0 and index_y_ < new_height and bucket_count(index_x_, index_y_) > 0) {
                neighbor(0) = bucket_x(index_x_, index_y_);
                neighbor(1) = bucket_y(index_x_, index_y_);
                neighbor(2) = bucket_depth(index_x_, index_y_);
                neighbors.push_back(neighbor);
            }
        }
    }

    // edge case, no enough neighbors
    if (neighbors.size() < 10)
        return -1.0f; // a fixed unrealistic value representing query failure

    // sort the vector; better ways can be quick select and heapify
    std::sort(neighbors.begin(), neighbors.end(), [&](const Eigen::Vector3f& n1, const Eigen::Vector3f& n2) -> bool {
        float distance_2d_n1 = std::sqrt(std::pow(x - n1(0), 2) + std::pow(y - n1(1), 2));
        float distance_2d_n2 = std::sqrt(std::pow(x - n2(0), 2) + std::pow(y - n2(1), 2));
        return distance_2d_n1 < distance_2d_n2;
    });
    
    float z = (neighbors[0](2) + neighbors[1](2) + neighbors[2](2))/3.0f;  // TODO: weighted distance
    assert(z > 0);
    return z;
    // // naive averaging is already providing good estimation => maybe just check the current bucket_depth is also a fast and good estimation

    // // select 3 nearest neighbor
    // Eigen::Vector3f n1 = neighbors[0]; // n1 here is the \hat{X}_i^{k−1} in eq.10 in DEMO paper
    // n1(0) *= n1(2); // go back to 3d coordinate
    // n1(1) *= n1(2); // go back to 3d coordinate
    // Eigen::Vector3f n2 = neighbors[1];
    // n2(0) *= n2(2);
    // n2(1) *= n2(2);
    // Eigen::Vector3f n3 = neighbors[2];
    // n3(0) *= n3(2);
    // n3(1) *= n3(2);

    // // calculate depth
    // Eigen::Vector3f cp = (n1 - n3).cross(n1 - n2);
    // float z = n1.dot(cp) / (0.0001f + x*cp(0) + y*cp(1) + cp(2));
    // // assert(z > 0); 
    // return z;
}

void visualizeDepth(const Eigen::MatrixXf& bucket_x, const Eigen::MatrixXf& bucket_y, const Eigen::MatrixXf& bucket_depth, const Eigen::MatrixXi& bucket_count) {
    cv::Mat image2 = cv::imread(image_file_path, cv::IMREAD_GRAYSCALE);
    cv::cvtColor(image2, image2, cv::COLOR_GRAY2BGR);

    int x, y;
    float depth, depth_min = 0.1f, depth_max = 50.0f, ratio;    
    for (x=0; x<IMG_WIDTH; x+=3) { // += 3 to make the visualization sparse
        for (y=0; y<IMG_HEIGHT; y+=3) {
            depth = queryDepth(static_cast<float>(x), static_cast<float>(y), bucket_x, bucket_y, bucket_depth, bucket_count, 5, 2);
            if (depth > 0) {
                ratio = std::max(std::min((depth-depth_min)/(depth_max-depth_min), 1.0f), 0.0f);
                if (ratio < 0.5) {
                    cv::circle(
                        image2, 
                        cv::Point(x, y), // x, y
                        1,
                        cv::Scalar(0, 255*ratio*2, 255*(1-ratio*2)),
                        cv::FILLED,
                        cv::LINE_8
                    );
                }
                else {
                    cv::circle(
                        image2, 
                        cv::Point(x, y),
                        1,
                        cv::Scalar(255*(ratio-0.5)*2, 255*(1-(ratio-0.5)*2), 0),
                        cv::FILLED,
                        cv::LINE_8
                    );
                }
            }
        }
    }

    cv::namedWindow("Display Kitti Sample Image With Depth Estimation", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Kitti Sample Image With Depth Estimation", image2);
    cv::imwrite(ros::package::getPath("visual_odometry") + "/figures/gray_image_with_depth_3nn_plane.png", image2);
    cv::waitKey(0); // wait for key to be pressed
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_kitti_pointcloud");

    // nanoflann::KNNResultSet<float> resultSet(3);

    // Eigen::MatrixXd m;

    Eigen::Matrix4f T_velo_cam = Eigen::Matrix4f::Zero();
    Eigen::Matrix4f T_cam_rect0 = Eigen::Matrix4f::Zero();
    Eigen::MatrixXf P_rect0 = Eigen::MatrixXf::Zero(3, 4);
    loadTransformations(T_velo_cam, T_cam_rect0, P_rect0);

    Eigen::MatrixXf point_cloud_3d_tilde; // row size is dynamic, and will be decided when load the point cloud; column size is fixed as 4
    // std::vector<Eigen::Vector4f> point_cloud_3d_tilde; // TODO: check if Eigen::MatrixXd is better for projection multiplication and kdtree query. Check this: https://github.com/yanii/kitti-pcl/blob/3b4ebfd49912702781b7c5b1cf88a00a8974d944/KITTI_README.TXT#L69
    loadPointCloud(point_cloud_3d_tilde);
    std::cout << "total num of point is " << point_cloud_3d_tilde.rows() << "\n" << std::endl;
    // std::cout << "first 5 rows are \n" << point_cloud_3d_tilde.topRows(5) << "\n" << std::endl;

    Eigen::MatrixXf point_cloud_2d;
    projectPointCloud(point_cloud_3d_tilde, T_velo_cam, T_cam_rect0, P_rect0, point_cloud_2d);
    
    printPointCloud2dStats(point_cloud_2d);

    cv::Mat image0 = cv::imread(image_file_path, cv::IMREAD_GRAYSCALE);
    visualizePointCloud(image0, point_cloud_2d);
    // cv::imwrite(ros::package::getPath("visual_odometry") + "/figures/gray_image_with_depth.png", image0);

    Eigen::MatrixXf point_cloud_2d_dnsp;
    Eigen::MatrixXf bucket_x;
    Eigen::MatrixXf bucket_y;
    Eigen::MatrixXf bucket_depth;
    Eigen::MatrixXi bucket_count; 
    std::tie(bucket_x, bucket_y, bucket_depth, bucket_count) = downsamplePointCloud(point_cloud_2d, 5, point_cloud_2d_dnsp);
    cv::Mat image1 = cv::imread(image_file_path, cv::IMREAD_GRAYSCALE);
    visualizePointCloud(image1, point_cloud_2d_dnsp);
    // cv::imwrite(ros::package::getPath("visual_odometry") + "/figures/gray_image_with_depth_dnsp.png", image1);

    // std::cout << bucket_x.rows() << ", " << bucket_x.cols() << std::endl;
    // print: 249, 75

    visualizeDepth(bucket_x, bucket_y, bucket_depth, bucket_count);

    return 0;
}