#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "ros/ros.h"
#include "ros/package.h"
// #include <cv_bridge/cv_bridge.h>

using std::cout;
using std::endl;

// Detect keypoints in image using the traditional Shi-Thomasi detector
std::vector<cv::KeyPoint> detKeypointsShiTomasi(cv::Mat &img, const bool& visualize_result, const bool& print_result)
{
    std::vector<cv::KeyPoint> keypoints;

    // compute detector parameters based on image size
    int block_size = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double max_overlap = 0.0; // max. permissible overlap between two features in %
    double min_distance = (1.0 - max_overlap) * block_size;
    int maxCorners = img.rows * img.cols / std::max(1.0, min_distance); // max. num. of keypoints

    double quality_level = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t;
    if (print_result)
        t = (double)cv::getTickCount();

    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, quality_level, min_distance, cv::Mat(), block_size, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {
        cv::KeyPoint new_keypoint;
        new_keypoint.pt = cv::Point2f((*it).x, (*it).y);
        new_keypoint.size = block_size;
        keypoints.push_back(new_keypoint);
    }

    if (print_result) {
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    }

    // visualize results
    if (visualize_result)
    {
        cv::Mat vis_image = img.clone();
        cv::drawKeypoints(img, keypoints, vis_image, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string window_name = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(window_name, 6);
        cv::imshow(window_name, vis_image);
        // cv::imwrite(ros::package::getPath("visual_odometry") + "/figures/gray_image_with_keypoints.png", vis_image);

        cv::waitKey(0);
    }

    return keypoints;
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
cv::Mat descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, const std::string& descriptor_type, const bool& print_result)
{
    cv::Mat descriptors;

    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    if (descriptor_type.compare("BRISK") == 0)
    {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float pattern_scale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, pattern_scale);
    }
    // else if (descriptor_type.compare("BRIEF") == 0) {
    //     extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    // }
    else if (descriptor_type.compare("ORB") == 0) {
        extractor = cv::ORB::create();
    }
    // else if (descriptor_type.compare("FREAK") == 0) {
    //     extractor = cv::xfeatures2d::FREAK::create();
    // }
    else if (descriptor_type.compare("AKAZE") == 0) {
        extractor = cv::AKAZE::create();
    }    
    else if (descriptor_type.compare("SIFT") == 0) {
        extractor = cv::SIFT::create();
    }

    // perform feature description
    double t;
    if (print_result)
        t = (double)cv::getTickCount();

    extractor->compute(img, keypoints, descriptors);

    if (print_result) {
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << descriptor_type << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
    }

    return descriptors;
}

// Find best matches for keypoints in two camera images based on several matching methods
std::vector<cv::DMatch> matchDescriptors(cv::Mat &desc_source, cv::Mat &desc_ref, const std::string descriptor_type, const std::string matcher_type, const std::string selector_type)
{
    std::vector<cv::DMatch> matches;

    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcher_type.compare("MAT_BF") == 0)
    {
        int normType;
        if (descriptor_type == "DES_BINARY") {
            normType = cv::NORM_HAMMING;
        }
        else {
            // if (desc_source.type() != CV_32F)
            // { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            //     desc_source.convertTo(desc_source, CV_32F);
            //     desc_ref.convertTo(desc_ref, CV_32F);
            // }
            normType = cv::NORM_L1;
            // std::cout << "normType = cv::NORM_L1;" << std::endl;
        }
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcher_type.compare("MAT_FLANN") == 0)
    {
        if (desc_source.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            desc_source.convertTo(desc_source, CV_32F);
            desc_ref.convertTo(desc_ref, CV_32F);
        }

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selector_type.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(desc_source, desc_ref, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selector_type.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        // double t = (double)cv::getTickCount();
      
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher->knnMatch( desc_source, desc_ref, knn_matches, 2 );
        for (const auto& knn_match:knn_matches) {
            if (knn_match[0].distance < 0.8*knn_match[1].distance) {
                matches.push_back(knn_match[0]);
            }
        }
      
        // t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        // cout << " (KNN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
        // cout << "KNN matches = " << knn_matches.size() << ", qualified matches = " << matches.size() << ", discard ratio = " << (float)(knn_matches.size() - matches.size())/ (float) knn_matches.size() << endl;
    }

    return matches;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "test_kitti_image");

    cv::Mat image0, image1;

    image0 = cv::imread("data/2011_09_26/2011_09_26_drive_0001_sync/image_00/data/0000000000.png", cv::IMREAD_GRAYSCALE);
    image1 = cv::imread("data/2011_09_26/2011_09_26_drive_0001_sync/image_00/data/0000000001.png", cv::IMREAD_GRAYSCALE);

    if (!image0.data or !image1.data)
    {
        printf("No image data \n");
        return -1;
    }

    // Both should print Image x: rows = 375, cols = 1242
    std::cout << "Image 0: rows = " << image0.size().height << ", cols = " << image0.size().width << std::endl;
    std::cout << "Image 1: rows = " << image1.size().height << ", cols = " << image1.size().width << std::endl;

    // cv::namedWindow("Display Kitti Sample Image", cv::WINDOW_AUTOSIZE);
    // cv::imshow("Display Kitti Sample Image", image0);
    // cv::waitKey(0);

    const bool visualize_result = true;
    const bool print_result = true;

    std::vector<cv::KeyPoint> keypoints0 = detKeypointsShiTomasi(image0, visualize_result, print_result);
    std::vector<cv::KeyPoint> keypoints1 = detKeypointsShiTomasi(image1, visualize_result, print_result);

    const std::string descriptor_type = "ORB"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    cv::Mat descriptors0 = descKeypoints(keypoints0, image0, descriptor_type, print_result);
    cv::Mat descriptors1 = descKeypoints(keypoints1, image1, descriptor_type, print_result);

    std::vector<cv::DMatch> matches = matchDescriptors(descriptors0, descriptors1, "DES_BINARY", "MAT_FLANN", "SEL_KNN");

    if (print_result)
        cout << "#4 : MATCH KEYPOINT DESCRIPTORS done, and the number of matches is " << matches.size() << endl;

    if (visualize_result) {
        cv::Mat matchImg = image1.clone();
        cv::drawMatches(image0, keypoints0,
                        image1, keypoints1,
                        matches, matchImg,
                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                        std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        std::string windowName = "Matching keypoints between two camera images";
        cv::namedWindow(windowName, 7);
        cv::imshow(windowName, matchImg);
        cout << "Press key to continue to next image" << endl;
        cv::waitKey(0); // wait for key to be pressed
    }

    return 0;
}