#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "ros/package.h"

#ifndef IMAGE_UTIL_H
#define IMAGE_UTIL_H

namespace vloam {
    enum class DetectorType {ShiTomasi, BRISK, FAST, ORB, AKAZE, SIFT};
    static const std::string DetectorType_str[] = {
        "ShiTomasi", "BRISK", "FAST", "ORB", "AKAZE", "SIFT"
    };
    enum class DescriptorType {BRISK, ORB, BRIEF, AKAZE, FREAK, SIFT};
    static const std::string DescriptorType_str[] = {
        "BRISK", "ORB", "BRIEF", "AKAZE", "FREAK", "SIFT"
    };
    enum class MatcherType {BF, FLANN};
    enum class SelectType {NN, KNN};

    class ImageUtil {
        public:
            ImageUtil() {
                print_result = false;
                visualize_result = false;
                detector_type = DetectorType::ShiTomasi;
                descriptor_type = DescriptorType::ORB;
                matcher_type = MatcherType::FLANN;
                selector_type = SelectType::KNN;
            }

            std::vector<cv::KeyPoint> detKeypoints(cv::Mat &img);
            void saveKeypointsImage(const std::string file_name);
            cv::Mat descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img);
            std::vector<cv::DMatch> matchDescriptors(cv::Mat &desc_source, cv::Mat &desc_ref);
            void visualizeMatches(const cv::Mat &image0, const cv::Mat &image1, const std::vector<cv::KeyPoint> &keypoints0, const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::DMatch>& matches);

            // std::string path_prefix;
            bool print_result;
            bool visualize_result;
            DetectorType detector_type;
            DescriptorType descriptor_type;
            MatcherType matcher_type;
            SelectType selector_type;

        private:
            double time;
            cv::Mat img_keypoints;
    };
}

#endif