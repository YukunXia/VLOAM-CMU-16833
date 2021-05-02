#include <visual_odometry/image_util.h>

namespace vloam {
    std::vector<cv::KeyPoint> ImageUtil::detKeypoints(cv::Mat &img) {
        std::vector<cv::KeyPoint> keypoints;

        if (print_result)
            time = (double)cv::getTickCount();

        if (detector_type == DetectorType::ShiTomasi) {
            int block_size = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
            double max_overlap = 0.0; // max. permissible overlap between two features in %
            double min_distance = (1.0 - max_overlap) * block_size;
            int maxCorners = img.rows * img.cols / std::max(1.0, min_distance); // max. num. of keypoints

            double quality_level = 0.01; // minimal accepted quality of image corners
            double k = 0.04;

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
        }
        else if (detector_type == DetectorType::FAST) {
            int threshold = 100;
            cv::FAST(img, keypoints, threshold, true);
        }
        else {
            cv::Ptr<cv::FeatureDetector> detector;
            if (detector_type == DetectorType::BRISK)
                detector = cv::BRISK::create();
            else if (detector_type == DetectorType::ORB)
                detector = cv::ORB::create();
            else if (detector_type == DetectorType::AKAZE)
                detector = cv::AKAZE::create();
            // else if (detector_type == DetectorType::SIFT)
            //     detector = cv::SIFT::create();
            else {
                std::cerr << "Detector is not implemented" << std::endl;
                exit(EXIT_FAILURE);
            }
            
            detector->detect(img, keypoints);
        }

        if (print_result) {
            time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
            std::cout << DetectorType_str[static_cast<int>(detector_type)] + "detection with n=" << keypoints.size() << " keypoints in " << 1000 * time / 1.0 << " ms" << std::endl;
        }

        if (visualize_result)
        {
            // std::vector<cv::KeyPoint> fake_keypoints;
            // fake_keypoints.push_back(keypoints[0]);
            // std::cout << "fake keypoints 0: " << keypoints[0].pt.x << ", " << keypoints[0].pt.y << std::endl;

            img_keypoints = img.clone();
            // cv::drawKeypoints(img, fake_keypoints, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            cv::drawKeypoints(img, keypoints, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            std::string window_name = DetectorType_str[static_cast<int>(detector_type)] + " Detector Results";
            cv::namedWindow(window_name, 6);
            cv::imshow(window_name, img_keypoints);
            cv::waitKey(0);
        }

        return keypoints;
    }

    void ImageUtil::saveKeypointsImage(const std::string file_name) {    
        if (!img_keypoints.data) {
            printf("No keypoints data \n");
            return;
        }
        cv::imwrite(ros::package::getPath("visual_odometry") + "/figures/" + file_name + ".png", img_keypoints);
    }

    cv::Mat ImageUtil::descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
        cv::Mat descriptors;

        cv::Ptr<cv::DescriptorExtractor> extractor; 
        if (descriptor_type == DescriptorType::BRISK)
        {
            int threshold = 30;        // FAST/AGAST detection threshold score.
            int octaves = 3;           // detection octaves (use 0 to do single scale)
            float pattern_scale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

            extractor = cv::BRISK::create(threshold, octaves, pattern_scale);
        }
        // else if (descriptor_type == DescriptorType::BRIEF) {
        //     extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
        // }
        else if (descriptor_type == DescriptorType::ORB) {
            extractor = cv::ORB::create();
        }
        // else if (descriptor_type == DescriptorType::FREAK) {
        //     extractor = cv::xfeatures2d::FREAK::create();
        // }
        else if (descriptor_type == DescriptorType::AKAZE) {
            extractor = cv::AKAZE::create();
        }    
        // else if (descriptor_type == DescriptorType::SIFT) {
        //     extractor = cv::SIFT::create();
        // }
        else {
            std::cerr << "Decscriptor is not implemented" << std::endl;
            exit(EXIT_FAILURE);
        }

        if (print_result)
            time = (double)cv::getTickCount();

        extractor->compute(img, keypoints, descriptors);

        if (print_result) {
            time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
            std::cout << DescriptorType_str[static_cast<int>(descriptor_type)] << " descriptor extraction in " << 1000 * time / 1.0 << " ms" << std::endl;
        }

        return descriptors;
    }

    std::vector<cv::DMatch> ImageUtil::matchDescriptors(cv::Mat &descriptors0, cv::Mat &descriptors1) {

        std::vector<cv::DMatch> matches;
        bool crossCheck = (selector_type == SelectType::NN);
        cv::Ptr<cv::DescriptorMatcher> matcher;

        // Reference: https://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html
        if (matcher_type == MatcherType::BF) {
            int normType;
            if (descriptor_type == DescriptorType::AKAZE or descriptor_type == DescriptorType::BRISK or descriptor_type == DescriptorType::ORB) {
                normType = cv::NORM_HAMMING;
            }
            // else if (descriptor_type == DescriptorType::SIFT) {
            //     normType = cv::NORM_L2;
            // }
            else {
                std::cerr << "Decscriptor is not implemented" << std::endl;
            }
            matcher = cv::BFMatcher::create(normType, crossCheck);
        }
        else if (matcher_type == MatcherType::FLANN) {
            if (descriptors0.type() != CV_32F) { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
                descriptors0.convertTo(descriptors0, CV_32F);
                descriptors1.convertTo(descriptors1, CV_32F);
            }

            matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        }


        if (print_result)
            time = (double)cv::getTickCount();

        if (selector_type == SelectType::NN) {
            matcher->match(descriptors0, descriptors1, matches);

            if (print_result) {
                time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
                std::cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * time / 1.0 << " ms" << std::endl;
            }
        }
        else if (selector_type == SelectType::KNN) { // k nearest neighbors (k=2)
            // double t = (double)cv::getTickCount();
        
            std::vector<std::vector<cv::DMatch>> knn_matches;
            matcher->knnMatch(descriptors0, descriptors1, knn_matches, 2 );
            for (const auto& knn_match:knn_matches) {
                if (knn_match[0].distance < 0.8*knn_match[1].distance) {
                    matches.push_back(knn_match[0]);
                }
            }

            if (print_result) {
                time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
                std::cout << " (KNN) with n=" << matches.size() << " matches in " << 1000 * time / 1.0 << " ms" << std::endl;
                std::cout << "KNN matches = " << knn_matches.size() << ", qualified matches = " << matches.size() << ", discard ratio = " << (float)(knn_matches.size() - matches.size())/ (float) knn_matches.size() << std::endl;
            }
        }

        if (print_result)
            std::cout << "MATCH KEYPOINT DESCRIPTORS done, and the number of matches is " << matches.size() << std::endl;

        return matches;
    }

    void ImageUtil::visualizeMatchesCallBack(int event, int x, int y) {
        if  ( event == cv::EVENT_LBUTTONDOWN ) {
            std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
        }
    }

    void visualizeMatchesOnMouse(int ev, int x, int y, int, void* obj) {
        ImageUtil* iu = static_cast<ImageUtil*>(obj);
        if (iu)
            iu->visualizeMatchesCallBack(ev, x, y);
    }

    cv::Mat ImageUtil::visualizeMatches(const cv::Mat &image0, const cv::Mat &image1, const std::vector<cv::KeyPoint> &keypoints0, const std::vector<cv::KeyPoint> &keypoints1, const std::vector<cv::DMatch>& matches, const int stride) {
        std::vector<cv::DMatch> matches_dnsp;

        for (int i=0; i<matches.size(); i+=stride) {
            matches_dnsp.push_back(matches[i]);
        }

        cv::Mat matchImg = image1.clone();
        cv::drawMatches(image0, keypoints0,
                        image1, keypoints1,
                        matches_dnsp, matchImg,
                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                        std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // std::string windowName = "Matching keypoints between two camera images";
        // cv::namedWindow(windowName, 7);
        // cv::setMouseCallback(windowName, visualizeMatchesOnMouse, this);
        // cv::imshow(windowName, matchImg);
        // std::cout << "Press key to continue to next image" << std::endl;
        // cv::waitKey(0); // wait for key to be pressed
        // ROS_INFO("image showed");

        return matchImg;
    }
}