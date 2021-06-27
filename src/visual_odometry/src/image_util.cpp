#include <visual_odometry/image_util.h>

namespace vloam
{
std::vector<cv::KeyPoint> ImageUtil::detKeypoints(cv::Mat& img)
{
  std::vector<cv::KeyPoint> keypoints;

  if (print_result)
    time = (double)cv::getTickCount();

  if (detector_type == DetectorType::ShiTomasi)
  {
    int block_size =
        5;  //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    // double max_overlap = 0.0; // max. permissible overlap between two features in %
    // double min_distance = (1.0 - max_overlap) * block_size;
    double min_distance = block_size * 1.5;
    // int maxCorners = img.rows * img.cols / std::max(1.0, min_distance); // max. num. of keypoints
    int maxCorners = 1024;

    double quality_level = 0.03;  // minimal accepted quality of image corners
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
  else if (detector_type == DetectorType::FAST)
  {
    int threshold = 100;
    cv::FAST(img, keypoints, threshold, true);
  }
  else
  {
    cv::Ptr<cv::FeatureDetector> detector;
    if (detector_type == DetectorType::BRISK)
      detector = cv::BRISK::create();
    else if (detector_type == DetectorType::ORB)
    {
      int num_features = 2000;
      float scaleFactor = 1.2f;
      int nlevels = 8;
      int edgeThreshold = 31;
      int firstLevel = 0;
      int WTA_K = 2;
      cv::ORB::ScoreType scoreType = cv::ORB::FAST_SCORE;
      int patchSize = 31;
      int fastThreshold = 20;
      detector = cv::ORB::create(num_features, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType,
                                 patchSize, fastThreshold);
    }
    else if (detector_type == DetectorType::AKAZE)
      detector = cv::AKAZE::create();
    else if (detector_type == DetectorType::SIFT)
      detector = cv::SIFT::create();
    else
    {
      std::cerr << "Detector is not implemented" << std::endl;
      exit(EXIT_FAILURE);
    }

    detector->detect(img, keypoints);
  }

  if (print_result)
  {
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << DetectorType_str[static_cast<int>(detector_type)] + "detection with n=" << keypoints.size()
              << " keypoints in " << 1000 * time / 1.0 << " ms" << std::endl;
  }

  if (visualize_result)
  {
    // std::vector<cv::KeyPoint> fake_keypoints;
    // fake_keypoints.push_back(keypoints[0]);
    // std::cout << "fake keypoints 0: " << keypoints[0].pt.x << ", " << keypoints[0].pt.y << std::endl;

    img_keypoints = img.clone();
    // cv::drawKeypoints(img, fake_keypoints, img_keypoints, cv::Scalar::all(-1),
    // cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::drawKeypoints(img, keypoints, img_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::string window_name = DetectorType_str[static_cast<int>(detector_type)] + " Detector Results";
    cv::namedWindow(window_name, 6);
    cv::imshow(window_name, img_keypoints);
    cv::waitKey(0);
  }

  return keypoints;
}

std::vector<cv::KeyPoint> ImageUtil::keyPointsNMS(  // TODO: check if opencv detector minDistance helps here
    std::vector<cv::KeyPoint>&& keypoints,
    const int bucket_width,   // width for horizontal direction in image plane => x, col
    const int bucket_height,  // height for vertical direction in image plane => y, row
    const int max_total_keypoints)
{
  const int bucket_shape_x = std::ceil(static_cast<float>(IMG_WIDTH) / static_cast<float>(bucket_width));    // 13
  const int bucket_shape_y = std::ceil(static_cast<float>(IMG_HEIGHT) / static_cast<float>(bucket_height));  // 4

  const int max_bucket_keypoints = max_total_keypoints / (bucket_shape_x * bucket_shape_y);  // 7

  std::vector<std::vector<std::vector<cv::KeyPoint>>> bucket(
      bucket_shape_x, std::vector<std::vector<cv::KeyPoint>>(bucket_shape_y, std::vector<cv::KeyPoint>()));

  // put all keypoints into buckets
  for (const auto& keypoint : keypoints)
  {
    bucket[static_cast<int>(keypoint.pt.x / static_cast<float>(bucket_width))]
          [static_cast<int>(keypoint.pt.y / static_cast<float>(bucket_height))]
              .push_back(keypoint);
  }

  std::vector<cv::KeyPoint> keypoints_after_NMS;
  keypoints_after_NMS.reserve(max_total_keypoints);

  auto keypoints_sort = [](const cv::KeyPoint& kp0, const cv::KeyPoint& kp1) { return kp0.response > kp1.response; };

  // iterate all bucket, sort and put keypoints with top response to the return
  int col, row;
  for (col = 0; col < bucket_shape_x; ++col)
  {
    for (row = 0; row < bucket_shape_y; ++row)
    {
      if (bucket[col][row].empty())
        continue;

      if (bucket[col][row].size() <= max_bucket_keypoints)
      {
        std::copy(bucket[col][row].begin(), bucket[col][row].end(), std::back_inserter(keypoints_after_NMS));
        continue;
      }

      std::sort(bucket[col][row].begin(), bucket[col][row].end(),
                keypoints_sort);  // ascending order of keypoint response
      std::copy(bucket[col][row].begin(), bucket[col][row].begin() + max_bucket_keypoints,
                std::back_inserter(keypoints_after_NMS));
    }
  }

  return keypoints_after_NMS;
}

void ImageUtil::saveKeypointsImage(const std::string file_name)
{
  if (!img_keypoints.data)
  {
    printf("No keypoints data \n");
    return;
  }
  cv::imwrite(ros::package::getPath("visual_odometry") + "/figures/" + file_name + ".png", img_keypoints);
}

cv::Mat ImageUtil::descKeypoints(std::vector<cv::KeyPoint>& keypoints, cv::Mat& img)
{
  cv::Mat descriptors;

  cv::Ptr<cv::DescriptorExtractor> extractor;
  if (descriptor_type == DescriptorType::BRISK)
  {
    int threshold = 30;          // FAST/AGAST detection threshold score.
    int octaves = 3;             // detection octaves (use 0 to do single scale)
    float pattern_scale = 1.0f;  // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

    extractor = cv::BRISK::create(threshold, octaves, pattern_scale);
  }
  // else if (descriptor_type == DescriptorType::BRIEF) {
  //     extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
  // }
  else if (descriptor_type == DescriptorType::ORB)
  {
    extractor = cv::ORB::create();
  }
  // else if (descriptor_type == DescriptorType::FREAK) {
  //     extractor = cv::xfeatures2d::FREAK::create();
  // }
  else if (descriptor_type == DescriptorType::AKAZE)
  {
    extractor = cv::AKAZE::create();
  }
  else if (descriptor_type == DescriptorType::SIFT)
  {
    extractor = cv::SIFT::create();
  }
  else
  {
    std::cerr << "Decscriptor is not implemented" << std::endl;
    exit(EXIT_FAILURE);
  }

  if (print_result)
    time = (double)cv::getTickCount();

  extractor->compute(img, keypoints, descriptors);

  if (print_result)
  {
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << DescriptorType_str[static_cast<int>(descriptor_type)] << " descriptor extraction in "
              << 1000 * time / 1.0 << " ms" << std::endl;
  }

  return descriptors;
}

std::vector<cv::DMatch> ImageUtil::matchDescriptors(cv::Mat& descriptors0, cv::Mat& descriptors1)
{
  std::vector<cv::DMatch> matches;
  bool crossCheck = (selector_type == SelectType::NN);
  cv::Ptr<cv::DescriptorMatcher> matcher;

  // Reference: https://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html
  if (matcher_type == MatcherType::BF)
  {
    int normType;
    if (descriptor_type == DescriptorType::AKAZE or descriptor_type == DescriptorType::BRISK or
        descriptor_type == DescriptorType::ORB)
    {
      normType = cv::NORM_HAMMING;
    }
    else if (descriptor_type == DescriptorType::SIFT)
    {
      normType = cv::NORM_L2;
    }
    else
    {
      std::cerr << "Decscriptor is not implemented" << std::endl;
    }
    matcher = cv::BFMatcher::create(normType, crossCheck);
  }
  else if (matcher_type == MatcherType::FLANN)
  {
    if (descriptors0.type() != CV_32F)
    {  // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV
       // implementation
      descriptors0.convertTo(descriptors0, CV_32F);
    }
    if (descriptors1.type() != CV_32F)
    {  // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV
       // implementation
      descriptors1.convertTo(descriptors1, CV_32F);
    }

    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
  }

  if (print_result)
    time = (double)cv::getTickCount();

  if (selector_type == SelectType::NN)
  {
    matcher->match(descriptors0, descriptors1, matches);

    if (print_result)
    {
      time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
      std::cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * time / 1.0 << " ms" << std::endl;
    }
  }
  else if (selector_type == SelectType::KNN)
  {  // k nearest neighbors (k=2)
    // double t = (double)cv::getTickCount();

    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher->knnMatch(descriptors0, descriptors1, knn_matches, 2);
    for (const auto& knn_match : knn_matches)
    {
      if (knn_match[0].distance < 0.8 * knn_match[1].distance)
      {
        matches.push_back(knn_match[0]);
      }
    }

    if (print_result)
    {
      time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
      std::cout << " (KNN) with n=" << matches.size() << " matches in " << 1000 * time / 1.0 << " ms" << std::endl;
      std::cout << "KNN matches = " << knn_matches.size() << ", qualified matches = " << matches.size()
                << ", discard ratio = " << (float)(knn_matches.size() - matches.size()) / (float)knn_matches.size()
                << std::endl;
    }
  }

  if (print_result)
    std::cout << "MATCH KEYPOINT DESCRIPTORS done, and the number of matches is " << matches.size() << std::endl;

  return matches;
}

void ImageUtil::visualizeMatchesCallBack(int event, int x, int y)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
  }
}

void visualizeMatchesOnMouse(int ev, int x, int y, int, void* obj)
{
  ImageUtil* iu = static_cast<ImageUtil*>(obj);
  if (iu)
    iu->visualizeMatchesCallBack(ev, x, y);
}

cv::Mat ImageUtil::visualizeMatches(const cv::Mat& image0, const cv::Mat& image1,
                                    const std::vector<cv::KeyPoint>& keypoints0,
                                    const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::DMatch>& matches)
{
  std::vector<cv::DMatch> matches_dnsp;
  const int stride = std::ceil(static_cast<float>(matches.size()) / 100.0f);  // at most 100 points

  int prev_pt_x, prev_pt_y, curr_pt_x, curr_pt_y;
  for (int i = 0; i < matches.size(); i += stride)
  {
    prev_pt_x = keypoints0[matches[i].queryIdx].pt.x;
    prev_pt_y = keypoints0[matches[i].queryIdx].pt.y;
    curr_pt_x = keypoints1[matches[i].trainIdx].pt.x;
    curr_pt_y = keypoints1[matches[i].trainIdx].pt.y;
    if (remove_VO_outlier > 0)
    {
      if (std::pow(prev_pt_x - curr_pt_x, 2) + std::pow(prev_pt_y - curr_pt_y, 2) >
          remove_VO_outlier * remove_VO_outlier)
        continue;
    }
    matches_dnsp.push_back(matches[i]);
  }

  cv::Mat matchImg = image1.clone();
  cv::drawMatches(image0, keypoints0, image1, keypoints1, matches_dnsp, matchImg, cv::Scalar::all(-1),
                  cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);

  // std::string windowName = "Matching keypoints between two camera images";
  // cv::namedWindow(windowName, 7);
  // cv::setMouseCallback(windowName, visualizeMatchesOnMouse, this);
  // cv::imshow(windowName, matchImg);
  // std::cout << "Press key to continue to next image" << std::endl;
  // cv::waitKey(0); // wait for key to be pressed
  // ROS_INFO("image showed");

  return matchImg;
}

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>, std::vector<uchar>> ImageUtil::calculateOpticalFlow(
    const cv::Mat& image0, const cv::Mat& image1, const std::vector<cv::KeyPoint>& keypoints0)
{
  std::vector<cv::Point2f> keypoints0_2f;
  std::vector<cv::Point2f> keypoints1_2f;
  std::vector<uchar> optical_flow_status;

  // transform keypoints to points_2f
  std::transform(keypoints0.cbegin(), keypoints0.cend(), std::back_inserter(keypoints0_2f),
                 [](const cv::KeyPoint& keypoint) { return keypoint.pt; });

  // calculate optical flow
  std::vector<float> err;
  cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
  cv::calcOpticalFlowPyrLK(image0, image1, keypoints0_2f, keypoints1_2f, optical_flow_status, err, cv::Size(15, 15), 2,
                           criteria);

  ROS_INFO("Optical Flow: total candidates = %ld",
           std::count(optical_flow_status.cbegin(), optical_flow_status.cend(), 1));

  return std::make_tuple(std::cref(keypoints0_2f), std::cref(keypoints1_2f), std::cref(optical_flow_status));
}

cv::Mat ImageUtil::visualizeOpticalFlow(const cv::Mat& image1, const std::vector<cv::Point2f>& keypoints0_2f,
                                        const std::vector<cv::Point2f>& keypoints1_2f,
                                        const std::vector<uchar>& optical_flow_status)
{
  // Create some random colors
  cv::Mat image1_color = image1.clone();
  cv::cvtColor(image1_color, image1_color, cv::COLOR_GRAY2BGR);
  cv::RNG rng;
  cv::Scalar color;
  int r, g, b, j;
  for (j = 0; j < keypoints0_2f.size(); ++j)
  {
    // Select good points
    if (optical_flow_status[j] == 1)
    {
      // draw the tracks
      r = rng.uniform(0, 256);
      g = rng.uniform(0, 256);
      b = rng.uniform(0, 256);
      color = cv::Scalar(r, g, b);
      cv::line(image1_color, keypoints1_2f[j], keypoints0_2f[j], color, 2);
      cv::circle(image1_color, keypoints1_2f[j], 3, color, -1);
    }
  }

  return image1_color;
}
cv::Mat ImageUtil::visualizeOpticalFlow(const cv::Mat& image1, const std::vector<cv::KeyPoint>& keypoints0,
                                        const std::vector<cv::KeyPoint>& keypoints1,
                                        const std::vector<cv::DMatch>& matches)
{
  // Create some random colors
  cv::Mat image1_color = image1.clone();
  cv::cvtColor(image1_color, image1_color, cv::COLOR_GRAY2BGR);
  cv::RNG rng;
  cv::Scalar color;
  int r, g, b, j;
  for (const auto match : matches)
  {
    // draw the tracks
    r = rng.uniform(0, 256);
    g = rng.uniform(0, 256);
    b = rng.uniform(0, 256);
    color = cv::Scalar(r, g, b);
    cv::line(image1_color, keypoints1[match.trainIdx].pt, keypoints0[match.queryIdx].pt, color, 2);
    cv::circle(image1_color, keypoints1[match.trainIdx].pt, 3, color, -1);
  }

  return image1_color;
}
}  // namespace vloam