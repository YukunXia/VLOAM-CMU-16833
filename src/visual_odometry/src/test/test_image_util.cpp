#include <visual_odometry/image_util.h>

using namespace vloam;

int main (int argc, char** argv) {
    ImageUtil ft;

    ft.print_result = true;
    ft.visualize_result = true;

    cv::Mat image0 = cv::imread("data/2011_09_26/2011_09_26_drive_0001_sync/image_00/data/0000000000.png", cv::IMREAD_GRAYSCALE);
    cv::Mat image1 = cv::imread("data/2011_09_26/2011_09_26_drive_0001_sync/image_00/data/0000000001.png", cv::IMREAD_GRAYSCALE);

    std::vector<cv::KeyPoint> keypoints0 = ft.detKeypoints(image0);
    std::vector<cv::KeyPoint> keypoints1 = ft.detKeypoints(image1);
    
    cv::Mat descriptors0 = ft.descKeypoints(keypoints0, image0);
    cv::Mat descriptors1 = ft.descKeypoints(keypoints1, image1);

    std::vector<cv::DMatch> matches = ft.matchDescriptors(descriptors0, descriptors1);

    ft.visualizeMatches(image0, image1, keypoints0, keypoints1, matches);

    return 0;
}