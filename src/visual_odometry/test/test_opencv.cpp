#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "test_opencv");

    sensor_msgs::Image::ConstPtr msg;

    cv_bridge::CvImagePtr cvImagePtr; 
    // http://docs.ros.org/en/diamondback/api/cv_bridge/html/c++/classcv__bridge_1_1CvImage.html
    try {
        cvImagePtr = cv_bridge::toCvCopy(msg);
    } 
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::Mat& mat = cvImagePtr->image;

    int i = 0, j = 0;
    if (cvImagePtr->encoding == "mono8") { // /camera/rgb/image_mono
        ROS_INFO("Grey image[%d,%d] = %d", i, j, (int) mat.at<unsigned char>(i, j));
    }
    else if (cvImagePtr->encoding == "bgr8") {
        cv::Vec3b pixel = mat.at<cv::Vec3b>(i,j);
        ROS_INFO("RGB image[%d,%d] = (r=%d,g=%d,b=%d)", i, j, pixel[2], pixel[1], pixel[0]);
    }
    else if (cvImagePtr->encoding == "32FC1") {
        ROS_INFO("Depth image[%d,%d] = %f", i, j, (float) mat.at<float>(i, j));
    }
    // Encoding list: http://docs.ros.org/en/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html

    return 0;
}