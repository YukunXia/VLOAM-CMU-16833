#include <string>
#include <sstream>
// #include <fstream>
#include <iostream>
#include <cstdlib>

#include <ros/ros.h>
#include "ros/package.h"
#include <actionlib/server/simple_action_server.h>

#include <data_loader/load_small_datasetAction.h>
#include <data_loader/load_small_datasetFeedback.h>
#include <data_loader/load_small_datasetResult.h>

#include <opencv2/opencv.hpp>

typedef actionlib::SimpleActionServer<data_loader::load_small_datasetAction> Server;
data_loader::load_small_datasetFeedback feedback;
data_loader::load_small_datasetResult result;


std::string seq, cmd;
std::ostringstream ss;

void execute(const data_loader::load_small_datasetGoalConstPtr& goal, Server* as) {
    result.loading_finished = false;

    ss.clear(); ss.str("");
    ss << std::setw(4) << std::setfill('0') << goal->seq;
    seq = std::string(ss.str());
    cmd = "rosbag play " + ros::package::getPath("data_loader") + "/bags/kitti_" + goal->date + "_drive_" + seq + "_synced.bag"; 
    // TODO: add one more entry of goal for different dataset type: 
    // In kitti2bag, kitti_types = ["raw_synced", "odom_color", "odom_gray"]
    // https://github.com/tomas789/kitti2bag/blob/bf0d46c49a77f5d5500621934ccd617d18cf776b/kitti2bag/kitti2bag.py#L264
    ROS_INFO("The command is %s", cmd.c_str());
    system(cmd.c_str());


    result.loading_finished = true;
    as->setSucceeded(result);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "load_small_dataset");

    ros::NodeHandle nh;

    Server server(nh, "load_small_dataset_action_server", boost::bind(&execute, _1, &server), false);
    server.start();


    ros::Rate loop_rate(100);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}