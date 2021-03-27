#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <actionlib/server/simple_action_server.h>

#include <data_loader/load_small_datasetAction.h>
#include <data_loader/load_small_datasetFeedback.h>
#include <data_loader/load_small_datasetResult.h>

typedef actionlib::SimpleActionServer<data_loader::load_small_datasetAction> Server;
data_loader::load_small_datasetFeedback feedback;
data_loader::load_small_datasetResult result;

// std::string date, seq;
std::string file_path_prefix;
std::string calib_cam_to_cam_file_path;
std::string calib_velo_to_cam_file_path;
std::string bin_file_prefix;
std::string image_file_prefix;
std::string timestamps_file_path;
std::ostringstream ss;

ros::Publisher pub_point_cloud;
ros::Publisher pub_image;
pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_msg;

void getDataPath(const std::string& date, std::string seq) {
    ss.clear(); ss.str("");
    ss << std::setw(4) << std::setfill('0') << seq;
    seq = std::string(ss.str());

    file_path_prefix = "data/" + date + "/";
    calib_cam_to_cam_file_path = file_path_prefix + "calib_cam_to_cam.txt";
    calib_velo_to_cam_file_path = file_path_prefix + "calib_velo_to_cam.txt";
    timestamps_file_path = file_path_prefix + date + "_drive_" + seq + "_sync/" + "image_00/timestamps.txt";
    bin_file_prefix = file_path_prefix + date + "_drive_" + seq + "_sync/" + "velodyne_points/data/";
    image_file_prefix = file_path_prefix + date + "_drive_" + seq + "_sync/" + "image_00/data/";
}

int getDataLength() {
    int data_length = 0;

    std::string line;
    std::ifstream time_stamp_file(timestamps_file_path);

    if(time_stamp_file.is_open()) {
        while(!time_stamp_file.eof()){
            getline(time_stamp_file,line);
            if (line.size() > 0)
                ++data_length;
        }
        time_stamp_file.close();
    }
    else {
        std::cerr << "file is not valid" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << "This dataset has " << data_length << " pieces of data\n" << std::endl;

    return data_length;
}

void loadPointCloud(const std::string bin_file_path) {
    int32_t num = 1000000; // about 10 times larger than the real point cloud size
    float *data = (float*)malloc(num*sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;

    // load point cloud
    FILE *stream;
    stream = fopen(bin_file_path.c_str(), "rb");
    num = fread(data,sizeof(float), num, stream)/4;
    point_cloud_msg->clear();
    point_cloud_msg->width = 1;
    point_cloud_msg->height = num;
    point_cloud_msg->reserve(num);
    // std::cout << num << std::endl;
    for (int32_t i=0; i<num; i++) {
        point_cloud_msg->push_back(pcl::PointXYZ(*px, *py, *pz));
        px+=4; py+=4; pz+=4;
    }

    fclose(stream);
    free(data);
}

void execute(const data_loader::load_small_datasetGoalConstPtr& goal, Server* as) {
    result.loading_finished = false;
    
    getDataPath(goal->date, goal->seq);
    int data_length = getDataLength();
    
    int i=0;
    ros::Rate lr(10);
    for (i=0; i<data_length; ++i) {
        ss.clear(); ss.str("");
        ss << std::setw(10) << std::setfill('0') << i;
        loadPointCloud(bin_file_prefix + std::string(ss.str()) + ".bin");

        pcl_conversions::toPCL(ros::Time::now(), point_cloud_msg->header.stamp);
        pub_point_cloud.publish(point_cloud_msg);
        
        feedback.loading_completion_percent = static_cast<float>(i)/static_cast<float>(data_length-1);
        as->publishFeedback(feedback);
        lr.sleep();
    }

    result.loading_finished = true;
    as->setSucceeded(result);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "load_small_dataset");

    point_cloud_msg = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    point_cloud_msg->header.frame_id = "velodyne";

    ros::NodeHandle nh;
    // pub_image = nh.advertise<>("/kitti_small/image_00" ,5);
    pub_point_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/kitti_small/point_cloud" ,5);

    Server server(nh, "load_small_dataset_action_server", boost::bind(&execute, _1, &server), false);
    server.start();


    ros::Rate loop_rate(100);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}