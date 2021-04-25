#include <lidar_odometry_mapping/scan_registration.h>
#include <lidar_odometry_mapping/laser_odometry.h>
#include <lidar_odometry_mapping/laser_mapping.h>

#include <ros/ros.h>

vloam::ScanRegistration scan_registration;
vloam::LaserOdometry laser_odometry;
vloam::LaserMapping laser_mapping;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "loam_main");
    ros::NodeHandle nh;
    
    scan_registration.init(&nh);
    laser_odometry.init(&nh);
    laser_mapping.init(&nh);

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        laser_odometry.solveLO();
        laser_mapping.solveMapping();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}