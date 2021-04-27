#include <lidar_odometry_mapping/lidar_odometry_mapping.h>

namespace vloam {

    void LidarOdometryMapping::init(ros::NodeHandle* nh_) {
        scan_registration.init(nh_);
        laser_odometry.init(nh_);
        laser_mapping.init(nh_);

        // subLaserCloud = nh->subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, &LidarOdometryMapping::laserCloudHandler, this);

        laserCloud = boost::make_shared<pcl::PointCloud<PointType>>();
        cornerPointsSharp = boost::make_shared<pcl::PointCloud<PointType>>();
        cornerPointsLessSharp = boost::make_shared<pcl::PointCloud<PointType>>();
        surfPointsFlat = boost::make_shared<pcl::PointCloud<PointType>>();
        surfPointsLessFlat = boost::make_shared<pcl::PointCloud<PointType>>();

        laserCloudCornerLast = boost::make_shared<pcl::PointCloud<PointType>>();
        laserCloudSurfLast = boost::make_shared<pcl::PointCloud<PointType>>();
        laserCloudFullRes = boost::make_shared<pcl::PointCloud<PointType>>();
    }

    void LidarOdometryMapping::reset() {
        // prepare: laserCloud, cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat
        scan_registration.reset();
        // laser_odometry.reset();
        laser_mapping.reset();
    }

    void LidarOdometryMapping::scanRegistrationIO(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn) {
        scan_registration.input(laserCloudIn);

        scan_registration.publish();

        scan_registration.output(
            laserCloud,  // 10Hz
            cornerPointsSharp,  // 10Hz
            cornerPointsLessSharp,  // 10Hz
            surfPointsFlat,  // 10Hz
            surfPointsLessFlat // 10Hz
        );
    }

    void LidarOdometryMapping::laserOdometryIO() {
        laser_odometry.input(
            laserCloud,  // 10Hz
            cornerPointsSharp, // 10Hz
            cornerPointsLessSharp,  // 10Hz
            surfPointsFlat,  // 10Hz
            surfPointsLessFlat // 10Hz
        );

        laser_odometry.solveLO(); // 10Hz

        laser_odometry.publish();

        laser_odometry.output(
            q_wodom_curr, // 10Hz
            t_wodom_curr, // 10Hz
            laserCloudCornerLast, // 2Hz // no change if skip_frame
            laserCloudSurfLast, // 2Hz // no change if skip_frame
            laserCloudFullRes, // 2Hz // no change if skip_frameee
            skip_frame
        );
    }

    void LidarOdometryMapping::laserMappingIO() {
        laser_mapping.input(
            laserCloudCornerLast, // 2Hz 
            laserCloudSurfLast, // 2Hz 
            laserCloudFullRes, // 2Hz 
            q_wodom_curr, // 10Hz
            t_wodom_curr, // 10Hz
            skip_frame
        );

        if (!skip_frame)
            laser_mapping.solveMapping(); // 2Hz

        laser_mapping.publish();

        // laser_odometry.output(

        // );
    }


    // void LidarOdometryMapping::solveLOAM() {
    //     laser_odometry.solveLO();
    //     laser_mapping.solveMapping();

    //     // TODO: check the running time constraint: 0.1s
    // }

}