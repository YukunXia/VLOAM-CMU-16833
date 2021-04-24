#include <cmath>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


std::string LOFileName;
std::string VOFileName;
std::string MOFileName;
FILE *LOFilePtr = NULL;
FILE *VOFilePtr = NULL;
FILE *MOFilePtr = NULL;

std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
geometry_msgs::TransformStamped imu_stamped_tf_velo, imu_stamped_tf_cam0;
tf2::Transform imu_T_velo, imu_T_cam0, cam0_T_velo;
// tf2::Vector3 lo_xyz, vo_xyz, mo_xyz;
// tf2::Quaternion lo_q, vo_q, mo_q;

bool odomInit= false;
double laserOdomTime=0, visualOdomTime=0, mapOdomTime=0;

void getTF(){
	imu_stamped_tf_velo = tf_buffer_ptr->lookupTransform("imu_link", "velo_link", ros::Time(0));
	tf2::fromMsg(imu_stamped_tf_velo.transform, imu_T_velo);
	imu_stamped_tf_cam0 = tf_buffer_ptr->lookupTransform("imu_link", "camera_gray_left", ros::Time(0));
	tf2::fromMsg(imu_stamped_tf_cam0.transform, imu_T_cam0);
	cam0_T_velo = imu_T_cam0.inverse() * imu_T_velo;
}

tf2::Transform applyTF(const nav_msgs::Odometry::ConstPtr &odomIn, const tf2::Transform &cam_T_velo){
	tf2::Transform init_velo_T_odom, init_cam0_T_odom;
	geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  	tf2::Quaternion q(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w);

	init_velo_T_odom.setOrigin(tf2::Vector3(odomIn->pose.pose.position.x, odomIn->pose.pose.position.y, odomIn->pose.pose.position.z));
	init_velo_T_odom.setRotation(q);
	init_cam0_T_odom = cam_T_velo * init_velo_T_odom;
	return init_cam0_T_odom;

}
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry){
	laserOdomTime = laserOdometry->header.stamp.toSec();
	
	if (!odomInit){
		try{
			getTF();
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			return;
		}
		odomInit = true;
	}

	// apply tf to transform odom to cam0 center
	tf2::Transform init_cam0_T_lo = applyTF(laserOdometry, cam0_T_velo);
	// printf("\n laserodom %f \n", init_cam0_T_lo.getOrigin().x());
	fprintf(LOFilePtr, "%f %f %f %f %f %f %f %f\n", 
	init_cam0_T_lo.getOrigin().x(), init_cam0_T_lo.getOrigin().y(), init_cam0_T_lo.getOrigin().z(), 
	init_cam0_T_lo.getRotation().x(), init_cam0_T_lo.getRotation().y(), init_cam0_T_lo.getRotation().z(), init_cam0_T_lo.getRotation().w(),
	laserOdomTime);

}
void visualOdometryHandler(const nav_msgs::Odometry::ConstPtr &visualOdometry){
	visualOdomTime = visualOdometry->header.stamp.toSec();
	if (!odomInit){
		try{
			getTF();
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			return;
		}
		odomInit = true;
	}

	tf2::Transform init_cam0_T_vo = applyTF(visualOdometry, cam0_T_velo);
	fprintf(VOFilePtr, "%f %f %f %f %f %f %f %f\n", 
	init_cam0_T_vo.getOrigin().x(), init_cam0_T_vo.getOrigin().y(), init_cam0_T_vo.getOrigin().z(), 
	init_cam0_T_vo.getRotation().x(), init_cam0_T_vo.getRotation().y(), init_cam0_T_vo.getRotation().z(), init_cam0_T_vo.getRotation().w(),
	visualOdomTime);
}
void mapOdometryHandler(const nav_msgs::Odometry::ConstPtr &mapOdometry){
	mapOdomTime = mapOdometry->header.stamp.toSec();

	if (!odomInit){
		try{
			getTF();
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			return;
		}
		odomInit = true;
	}

	tf2::Transform init_cam0_T_mo = applyTF(mapOdometry, cam0_T_velo);
	fprintf(MOFilePtr, "%f %f %f %f %f %f %f %f\n", 
	init_cam0_T_mo.getOrigin().x(), init_cam0_T_mo.getOrigin().y(), init_cam0_T_mo.getOrigin().z(), 
	init_cam0_T_mo.getRotation().x(), init_cam0_T_mo.getRotation().y(), init_cam0_T_mo.getRotation().z(), init_cam0_T_mo.getRotation().w(),
	mapOdomTime);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometryEval");
	ros::NodeHandle nh;
  	ros::NodeHandle nhPrivate = ros::NodeHandle("~");
	nhPrivate.getParam("lidar_odom_filename", LOFileName);
	nhPrivate.getParam("visual_odom_filename", VOFileName);
	nhPrivate.getParam("map_odom_filename", MOFileName);
	
	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);
	ros::Subscriber subVisualOdometry = nh.subscribe<nav_msgs::Odometry>("/visual_odom_to_init", 100, visualOdometryHandler);
	ros::Subscriber subMapOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, mapOdometryHandler);


	tf_buffer_ptr = std::make_shared<tf2_ros::Buffer>();
    tf2_ros::TransformListener tf_listener(*tf_buffer_ptr);

	LOFileName += ".txt";
	VOFileName += ".txt";
	MOFileName += ".txt";
	LOFilePtr = fopen(LOFileName.c_str(), "w");
	VOFilePtr = fopen(VOFileName.c_str(), "w");
	MOFilePtr = fopen(MOFileName.c_str(), "w");
	printf("\n lo file path; %s\n", LOFileName.c_str());

	ros::spin();


	fclose(LOFilePtr);
  	fclose(VOFilePtr);
	fclose(MOFilePtr);

  	printf("\n LO, VO ,MO saved\n\n");
	return 0;
}
