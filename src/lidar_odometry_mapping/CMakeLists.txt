cmake_minimum_required(VERSION 2.8.3)
project(lidar_odometry_mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
  vloam_tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 4.5.1 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

add_library(scan_registration SHARED
  src/scan_registration.cpp
)
target_link_libraries(scan_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES} vloam_tf)
set_target_properties(scan_registration PROPERTIES PUBLIC_HEADER include/${PROJECT_NAME}/scan_registration.h)

add_library(laser_odometry SHARED
  src/laser_odometry.cpp
)
target_link_libraries(laser_odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} vloam_tf)
set_target_properties(laser_odometry PROPERTIES PUBLIC_HEADER include/${PROJECT_NAME}/laser_odometry.h)

add_library(laser_mapping SHARED
  src/laser_mapping.cpp
)
target_link_libraries(laser_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} vloam_tf)
set_target_properties(laser_mapping PROPERTIES PUBLIC_HEADER include/${PROJECT_NAME}/laser_mapping.h)

add_library(lidar_odometry_mapping SHARED
  src/lidar_odometry_mapping.cpp
)
target_link_libraries(lidar_odometry_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} scan_registration laser_odometry laser_mapping vloam_tf)
set_target_properties(lidar_odometry_mapping PROPERTIES PUBLIC_HEADER include/${PROJECT_NAME}/lidar_odometry_mapping.h)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING 
    PATTERN "*.hpp"
    PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)


