#include <lidar_odometry_mapping/laser_odometry.h>

namespace vloam {

    // init before a new rosbag comes
    void LaserOdometry::init (ros::NodeHandle* nh_) {
            nh = nh_;

            corner_correspondence = 0;
            plane_correspondence = 0;

            skipFrameNum = 5;
            systemInited = false;

            timeCornerPointsSharp = 0;
            timeCornerPointsLessSharp = 0;
            timeSurfPointsFlat = 0;
            timeSurfPointsLessFlat = 0;
            timeLaserCloudFullRes = 0;

            kdtreeCornerLast = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
            kdtreeSurfLast = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();

            cornerPointsSharp = boost::make_shared<pcl::PointCloud<PointType>>();
            cornerPointsLessSharp = boost::make_shared<pcl::PointCloud<PointType>>();
            surfPointsFlat = boost::make_shared<pcl::PointCloud<PointType>>();
            surfPointsLessFlat = boost::make_shared<pcl::PointCloud<PointType>>();

            laserCloudCornerLast = boost::make_shared<pcl::PointCloud<PointType>>();
            laserCloudSurfLast = boost::make_shared<pcl::PointCloud<PointType>>();
            laserCloudFullRes = boost::make_shared<pcl::PointCloud<PointType>>();

            laserCloudCornerLastNum = 0;
            laserCloudSurfLastNum = 0;

            // Transformation from current frame to world frame
            q_w_curr = Eigen::Quaterniond(1, 0, 0, 0);
            t_w_curr = Eigen::Vector3d(0, 0, 0);

            // q_curr_last(x, y, z, w), t_curr_last
            para_q[0] = 0.0;
            para_q[1] = 0.0;
            para_q[2] = 0.0;
            para_q[3] = 1.0;
            para_t[0] = 0.0;
            para_t[1] = 0.0;
            para_t[2] = 0.0;

            // // q_last_curr = Eigen::Map<Eigen::Quaterniond>(para_q);
            // // t_last_curr = Eigen::Map<Eigen::Vector3d>(para_t);
            new (&q_last_curr) Eigen::Map<Eigen::Quaterniond>(para_q);
            new (&t_last_curr) Eigen::Map<Eigen::Vector3d>(para_t);

            // ROS_INFO("INIT raw: q_last_curr: w = %f x = %f, y = %f, z = %f", para_q[3], para_q[0], para_q[1], para_q[2]);
            // ROS_INFO("INIT: q_last_curr: w = %f x = %f, y = %f, z = %f", q_last_curr.w(), q_last_curr.x(), q_last_curr.y(), q_last_curr.z());
            // ROS_INFO("INIT: t_last_curr: x = %f, y = %f, z = %f", t_last_curr.x(), t_last_curr.y(), t_last_curr.z());

            // Eigen::Map<Eigen::Quaterniond> q_last_curr2(para_q);
            // Eigen::Map<Eigen::Vector3d> t_last_curr2(para_t);
            // ROS_INFO("INIT2: q_last_curr: w = %f x = %f, y = %f, z = %f", q_last_curr2.w(), q_last_curr2.x(), q_last_curr2.y(), q_last_curr2.z());
            // ROS_INFO("INIT2: t_last_curr: x = %f, y = %f, z = %f", t_last_curr2.x(), t_last_curr2.y(), t_last_curr2.z());

            count_receive = std::vector<int>(5, 0);

            subCornerPointsSharp = nh->subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, &LaserOdometry::laserCloudSharpHandler, this);
            subCornerPointsLessSharp = nh->subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100, &LaserOdometry::laserCloudLessSharpHandler, this);
            subSurfPointsFlat = nh->subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100, &LaserOdometry::laserCloudFlatHandler, this);
            subSurfPointsLessFlat = nh->subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, &LaserOdometry::laserCloudLessFlatHandler, this);
            subLaserCloudFullRes = nh->subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100, &LaserOdometry::laserCloudFullResHandler, this);

            pubLaserCloudCornerLast = nh->advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
            pubLaserCloudSurfLast = nh->advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
            pubLaserCloudFullRes = nh->advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100);
            pubLaserOdometry = nh->advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
            pubLaserPath = nh->advertise<nav_msgs::Path>("/laser_odom_path", 100);

            frameCount = 0;
    }

    // reset after a new message comes
    void LaserOdometry::reset() {
        count_receive = std::vector<int>(5, 0);

        // LOAM must use previous f2f odom as a prior
        // para_q[0] = 0.0;
        // para_q[1] = 0.0;
        // para_q[2] = 0.0;
        // para_q[3] = 1.0;
        // para_t[0] = 0.0;
        // para_t[1] = 0.0;
        // para_t[2] = 0.0;

        // new (&q_last_curr) Eigen::Map<Eigen::Quaterniond>(para_q);
        // new (&t_last_curr) Eigen::Map<Eigen::Vector3d>(para_t);
    }

    // undistort lidar point
    void LaserOdometry::TransformToStart(PointType const *const pi, PointType *const po)
    {
        //interpolation ratio
        double s;
        if (DISTORTION)
            s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
        else
            s = 1.0;
        //s = 1;
        Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
        Eigen::Vector3d t_point_last = s * t_last_curr;
        Eigen::Vector3d point(pi->x, pi->y, pi->z);
        Eigen::Vector3d un_point = q_point_last * point + t_point_last;

        po->x = un_point.x();
        po->y = un_point.y();
        po->z = un_point.z();
        po->intensity = pi->intensity;
    }

    // transform all lidar points to the start of the next frame
    void LaserOdometry::TransformToEnd(PointType const *const pi, PointType *const po)
    {
        // undistort point first
        pcl::PointXYZI un_point_tmp;
        TransformToStart(pi, &un_point_tmp);

        Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
        Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

        po->x = point_end.x();
        po->y = point_end.y();
        po->z = point_end.z();

        //Remove distortion time info
        po->intensity = int(pi->intensity);
    }

    void LaserOdometry::laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
    {
        cornerSharpBuf = cornerPointsSharp2;
        ++count_receive[0];
    }

    void LaserOdometry::laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
    {
        cornerLessSharpBuf = cornerPointsLessSharp2;
        ++count_receive[1];
    }

    void LaserOdometry::laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
    {
        surfFlatBuf = surfPointsFlat2;
        ++count_receive[2];
    }

    void LaserOdometry::laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
    {
        surfLessFlatBuf = surfPointsLessFlat2;
        ++count_receive[3];
    }

    //receive all point cloud
    void LaserOdometry::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
    {
        fullPointsBuf = laserCloudFullRes2;
        ++count_receive[4];
    }

    void LaserOdometry::solveLO() {
        if (count_receive[0] > 1 or count_receive[1] > 1 or count_receive[2] > 1 or count_receive[3] > 1 or count_receive[4] > 1) {
            ROS_ERROR("LO receives wrong inputs");
            ROS_BREAK();
            return;
        }

        if (count_receive[0] == 1 && count_receive[1] == 1 && count_receive[2] == 1 && count_receive[3] == 1 && count_receive[4] == 1)
        {
            this->reset();

            ROS_INFO("LO: Finished reset");

            timeCornerPointsSharp = cornerSharpBuf->header.stamp.toSec();
            timeCornerPointsLessSharp = cornerLessSharpBuf->header.stamp.toSec();
            timeSurfPointsFlat = surfFlatBuf->header.stamp.toSec();
            timeSurfPointsLessFlat = surfLessFlatBuf->header.stamp.toSec();
            timeLaserCloudFullRes = fullPointsBuf->header.stamp.toSec();

            if (timeCornerPointsSharp != timeLaserCloudFullRes ||
                timeCornerPointsLessSharp != timeLaserCloudFullRes ||
                timeSurfPointsFlat != timeLaserCloudFullRes ||
                timeSurfPointsLessFlat != timeLaserCloudFullRes)
            {
                printf("unsync messeage!");
                ROS_BREAK();
            }

            pcl::fromROSMsg(*cornerSharpBuf, *cornerPointsSharp);
            pcl::fromROSMsg(*cornerLessSharpBuf, *cornerPointsLessSharp);
            pcl::fromROSMsg(*surfFlatBuf, *surfPointsFlat);
            pcl::fromROSMsg(*surfLessFlatBuf, *surfPointsLessFlat);
            pcl::fromROSMsg(*fullPointsBuf, *laserCloudFullRes);

            TicToc t_whole;
            // initializing
            
            ROS_INFO("LO: before solving");

            if (!systemInited)
            {
                systemInited = true;
                std::cout << "Initialization finished \n";
            }
            else
            {
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();

                TicToc t_opt;
                for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
                {
                    corner_correspondence = 0;
                    plane_correspondence = 0;

                    //ceres::LossFunction *loss_function = NULL;
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
                    ceres::LocalParameterization *q_parameterization =
                        new ceres::EigenQuaternionParameterization();
                    ceres::Problem::Options problem_options;

                    ceres::Problem problem(problem_options);
                    problem.AddParameterBlock(para_q, 4, q_parameterization);
                    problem.AddParameterBlock(para_t, 3);

                    pcl::PointXYZI pointSel;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;

                    TicToc t_data;
                    // find correspondence for corner features
                    for (int i = 0; i < cornerPointsSharpNum; ++i)
                    {
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                                                        (laserCloudCornerLast->points[j].x - pointSel.x) +
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) *
                                                        (laserCloudCornerLast->points[j].y - pointSel.y) +
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) *
                                                        (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                        }
                        if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                        {
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                        cornerPointsSharp->points[i].y,
                                                        cornerPointsSharp->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                            laserCloudCornerLast->points[closestPointInd].y,
                                                            laserCloudCornerLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                            laserCloudCornerLast->points[minPointInd2].y,
                                                            laserCloudCornerLast->points[minPointInd2].z);

                            double s;
                            if (DISTORTION)
                                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                            else
                                s = 1.0;
                            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            corner_correspondence++;
                        }
                    }

                    // find correspondence for plane features
                    for (int i = 0; i < surfPointsFlatNum; ++i)
                    {
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];

                            // get closest point's scan ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

                            // search in the direction of increasing scan line
                            for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or lower scan line
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                // if in the higher scan line
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd - 1; j >= 0; --j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;

                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                                                        (laserCloudSurfLast->points[j].x - pointSel.x) +
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) *
                                                        (laserCloudSurfLast->points[j].y - pointSel.y) +
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) *
                                                        (laserCloudSurfLast->points[j].z - pointSel.z);

                                // if in the same or higher scan line
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    // find nearer point
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            if (minPointInd2 >= 0 && minPointInd3 >= 0)
                            {

                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                            surfPointsFlat->points[i].y,
                                                            surfPointsFlat->points[i].z);
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                                laserCloudSurfLast->points[closestPointInd].y,
                                                                laserCloudSurfLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                                laserCloudSurfLast->points[minPointInd2].y,
                                                                laserCloudSurfLast->points[minPointInd2].z);
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                                laserCloudSurfLast->points[minPointInd3].y,
                                                                laserCloudSurfLast->points[minPointInd3].z);

                                double s;
                                if (DISTORTION)
                                    s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                                else
                                    s = 1.0;
                                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                plane_correspondence++;
                            }
                        }
                    }

                    //printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
                    printf("data association time %f ms \n", t_data.toc());

                    if ((corner_correspondence + plane_correspondence) < 10)
                    {
                        printf("less correspondence! *************************************************\n");
                    }

                    TicToc t_solver;
                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR;
                    options.max_num_iterations = 4;
                    options.minimizer_progress_to_stdout = false;
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);
                    ROS_INFO("full report: \n%s", summary.FullReport().c_str());
                    // ROS_INFO("raw: q_last_curr: w = %f x = %f, y = %f, z = %f", para_q[3], para_q[0], para_q[1], para_q[2]);
                    printf("solver time %f ms \n", t_solver.toc());
                }
                printf("optimization twice time %f \n", t_opt.toc());

                t_w_curr = t_w_curr + q_w_curr * t_last_curr;
                q_w_curr = q_w_curr * q_last_curr;
            }

            TicToc t_pub;

            // ROS_INFO("raw: q_last_curr: w = %f x = %f, y = %f, z = %f", para_q[3], para_q[0], para_q[1], para_q[2]);
            // ROS_INFO("q_last_curr: w = %f x = %f, y = %f, z = %f", q_last_curr.w(), q_last_curr.x(), q_last_curr.y(), q_last_curr.z());
            // ROS_INFO("q_w_curr: w = %f x = %f, y = %f, z = %f", q_w_curr.w(), q_w_curr.x(), q_w_curr.y(), q_w_curr.z());

            // ROS_INFO("t_last_curr: x = %f, y = %f, z = %f", t_last_curr.x(), t_last_curr.y(), t_last_curr.z());
            // ROS_INFO("t_w_curr: x = %f, y = %f, z = %f", t_w_curr.x(), t_w_curr.y(), t_w_curr.z());

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "laser_odom";
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            pubLaserOdometry.publish(laserOdometry);

            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "map";
            pubLaserPath.publish(laserPath);

            // transform corner features and plane features to the scan end point
            if (0)
            {
                int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
                for (int i = 0; i < cornerPointsLessSharpNum; i++)
                {
                    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
                }

                int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
                for (int i = 0; i < surfPointsLessFlatNum; i++)
                {
                    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
                }

                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++)
                {
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }
            }

            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;
            laserCloudCornerLast = laserCloudTemp;

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;

            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();

            // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

            kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

            if (frameCount % skipFrameNum == 0)
            {
                frameCount = 0;

                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = "camera";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }
            printf("publication time %f ms \n", t_pub.toc());
            printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
            if(t_whole.toc() > 100)
                ROS_WARN("odometry process over 100ms");

            frameCount++;
        }
    }

}