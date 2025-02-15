#include "utility.h"
#include <boost/shared_ptr.hpp>
#include <gazebo_msgs/ModelStates.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

class TraversabilityFilter {

private:
    // ROS node handler
    ros::NodeHandle nh;
    // ROS subscriber
    ros::Subscriber subCloud;
    ros::Subscriber amcl_pose;
    ros::Subscriber gazebo_pose;
    // Mutex Memory Lock
    std::mutex mtx;
    // ROS publisher
    ros::Publisher pubCloud;
    ros::Publisher pubCloudVisualHiRes;
    ros::Publisher pubCloudVisualLowRes;
    ros::Publisher pubLaserScan;
    ros::Publisher pubFullCloud;

    laser_geometry::LaserProjection projector_;

    geometry_msgs::Pose robotPose;
    // Point Cloud
    pcl::PointCloud<PointType>::Ptr laserCloudIn; // projected full velodyne cloud
    pcl::PointCloud<PointType>::Ptr laserCloudOut; // filtered and downsampled point cloud
    pcl::PointCloud<PointType>::Ptr laserCloudObstacles; // cloud for saving points that are classified as obstables, convert them to laser scan

    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;
    pcl::PointCloud<PointType>::Ptr laserCloudInOriginal;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud;

    sensor_msgs::PointCloud2 globalCloudMsg;
    sensor_msgs::PointCloud2ConstPtr globalCloudMsgPtr;

    std_msgs::Header cloudHeader;

    PointType nanPoint; // fill in fullCloud at each iteration

    // Transform Listener
    tf::TransformListener listener;
    tf::StampedTransform transform;
    // A few points
    PointType robotPoint;
    PointType localMapOrigin;
    // point cloud saved as N_SCAN * Horizon_SCAN form
    vector<vector<PointType>> laserCloudMatrix;
    // Matrice
    cv::Mat obstacleMatrix; // -1 - invalid, 0 - free, 1 - obstacle
    cv::Mat rangeMatrix; // -1 - invalid, >0 - valid range value
    // laser scan message
    sensor_msgs::LaserScan laserScan;
    // for downsample
    float** minHeight;
    float** maxHeight;
    bool** obstFlag;
    bool** initFlag;

public:
    TraversabilityFilter()
        : nh("~")
    {

        //subCloud = nh.subscribe<sensor_msgs::LaserScan> ("/lio_sam/mapping/map_global_full", 5, &TraversabilityFilter::cloudHandler, this); // /full_cloud_info
        subCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 5, &TraversabilityFilter::cloudHandler, this); // /full_cloud_info  /go1_gazebo/scan /rslidar_points  /keyframe_scan /current_scan
        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_info", 5);


        pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/filtered_pointcloud", 5);
        pubCloudVisualHiRes = nh.advertise<sensor_msgs::PointCloud2>("/filtered_pointcloud_visual_high_res", 5);
        pubCloudVisualLowRes = nh.advertise<sensor_msgs::PointCloud2>("/filtered_pointcloud_visual_low_res", 5);
        pubLaserScan = nh.advertise<sensor_msgs::LaserScan>("/pointcloud_2_laserscan", 5);

        // amcl_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &TraversabilityFilter::PoseHandler, this);
        gazebo_pose = nh.subscribe("/gazebo/model_states", 1, &TraversabilityFilter::PoseHandler, this);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();

        pointcloud2laserscanInitialization();
    }

    void allocateMemory()
    {

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudOut.reset(new pcl::PointCloud<PointType>());
        laserCloudObstacles.reset(new pcl::PointCloud<PointType>());
        transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());
        laserCloudInOriginal.reset(new pcl::PointCloud<PointType>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        obstacleMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(-1));
        rangeMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(-1));

        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        laserCloudMatrix.resize(N_SCAN);
        for (int i = 0; i < N_SCAN; ++i)
            laserCloudMatrix[i].resize(Horizon_SCAN);

        initFlag = new bool*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            initFlag[i] = new bool[filterHeightMapArrayLength];

        obstFlag = new bool*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            obstFlag[i] = new bool[filterHeightMapArrayLength];

        minHeight = new float*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            minHeight[i] = new float[filterHeightMapArrayLength];

        maxHeight = new float*[filterHeightMapArrayLength];
        for (int i = 0; i < filterHeightMapArrayLength; ++i)
            maxHeight[i] = new float[filterHeightMapArrayLength];

        resetParameters();
    }

    void resetParameters()
    {

        laserCloudIn->clear();
        laserCloudOut->clear();
        laserCloudObstacles->clear();

        laserCloudInOriginal->clear();
        transformed_cloud->clear();

        obstacleMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(-1));
        rangeMatrix = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(-1));

        for (int i = 0; i < filterHeightMapArrayLength; ++i) {
            for (int j = 0; j < filterHeightMapArrayLength; ++j) {
                initFlag[i][j] = false;
                obstFlag[i][j] = false;
            }
        }
        
        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~TraversabilityFilter() { }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        projectPointCloud(laserCloudMsg);

        extractRawCloud();

        if (transformCloud() == false)
            return;

        cloud2Matrix();

        applyFilter();

        extractFilteredCloud();

        downsampleCloud();

        predictCloudBGK();

        publishCloud();

        publishLaserScan();

        resetParameters();
    }

    void projectPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint;

        cloudHeader = laserCloudMsg->header;

        pcl::fromROSMsg(*laserCloudMsg, *laserCloudInOriginal);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudInOriginal, *laserCloudInOriginal, indices);
        
        // have "ring" channel in the cloud
        if (useCloudRing == true){
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            // if (laserCloudInRing->is_dense == false) {
            //     ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            //     ros::shutdown();
            // }  
        }

        cloudSize = laserCloudInOriginal->points.size();

        for (size_t i = 0; i < cloudSize; ++i){

            thisPoint.x = laserCloudInOriginal->points[i].x;
            thisPoint.y = laserCloudInOriginal->points[i].y;
            thisPoint.z = laserCloudInOriginal->points[i].z;
            // find the row and column index in the iamge for this point
            if (useCloudRing == true){
                rowIdn = laserCloudInRing->points[i].ring;
            }
            else{
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            }
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange)
                continue;
            
            //rangeMatrix.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
        }

        tf::StampedTransform transform;
        
        try {listener.lookupTransform("base", "rslidar", ros::Time(0), transform);} 
        catch (tf::TransformException &ex) {/*ROS_ERROR("%s", ex.what());*/return;}

        Eigen::Affine3d eigen_transform;
        tf::transformTFToEigen(transform, eigen_transform);

        //Eigen::Affine3d rotation_180 = Eigen::Affine3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

        //Eigen::Affine3d combined_transform = eigen_transform * rotation_180;

        pcl::transformPointCloud(*fullInfoCloud, *transformed_cloud, eigen_transform);

        sensor_msgs::PointCloud2 globalCloudMsg;
        pcl::toROSMsg(*transformed_cloud, globalCloudMsg);
        globalCloudMsg.header.frame_id = "base";
        globalCloudMsg.header.stamp = cloudHeader.stamp; //ros::Time::now();
        pubFullCloud.publish(globalCloudMsg);
    }


    void extractRawCloud()
    {
        pcl::copyPointCloud(*transformed_cloud, *laserCloudIn);
        // ROS msg -> PCL cloud
        //pcl::fromROSMsg(*fullInfoCloud, *laserCloudIn);
        // extract range info
        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                int index = j  + i * Horizon_SCAN;
                // skip NaN point
                if (laserCloudIn->points[index].intensity == std::numeric_limits<float>::quiet_NaN()) continue;
                // save range info
                rangeMatrix.at<float>(i, j) = laserCloudIn->points[index].intensity;
                // reset obstacle status to 0 - free 
                obstacleMatrix.at<int>(i, j) = 0;
            }
        }

        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr processedCloud(new pcl::PointCloud<pcl::PointXYZI>); // New point cloud for processed points

        // pcl::fromROSMsg(*laserCloudMsg, *cloudXYZI);

        // for (const auto& point : cloudXYZI->points) {
        //     pcl::PointXYZI pt;
        //     pt.x = point.x;
        //     pt.y = point.y;
        //     pt.z = point.z;

        //     float distance = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        //     pt.intensity = 1.0 / (distance * distance);

        //     processedCloud->points.push_back(pt);
        // }

        // // Copy points
        // for (const auto& point : processedCloud->points) {
        //     PointType pt;
        //     pt.x = point.x;
        //     pt.y = point.y;
        //     pt.z = point.z;
        //     pt.intensity = point.intensity;

        //     laserCloudIn->points.push_back(pt);
        // }

        // // extract range info
        // for (int i = 0; i < N_SCAN; ++i) {
        //     for (int j = 0; j < Horizon_SCAN; ++j) {
        //         int index = j + i * Horizon_SCAN;

        //         // Check if index is within bounds
        //         if (index >= laserCloudIn->points.size()) {
        //             // std::cerr << "Index out of bounds: " << index << std::endl;
        //             continue;
        //         }
        //         // skip NaN point
        //         if (std::isnan(laserCloudIn->points[index].intensity))
        //             continue;
        //         // save range info
        //         rangeMatrix.at<float>(i, j) = laserCloudIn->points[index].intensity;
        //     }
        // }
    }

    /*
        void cloudHandler(const sensor_msgs::LaserScan::ConstPtr& laserCloudMsg){


            extractRawCloud(laserCloudMsg);

            if (transformCloud() == false) return;

            cloud2Matrix();

            applyFilter();

            extractFilteredCloud();

            downsampleCloud();

            predictCloudBGK();

            publishCloud();

            publishLaserScan();

            resetParameters();
        }
        */
    /*
     void PoseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
         robotPoint.x = msg->pose.pose.position.x;
         robotPoint.y = msg->pose.pose.position.y;
         robotPoint.z = msg->pose.pose.position.z;
     }*/
    /*
    void PoseHandler(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        robotPoint.x=msg->pose[0].position.x;
        robotPoint.y=msg->pose[0].position.y;
        robotPoint.z=msg->pose[0].position.z;
    }*/

    void PoseHandler(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        robotPose = msg->pose[0];
    }
    /*
        void extractRawCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
            // ROS msg -> PCL cloud
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
            // extract range info
            for (int i = 0; i < N_SCAN; ++i){
                for (int j = 0; j < Horizon_SCAN; ++j){
                    int index = j  + i * Horizon_SCAN;
                    // skip NaN point
                    if (laserCloudIn->points[index].intensity == std::numeric_limits<float>::quiet_NaN()) continue;
                    ROS_INFO("laserCloudIn->points[index].intensity: %f", laserCloudIn->points[index].intensity);
                    // save range info
                    rangeMatrix.at<float>(i, j) = laserCloudIn->points[index].intensity;
                    // reset obstacle status to 0 - free
                    obstacleMatrix.at<int>(i, j) = 0;
                }
            }
        }
    */
    /*
        void extractRawCloud(const sensor_msgs::LaserScan::ConstPtr& laserCloudMsg){

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI(new pcl::PointCloud<pcl::PointXYZI>);

            for(int i = 0; i < laserCloudMsg->ranges.size(); i++){
                pcl::PointXYZI point;
                point.x = laserCloudMsg->ranges[i] * cos(laserCloudMsg->angle_min + i * laserCloudMsg->angle_increment);
                point.y = laserCloudMsg->ranges[i] * sin(laserCloudMsg->angle_min + i * laserCloudMsg->angle_increment);
                point.z = 0; // assuming the LiDAR is horizontally mounted

                float distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
                point.intensity = 1.0 / (distance * distance);

                cloudXYZI->points.push_back(point);
            }

            // Copy points
            for (const auto& point : cloudXYZI->points) {
                PointType pt;
                pt.x = point.x;
                pt.y = point.y;
                pt.z = point.z;
                pt.intensity = point.intensity;

                laserCloudIn->points.push_back(pt);
            }

            // extract range info
            for (int i = 0; i < N_SCAN; ++i){
                for (int j = 0; j < Horizon_SCAN; ++j){
                    int index = j  + i * Horizon_SCAN;
                    // skip NaN point
                    if (std::isnan(laserCloudIn->points[index].intensity)) continue;
                    // save range info
                    rangeMatrix.at<float>(i, j) = laserCloudIn->points[index].intensity;
                    // reset obstacle status to 0 - free
                    obstacleMatrix.at<int>(i, j) = 0;
                }
            }
        }*/

    bool transformCloud()
    {
        // ros::Duration(0.5).sleep();
        //  Listen to the TF transform and prepare for point cloud transformation

        // try {
        //     listener.lookupTransform("map", "base", ros::Time(0), transform); // odom -> base
        // } catch (tf::TransformException& ex) {
        //     ROS_ERROR("Transform Error! Filter -> TransformCLoud() %s", ex.what());
        //     ros::Duration(0.5).sleep();
        // }

        try{listener.lookupTransform("map","base", ros::Time(0), transform); }
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return false; }

        // std::cout << "Accessing transform..." << std::endl;
        robotPoint.x = transform.getOrigin().x();
        robotPoint.y = transform.getOrigin().y();
        robotPoint.z = transform.getOrigin().z();

        // robotPoint.x = amcl_pose.x;
        // robotPoint.y = amcl_pose.y;
        // robotPoint.z = amcl_pose.z;

        // robotPoint.x = robotPose.position.x;
        // robotPoint.y = robotPose.position.y;
        // robotPoint.z = robotPose.position.z;

        laserCloudIn->header.frame_id = "base";
        laserCloudIn->header.stamp = 0; // don't use the latest time, we don't have that transform in the queue yet

        pcl::PointCloud<PointType> laserCloudTemp;
        // std::cout << "Transforming point cloud..." << std::endl;
        pcl_ros::transformPointCloud("map", *laserCloudIn, laserCloudTemp, listener);
        *laserCloudIn = laserCloudTemp;

        return true;
    }

    void cloud2Matrix(){

        for (int i = 0; i < N_SCAN; ++i){
            for (int j = 0; j < Horizon_SCAN; ++j){
                int index = j  + i * Horizon_SCAN;
                PointType p = laserCloudIn->points[index];
                laserCloudMatrix[i][j] = p;
            }
        }
    }

    void applyFilter()
    {

        if (urbanMapping == true) {
            positiveCurbFilter();
            negativeCurbFilter();
        }

        slopeFilter();
    }

    void positiveCurbFilter()
    {
        int rangeCompareNeighborNum = 3;
        float diff[Horizon_SCAN - 1];

        for (int i = 0; i < scanNumCurbFilter; ++i) {
            // calculate range difference
            for (int j = 0; j < Horizon_SCAN - 1; ++j)
                diff[j] = rangeMatrix.at<float>(i, j) - rangeMatrix.at<float>(i, j + 1);

            for (int j = rangeCompareNeighborNum; j < Horizon_SCAN - rangeCompareNeighborNum; ++j) {

                // Point that has been verified by other filters
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;

                bool breakFlag = false;
                // point is too far away, skip comparison since it can be inaccurate
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                // make sure all points have valid range info
                for (int k = -rangeCompareNeighborNum; k <= rangeCompareNeighborNum; ++k)
                    if (rangeMatrix.at<float>(i, j + k) == -1) {
                        breakFlag = true;
                        break;
                    }
                if (breakFlag == true)
                    continue;
                // range difference should be monotonically increasing or decresing
                for (int k = -rangeCompareNeighborNum; k < rangeCompareNeighborNum - 1; ++k)
                    if (diff[j + k] * diff[j + k + 1] <= 0) {
                        breakFlag = true;
                        break;
                    }
                if (breakFlag == true)
                    continue;
                // the range difference between the start and end point of neighbor points is smaller than a threashold, then continue
                if (abs(rangeMatrix.at<float>(i, j - rangeCompareNeighborNum) - rangeMatrix.at<float>(i, j + rangeCompareNeighborNum)) / rangeMatrix.at<float>(i, j) < 0.03)
                    continue;
                // if "continue" is not used at this point, it is very likely to be an obstacle point
                obstacleMatrix.at<int>(i, j) = 1;
            }
        }
    }

    void negativeCurbFilter()
    {
        int rangeCompareNeighborNum = 3;

        for (int i = 0; i < scanNumCurbFilter; ++i) {
            for (int j = 0; j < Horizon_SCAN; ++j) {
                // Point that has been verified by other filters
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;
                // point without range value cannot be verified
                if (rangeMatrix.at<float>(i, j) == -1)
                    continue;
                // point is too far away, skip comparison since it can be inaccurate
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                // check neighbors
                for (int m = -rangeCompareNeighborNum; m <= rangeCompareNeighborNum; ++m) {
                    int k = j + m;
                    if (k < 0 || k >= Horizon_SCAN)
                        continue;
                    if (rangeMatrix.at<float>(i, k) == -1)
                        continue;
                    // height diff greater than threashold, might be a negative curb
                    if (laserCloudMatrix[i][j].z - laserCloudMatrix[i][k].z > 0.1
                        && pointDistance(laserCloudMatrix[i][j], laserCloudMatrix[i][k]) <= 1.0) {
                        obstacleMatrix.at<int>(i, j) = 1;
                        break;
                    }
                }
            }
        }
    }

    void slopeFilter()
    {

        for (int i = 0; i < scanNumSlopeFilter; ++i) {
            for (int j = 0; j < Horizon_SCAN; ++j) {
                // Point that has been verified by other filters
                if (obstacleMatrix.at<int>(i, j) == 1)
                    continue;
                // point without range value cannot be verified
                if (rangeMatrix.at<float>(i, j) == -1 || rangeMatrix.at<float>(i + 1, j) == -1)
                    continue;
                // point is too far away, skip comparison since it can be inaccurate
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit)
                    continue;
                // Two range filters here:
                // 1. if a point's range is larger than scanNumSlopeFilter th a point's range
                // 2. if a point's range is larger than the upper point's range
                // then this point is very likely on obstacle. i.e. a point under the car or on a pole
                // if (  (rangeMatrix.at<float>(scanNumSlopeFilter, j) != -1 && rangeMatrix.at<float>(i, j) > rangeMatrix.at<float>(scanNumSlopeFilter, j))
                //     || (rangeMatrix.at<float>(i, j) > rangeMatrix.at<float>(i+1, j)) ){
                //     obstacleMatrix.at<int>(i, j) = 1;
                //     continue;
                // }
                // Calculate slope angle
                float diffX = laserCloudMatrix[i + 1][j].x - laserCloudMatrix[i][j].x;
                float diffY = laserCloudMatrix[i + 1][j].y - laserCloudMatrix[i][j].y;
                float diffZ = laserCloudMatrix[i + 1][j].z - laserCloudMatrix[i][j].z;
                float angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;
                // Slope angle is larger than threashold, mark as obstacle point
                if (angle < -filterAngleLimit || angle > filterAngleLimit) {
                    obstacleMatrix.at<int>(i, j) = 1;
                    continue;
                }
            }
        }
    }

    void extractFilteredCloud()
    {
        for (int i = 0; i < scanNumMax; ++i) {
            for (int j = 0; j < Horizon_SCAN; ++j) {
                // invalid points and points too far are skipped
                if (rangeMatrix.at<float>(i, j) > sensorRangeLimit || rangeMatrix.at<float>(i, j) == -1)
                    continue;
                // update point intensity (occupancy) into
                PointType p = laserCloudMatrix[i][j];
                p.intensity = obstacleMatrix.at<int>(i, j) == 1 ? 100 : 0;
                // save updated points
                laserCloudOut->push_back(p);
                // extract obstacle points and convert them to laser scan
                if (p.intensity == 100)
                    laserCloudObstacles->push_back(p);
            }
        }

        // Publish laserCloudOut for visualization (before downsample and BGK prediction)
        if (pubCloudVisualHiRes.getNumSubscribers() != 0) {
            sensor_msgs::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
            laserCloudTemp.header.stamp = ros::Time::now();
            laserCloudTemp.header.frame_id = "map";
            pubCloudVisualHiRes.publish(laserCloudTemp);
        }
    }

    void downsampleCloud()
    {

        float roundedX = float(int(robotPoint.x * 10.0f)) / 10.0f;
        float roundedY = float(int(robotPoint.y * 10.0f)) / 10.0f;
        // height map origin
        localMapOrigin.x = roundedX - sensorRangeLimit;
        localMapOrigin.y = roundedY - sensorRangeLimit;

        // convert from point cloud to height map
        int cloudSize = laserCloudOut->points.size();
        for (int i = 0; i < cloudSize; ++i) {

            int idx = (laserCloudOut->points[i].x - localMapOrigin.x) / mapResolution;
            int idy = (laserCloudOut->points[i].y - localMapOrigin.y) / mapResolution;
            // points out of boundry
            if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                continue;
            // obstacle point (decided by curb or slope filter)
            if (laserCloudOut->points[i].intensity == 100)
                obstFlag[idx][idy] = true;
            // save min and max height of a grid
            if (initFlag[idx][idy] == false) {
                minHeight[idx][idy] = laserCloudOut->points[i].z;
                maxHeight[idx][idy] = laserCloudOut->points[i].z;
                initFlag[idx][idy] = true;
            } else {
                minHeight[idx][idy] = std::min(minHeight[idx][idy], laserCloudOut->points[i].z);
                maxHeight[idx][idy] = std::max(maxHeight[idx][idy], laserCloudOut->points[i].z);
            }
        }
        // intermediate cloud
        pcl::PointCloud<PointType>::Ptr laserCloudTemp(new pcl::PointCloud<PointType>());
        // convert from height map to point cloud
        for (int i = 0; i < filterHeightMapArrayLength; ++i) {
            for (int j = 0; j < filterHeightMapArrayLength; ++j) {
                // no point at this grid
                if (initFlag[i][j] == false)
                    continue;
                // convert grid to point
                PointType thisPoint;
                thisPoint.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
                thisPoint.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
                thisPoint.z = maxHeight[i][j];

                if (obstFlag[i][j] == true || maxHeight[i][j] - minHeight[i][j] > filterHeightLimit) {
                    obstFlag[i][j] = true;
                    thisPoint.intensity = 100; // obstacle
                    laserCloudTemp->push_back(thisPoint);
                } else {
                    thisPoint.intensity = 0; // free
                    laserCloudTemp->push_back(thisPoint);
                }
            }
        }

        *laserCloudOut = *laserCloudTemp;

        // Publish laserCloudOut for visualization (after downsample but beforeBGK prediction)
        if (pubCloudVisualLowRes.getNumSubscribers() != 0) {
            sensor_msgs::PointCloud2 laserCloudTemp;
            pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
            laserCloudTemp.header.stamp = ros::Time::now();
            laserCloudTemp.header.frame_id = "map";
            pubCloudVisualLowRes.publish(laserCloudTemp);
        }
    }

    void predictCloudBGK()
    {

        if (predictionEnableFlag == false)
            return;

        int kernelGridLength = int(predictionKernalSize / mapResolution);

        for (int i = 0; i < filterHeightMapArrayLength; ++i) {
            for (int j = 0; j < filterHeightMapArrayLength; ++j) {
                // skip observed point
                if (initFlag[i][j] == true)
                    continue;
                PointType testPoint;
                testPoint.x = localMapOrigin.x + i * mapResolution + mapResolution / 2.0;
                testPoint.y = localMapOrigin.y + j * mapResolution + mapResolution / 2.0;
                testPoint.z = robotPoint.z; // this value is not used except for computing distance with robotPoint
                // skip grids too far
                if (pointDistance(testPoint, robotPoint) > sensorRangeLimit)
                    continue;
                // Training data
                vector<float> xTrainVec; // training data x and y coordinates
                vector<float> yTrainVecElev; // training data elevation
                vector<float> yTrainVecOccu; // training data occupancy
                // Fill trainig data (vector)
                for (int m = -kernelGridLength; m <= kernelGridLength; ++m) {
                    for (int n = -kernelGridLength; n <= kernelGridLength; ++n) {
                        // skip grids too far
                        if (std::sqrt(float(m * m + n * n)) * mapResolution > predictionKernalSize)
                            continue;
                        int idx = i + m;
                        int idy = j + n;
                        // index out of boundry
                        if (idx < 0 || idy < 0 || idx >= filterHeightMapArrayLength || idy >= filterHeightMapArrayLength)
                            continue;
                        // save only observed grid in this scan
                        if (initFlag[idx][idy] == true) {
                            xTrainVec.push_back(localMapOrigin.x + idx * mapResolution + mapResolution / 2.0);
                            xTrainVec.push_back(localMapOrigin.y + idy * mapResolution + mapResolution / 2.0);
                            yTrainVecElev.push_back(maxHeight[idx][idy]);
                            yTrainVecOccu.push_back(obstFlag[idx][idy] == true ? 1 : 0);
                        }
                    }
                }
                // no training data available, continue
                if (xTrainVec.size() == 0)
                    continue;
                // convert from vector to eigen
                Eigen::MatrixXf xTrain = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTrainVec.data(), xTrainVec.size() / 2, 2);
                Eigen::MatrixXf yTrainElev = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecElev.data(), yTrainVecElev.size(), 1);
                Eigen::MatrixXf yTrainOccu = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(yTrainVecOccu.data(), yTrainVecOccu.size(), 1);
                // Test data (current grid)
                vector<float> xTestVec;
                xTestVec.push_back(testPoint.x);
                xTestVec.push_back(testPoint.y);
                Eigen::MatrixXf xTest = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(xTestVec.data(), xTestVec.size() / 2, 2);
                // Predict
                Eigen::MatrixXf Ks; // covariance matrix
                covSparse(xTest, xTrain, Ks); // sparse kernel

                Eigen::MatrixXf ybarElev = (Ks * yTrainElev).array();
                Eigen::MatrixXf ybarOccu = (Ks * yTrainOccu).array();
                Eigen::MatrixXf kbar = Ks.rowwise().sum().array();

                // Update Elevation with Prediction
                if (std::isnan(ybarElev(0, 0)) || std::isnan(ybarOccu(0, 0)) || std::isnan(kbar(0, 0)))
                    continue;

                if (kbar(0, 0) == 0)
                    continue;

                float elevation = ybarElev(0, 0) / kbar(0, 0);
                float occupancy = ybarOccu(0, 0) / kbar(0, 0);

                PointType p;
                p.x = xTestVec[0];
                p.y = xTestVec[1];
                p.z = elevation;
                p.intensity = (occupancy > 0.5) ? 100 : 0;

                laserCloudOut->push_back(p);
            }
        }
    }

    void dist(const Eigen::MatrixXf& xStar, const Eigen::MatrixXf& xTrain, Eigen::MatrixXf& d) const
    {
        d = Eigen::MatrixXf::Zero(xStar.rows(), xTrain.rows());
        for (int i = 0; i < xStar.rows(); ++i) {
            d.row(i) = (xTrain.rowwise() - xStar.row(i)).rowwise().norm();
        }
    }

    void covSparse(const Eigen::MatrixXf& xStar, const Eigen::MatrixXf& xTrain, Eigen::MatrixXf& Kxz) const
    {
        dist(xStar / (predictionKernalSize + 0.1), xTrain / (predictionKernalSize + 0.1), Kxz);
        Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) + (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * 1.0f;
        // Clean up for values with distance outside length scale, possible because Kxz <= 0 when dist >= predictionKernalSize
        for (int i = 0; i < Kxz.rows(); ++i)
            for (int j = 0; j < Kxz.cols(); ++j)
                if (Kxz(i, j) < 0)
                    Kxz(i, j) = 0;
    }

    void publishCloud()
    {
        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudOut, laserCloudTemp);
        laserCloudTemp.header.stamp = ros::Time::now();
        laserCloudTemp.header.frame_id = "map";
        pubCloud.publish(laserCloudTemp);
    }

    void publishLaserScan()
    {

        updateLaserScan();

        laserScan.header.stamp = ros::Time::now();
        pubLaserScan.publish(laserScan);
        // initialize laser scan for new scan
        std::fill(laserScan.ranges.begin(), laserScan.ranges.end(), laserScan.range_max + 1.0);
    }

    void updateLaserScan()
    {
        // ros::Duration(0.5).sleep();
        // try {
        //     listener.lookupTransform("base", "map", ros::Time(0), transform); // odom -> base
        // } catch (tf::TransformException& ex) {
        //     ROS_ERROR("Transform Error! Filter -> updateLaserScan() %s", ex.what());
        //     ros::Duration(0.5).sleep();
        // }

        try{listener.lookupTransform("base","map", ros::Time(0), transform);}
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return; }

        laserCloudObstacles->header.frame_id = "map"; // map
        laserCloudObstacles->header.stamp = 0;
        // transform obstacle cloud back to "base" frame
        pcl::PointCloud<PointType> laserCloudTemp;
        pcl_ros::transformPointCloud("base", *laserCloudObstacles, laserCloudTemp, listener);
        // convert point to scan
        int cloudSize = laserCloudTemp.points.size();
        for (int i = 0; i < cloudSize; ++i) {
            PointType* point = &laserCloudTemp.points[i];
            float x = point->x;
            float y = point->y;
            float range = std::sqrt(x * x + y * y);
            float angle = std::atan2(y, x);
            int index = (angle - laserScan.angle_min) / laserScan.angle_increment;
            if (index >= 0 && index < laserScan.ranges.size())
                laserScan.ranges[index] = std::min(laserScan.ranges[index], range);
        }
    }

    void pointcloud2laserscanInitialization()
    {

        laserScan.header.frame_id = "base"; // assume laser has the same frame as the robot

        laserScan.angle_min = -M_PI;
        laserScan.angle_max = M_PI;
        laserScan.angle_increment = 1.0f / 180 * M_PI;
        laserScan.time_increment = 0;

        laserScan.scan_time = 0.1;
        laserScan.range_min = 0.3;
        laserScan.range_max = 100;

        int range_size = std::ceil((laserScan.angle_max - laserScan.angle_min) / laserScan.angle_increment);
        laserScan.ranges.assign(range_size, laserScan.range_max + 1.0);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "traversability_mapping");

    TraversabilityFilter TFilter;

    ROS_INFO("\033[1;32m---->\033[0m Traversability Filter Started.");

    ros::spin();

    return 0;
}
