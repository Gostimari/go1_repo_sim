#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;
    std::uint16_t reflectivity;
    std::uint8_t ring;
    std::uint16_t noise;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint32_t, t, t) (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring) (std::uint16_t, noise, noise) (std::uint32_t, range, range)
)

//using PointXYZIRT = VelodynePointXYZIRT;
using PointXYZIRT = OusterPointXYZIRT;

typedef pcl::PointXYZI PointType;

// Construct the path to scan2map.txt relative to the source directory
std::string file = "scan2map.txt";

pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
pcl::PointCloud<PointType>::Ptr readCloud;
std::deque<sensor_msgs::PointCloud2> cloudQueue;
sensor_msgs::PointCloud2 currentCloudMsg;

ros::Publisher pubCloud;
ros::Subscriber subCloud;



void initializeFile()
{
    std::ofstream outFile(file, std::ios::trunc);
    if(!outFile.is_open())
    {
        ROS_ERROR("Failed to open file: %s", file.c_str());
        return;
    } else {
        ROS_INFO("File opened: %s", file.c_str());
    }
    outFile.close();

    // Allocate memory for the point cloud
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    readCloud.reset(new pcl::PointCloud<PointType>());
}

pcl::PointCloud<PointType>::Ptr readFile(const std::string& filename)
{
    std::ifstream inFile(filename);

    if (!inFile.is_open())
    {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return readCloud;
    }

    PointType point;
    while (inFile >> point.x >> point.y >> point.z >> point.intensity)
    {
        readCloud->points.push_back(point);
    }

    inFile.close();
    return readCloud;
}

void publishCloud(const std::string& filename)
{
    pcl::PointCloud<PointType>::Ptr Cloud = readFile(filename);

    //readCloud -> clear();

    if (Cloud->points.empty())
    {
        ROS_WARN("No points to publish from file: %s", filename.c_str());
        return;
    }

    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg(*Cloud, cloudMsg);
    cloudMsg.header.stamp = ros::Time::now();
    cloudMsg.header.frame_id = "map";
    pubCloud.publish(cloudMsg);
    ROS_INFO("Point Cloud published");

    Cloud -> clear();
    readCloud -> clear();
}

void saveCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    std::ofstream outFile(file, std::ios::app);

    // cloudQueue.push_back(*laserCloudMsg);
    // currentCloudMsg = std::move(cloudQueue.front());
    // cloudQueue.pop_front();

    currentCloudMsg = *laserCloudMsg;

    pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);

    int cloudSize = laserCloudIn->points.size();

    for (int i = 0; i < cloudSize; ++i)
    {
        outFile << laserCloudIn->points[i].x << " " << laserCloudIn->points[i].y << " " << laserCloudIn->points[i].z << " " << laserCloudIn->points[i].intensity << "\n";
    }

    outFile.close();
    laserCloudIn->clear();
    //cloudQueue.clear();

    publishCloud(file);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan2map");
    ros::NodeHandle nh;

    initializeFile();

    subCloud = nh.subscribe<sensor_msgs::PointCloud2>("/current_scan", 100, saveCloud);

    pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/map_cloud", 100, true);

    //publishCloud(file);

    ros::spin();

    return 0;
}
