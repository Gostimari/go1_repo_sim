#include <ros/ros.h>
#include "utility.h"
#include <nav_msgs/OccupancyGrid.h>
#include <elevation_msgs/OccupancyElevation.h>

class OccupancyElevationConverter {

private:

    ros::NodeHandle nh;
    // Transform Listener
    tf::TransformListener listener;
    tf::StampedTransform stamped_transform;

    ros::Publisher elevation_pub;
    ros::Subscriber occupancy_sub;

    elevation_msgs::OccupancyElevation elevation_msg;

    // Local Map Extraction
    PointType robotPoint;
    PointType localMapOriginPoint;

public:
    OccupancyElevationConverter()
    { 
        occupancy_sub = nh.subscribe("/elevation_map_fused_visualization/elevation_grid", 1000, &OccupancyElevationConverter::occupancyCallback, this);
        elevation_pub = nh.advertise<elevation_msgs::OccupancyElevation>("/elevation_map_fused_visualization/elevation_occupancy", 1000);
    };

    ~OccupancyElevationConverter(){};

    bool getRobotPosition()
    {

        try{listener.lookupTransform("map","base", ros::Time(0), stamped_transform); } 
        catch (tf::TransformException ex){ ROS_ERROR("Transfrom Failure."); return false; }

        robotPoint.x = stamped_transform.getOrigin().x();
        robotPoint.y = stamped_transform.getOrigin().y();
        robotPoint.z = stamped_transform.getOrigin().z();

        return true;
    }

    void initializeLocalOccupancyMap()
    {
        // initialization of customized map message
        elevation_msg.header.frame_id = "odom"; //odom
        elevation_msg.occupancy.info.width = localMapArrayLength;
        elevation_msg.occupancy.info.height = localMapArrayLength;
        elevation_msg.occupancy.info.resolution = mapResolution;

        elevation_msg.occupancy.info.origin.orientation.x = 0.0;
        elevation_msg.occupancy.info.origin.orientation.y = 0.0;
        elevation_msg.occupancy.info.origin.orientation.z = 0.0;
        elevation_msg.occupancy.info.origin.orientation.w = 1.0;

        elevation_msg.occupancy.data.resize(elevation_msg.occupancy.info.width * elevation_msg.occupancy.info.height);
        elevation_msg.height.resize(elevation_msg.occupancy.info.width * elevation_msg.occupancy.info.height);
        elevation_msg.costMap.resize(elevation_msg.occupancy.info.width * elevation_msg.occupancy.info.height);
    }

    void occupancyCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

        if(!getRobotPosition()) return;

        // local map origin x and y
        localMapOriginPoint.x = robotPoint.x - localMapLength / 2;
        localMapOriginPoint.y = robotPoint.y - localMapLength / 2;
        localMapOriginPoint.z = robotPoint.z;

        // Initialize local occupancy grid map to unknown, height to -FLT_MAX
        std::fill(elevation_msg.occupancy.data.begin(), elevation_msg.occupancy.data.end(), -1);
        std::fill(elevation_msg.height.begin(), elevation_msg.height.end(), -FLT_MAX);
        std::fill(elevation_msg.costMap.begin(), elevation_msg.costMap.end(), 0);

        elevation_msg.header.stamp = ros::Time::now();
        elevation_msg.occupancy.info = msg->info;

        elevation_msg.occupancy.info.origin.position.x = localMapOriginPoint.x;
        elevation_msg.occupancy.info.origin.position.y = localMapOriginPoint.y;
        elevation_msg.occupancy.info.origin.position.z = localMapOriginPoint.z; // add 10, just for visualization

        elevation_msg.occupancy.data = msg->data;

        elevation_pub.publish(elevation_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "occupancy_to_elevation");

    OccupancyElevationConverter occupancy_elevation_converter;

    ros::spin();
    return 0;
}

