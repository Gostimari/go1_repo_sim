#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>


std::vector <std::pair<double, double>> waypointVect;
std::vector<std::pair < double, double> > ::iterator iter; //init. iterator
geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;
int count = 0, waypointCount = 0, wait_count = 0;
double numWaypoints = 0;
double latiGoal, longiGoal, latiNext, longiNext;
std::string utm_zone;
std::string path_local, path_abs;

bool goal_reached = false;
bool initial_gps = true;
bool waypoint_start = false;
double x,y, goal_tolerance;
double latitude, longitude, altitude;


int countWaypointsInFile(std::string path_local)
{
    path_abs = ros::package::getPath("gps_waypoint_nav") + path_local;
    std::ifstream fileCount(path_abs.c_str());
    if(fileCount.is_open())
    {
        double lati = 0;
        while(!fileCount.eof())
        {
            fileCount >> lati;
            ++count;
        }
        count = count - 1;
        numWaypoints = count / 2;
        ROS_INFO("%.0f GPS waypoints were read", numWaypoints);
        fileCount.close();
    }
    else
    {
        std::cout << "Unable to open waypoint file" << std::endl;
        ROS_ERROR("Unable to open waypoint file");
    }
    return numWaypoints;
}

std::vector <std::pair<double, double>> getWaypoints(std::string path_local)
{
    double lati = 0, longi = 0;

    path_abs = ros::package::getPath("gps_waypoint_nav") + path_local;
    std::ifstream fileRead(path_abs.c_str());
    for(int i = 0; i < numWaypoints; i++)
    {
        fileRead >> longi;
        fileRead >> lati;
        waypointVect.push_back(std::make_pair(lati, longi));
    }
    fileRead.close();

    //Outputting vector
    ROS_INFO("The following GPS Waypoints have been set:");
    for(std::vector < std::pair < double, double >> ::iterator iterDisp = waypointVect.begin(); iterDisp != waypointVect.end();
    iterDisp++)
    {
        ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
    }
    return waypointVect;
}

geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
{
    double utm_x = 0, utm_y = 0;
    geometry_msgs::PointStamped UTM_point_output;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    //Construct UTM_point and map_point geometry messages
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
    UTM_point_output.point.x = utm_x;
    UTM_point_output.point.y = utm_y;
    UTM_point_output.point.z = 0;

    return UTM_point_output;
}

geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform("map", "utm", time_now, ros::Duration(3.0));
            listener.transformPoint("map", UTM_input, map_point_output);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            //ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return map_point_output;
}

move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped map_point, geometry_msgs::PointStamped map_next, bool last_point)
{
    move_base_msgs::MoveBaseGoal goal;

    //Specify what frame we want the goal to be published in
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    // Specify x and y goal
    goal.target_pose.pose.position.x = map_point.point.x; //specify x goal
    goal.target_pose.pose.position.y = map_point.point.y; //specify y goal

    // Specify heading goal using current goal and next goal (point robot towards its next goal once it has achieved its current goal)
    if(last_point == false)
    {
        tf::Matrix3x3 rot_euler;
        tf::Quaternion rot_quat;

        // Calculate quaternion
        float x_curr = map_point.point.x, y_curr = map_point.point.y; // set current coords.
        float x_next = map_next.point.x, y_next = map_next.point.y; // set coords. of next waypoint
        float delta_x = x_next - x_curr, delta_y = y_next - y_curr;   // change in coords.
        float yaw_curr = 0, pitch_curr = 0, roll_curr = 0;
        yaw_curr = atan2(delta_y, delta_x);

        // Specify quaternions
        rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
        rot_euler.getRotation(rot_quat);

        goal.target_pose.pose.orientation.x = rot_quat.getX();
        goal.target_pose.pose.orientation.y = rot_quat.getY();
        goal.target_pose.pose.orientation.z = rot_quat.getZ();
        goal.target_pose.pose.orientation.w = rot_quat.getW();
    }
    else
    {
        goal.target_pose.pose.orientation.w = 1.0;
    }

    return goal;
}

void gps(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    if(initial_gps)
    {
        latitude = gps_msg->latitude;
        longitude = gps_msg->longitude;
        altitude = gps_msg->altitude;
        initial_gps = false;
    }
}

void odometry_CB(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    x = odom_msg->pose.pose.position.x;
    y = odom_msg->pose.pose.position.y;
    waypoint_start = true;
}

void waitToReachGoal(double map_x, double map_y, double goal_tolerance);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_waypoint"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    ROS_INFO("Initiated gps_waypoint node");
    //construct an action client that we use to communication with the action named move_base.
    //Setting true is telling the constructor to start ros::spin()

    // Initiate publisher to send end of node message
    ros::Publisher pubWaypointNodeEnded = n.advertise<std_msgs::Bool>("/gps_waypoint_nav/waypoint_following_status", 100);

    ros::Publisher move_base_simple_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    ros::Publisher vizualize_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/gps_waypoint_nav/vizualize_goal", 1000);

    ros::Subscriber sub_odom = n.subscribe("/gps_waypoint_nav/odometry/gps", 1000, odometry_CB); // change to the navsat odometry

    ros::Subscriber sub_gps = n.subscribe("/reach/fix", 1000, gps);

    //Count number of waypoints
    ros::param::get("/gps_waypoint_nav/coordinates_file", path_local);
    numWaypoints = countWaypointsInFile(path_local);

    //Reading waypoints from text file and output results
    waypointVect = getWaypoints(path_local);
    //ROS_INFO("waypointVect: %d", waypointVect.size());

    goal_tolerance = 7.0; //set goal tolerance

    int counter = 0;

    while(ros::ok())
    {
        if (waypoint_start){

            if(counter == waypointVect.size())
            {
                break;
            }

            // Iterate through vector of waypoints for setting goals
            for(iter = waypointVect.begin(); iter < waypointVect.end(); iter++)
            {
                //Setting goal:
                latiGoal = iter->first;
                longiGoal = iter->second;
                bool final_point = false;

                //set next goal point if not at last waypoint
                if(iter < (waypointVect.end() - 1))
                {
                    iter++;
                    latiNext = iter->first;
                    longiNext = iter->second;
                    iter--;
                }
                else //set to current
                {
                    latiNext = iter->first;
                    longiNext = iter->second;
                    final_point = true;
                }

                ROS_INFO("Received Latitude goal:%.8f", latiGoal);
                ROS_INFO("Received longitude goal:%.8f", longiGoal);

                //Convert lat/long to utm:
                UTM_point = latLongtoUTM(latiGoal, longiGoal);
                UTM_next = latLongtoUTM(latiNext, longiNext);

                //Transform UTM to map point in odom frame
                map_point = UTMtoMapPoint(UTM_point);
                map_next = UTMtoMapPoint(UTM_next);

                //Build goal to send to move_base
                //move_base_msgs::MoveBaseGoal goal = buildGoal(map_point, map_next, final_point); //initiate a move_base_msg called goal

                // Send Goal
                ROS_INFO("Sending goal: x: %.2f, y: %.2f", map_point.point.x, map_point.point.y);
                //ac.sendGoal(goal); //push goal to move_base node
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header = map_point.header;
                pose_msg.pose.position.x = map_point.point.x;
                pose_msg.pose.position.y = map_point.point.y;
                pose_msg.pose.position.z = 0.0;
                pose_msg.pose.orientation.x = 0.0;
                pose_msg.pose.orientation.y = 0.0;
                pose_msg.pose.orientation.z = 0.0;
                pose_msg.pose.orientation.w = 1.0;
                vizualize_goal_pub.publish(pose_msg);

                pose_msg.header = map_point.header;
                pose_msg.pose.position.x = -map_point.point.x;
                pose_msg.pose.position.y = map_point.point.y;
                pose_msg.pose.position.z = 0.0;
                pose_msg.pose.orientation.x = 0.0;
                pose_msg.pose.orientation.y = 0.0;
                pose_msg.pose.orientation.z = 0.0;
                pose_msg.pose.orientation.w = 1.0;
                move_base_simple_pub.publish(pose_msg);
                //move_base_simple_pub.publish(map_point);

                //Wait for result
                //ac.waitForResult(); //waiting to see if move_base was able to reach goal
                waitToReachGoal(map_point.point.x, map_point.point.y, goal_tolerance);

                if(goal_reached)
                {
                    if(final_point)
                    {
                        ROS_INFO("GO1 has reached the final Goal!");
                    }else
                    {
                        ROS_INFO("GO1 has reached its goal!");
                    }
                }
                else
                {
                    ROS_ERROR("Go1 was unable to reach its goal. GPS Waypoint unreachable.");
                    ROS_ERROR("Sending the next goal");
                }
                counter++;
            } // End for loop iterating through waypoint vector
        }
        else{

        }
        ros::spinOnce();
    }
    // Notify joy_launch_control that waypoint following is complete
    std_msgs::Bool node_ended;
    node_ended.data = true;
    pubWaypointNodeEnded.publish(node_ended);
    ROS_INFO("Ending node...");
    return 0;
}

void waitToReachGoal(double map_x, double map_y, double goal_tolerance)
{
    ros::Time time_last = ros::Time::now();
    ros::Time time_last_distance_check = ros::Time::now();
    double last_distance_to_goal = sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y)), current_distance_to_goal;
    bool is_distance_changing = true;

    ROS_INFO("Waiting for robot to approach goal...");
    ROS_INFO("Goal Tolerance: %.1f m", goal_tolerance);
    while(sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y)) > goal_tolerance)
    {
        current_distance_to_goal = sqrt((map_x - x) * (map_x - x) + (map_y - y) * (map_y - y));

        if((ros::Time::now() - time_last) > ros::Duration(1))
        {
            ROS_INFO("Distance to Goal: %.2f", current_distance_to_goal);
            time_last = ros::Time::now();
        }
        if((ros::Time::now() - time_last_distance_check) > ros::Duration(7))
        {
            // check that it has moved enough
            if(abs(current_distance_to_goal - last_distance_to_goal) < 0.1)
            {
                is_distance_changing = false;
            }
            time_last_distance_check = ros::Time::now();
            last_distance_to_goal = current_distance_to_goal;
        }
        ros::spinOnce();
    }
    if(is_distance_changing == false)
    {
        ROS_WARN("Distance to goal not changing, switching to next goal");
        goal_reached = false;
    }
    else
    {
        //ROS_INFO("Goal tolerance reached");
        goal_reached = true;
        return;
    }
}
