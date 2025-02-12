#include <ros/ros.h>
#include <ros/package.h>
#include <utility>
#include <fstream>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <math.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>



bool collect_request = true;
bool continue_collection = true;

bool gps_received = false;

double lati_point=0, longi_point=0, lati_last=0, longi_last=0;

double ori_point_x=0, ori_point_y=0;

int activation_key = 0;
int points_counter = 1;

double min_coord_change = 10 * pow(10,-6);
std::string end_button_sym, collect_button_sym;
int end_button_num = 0, collect_button_num = 0;

void filtered_gps_CB(const geometry_msgs::PoseWithCovarianceStamped& gps_msg)
{
		lati_point = gps_msg.pose.pose.position.x;
		longi_point = gps_msg.pose.pose.position.y;

		ori_point_x = gps_msg.pose.pose.orientation.x;
		ori_point_y = gps_msg.pose.pose.orientation.y;
		gps_received = true;
		ROS_INFO("GPS data received");
}


int main(int argc, char** argv)
{
	//Initialize variables
		int numWaypoints = 0;
		std::string path_local;

    // Initialize node and time
		ros::init(argc, argv, "collect_gps_waypoints"); //initiate node called collect_gps_waypoints
		ros::NodeHandle n;
		ros::Time::init();
		ros::Time time_last;
		ros::Time time_current;
		ros::Duration duration_min(1);

	// Get button numbers to collect waypoints and end collection
		// ros::param::get("/gps_waypoint_nav/collect_button_num", collect_button_num);
		// ros::param::get("/gps_waypoint_nav/end_button_num", end_button_num);

    //Initiate subscribers
		ros::Subscriber sub_gps = n.subscribe("/mapviz/gps_waypoint", 100, filtered_gps_CB); ///gps_waypoint_nav/gps/filtered
		ROS_INFO("Initiated collect_gps_waypoints node");

	// Initiate publisher to send end of node message
		ros::Publisher pubCollectionNodeEnded = n.advertise<std_msgs::Bool>("/gps_waypoint_nav/collection_status",100);

    //Read file path and create/open file
    	ros::param::get("/gps_waypoint_nav/coordinates_file", path_local);
		std::string path_abs =  ros::package::getPath("gps_waypoint_nav") + path_local;	
		std::ofstream coordFile (path_abs.c_str());
		ROS_INFO("Saving coordinates to: %s", path_abs.c_str());
		
	// Give instructions:
		// ros::param::get("/gps_waypoint_nav/collect_button_sym", collect_button_sym);
		// ros::param::get("/gps_waypoint_nav/end_button_sym", end_button_sym);
		// std::cout << std::endl;
		// std::cout << "Press " << collect_button_sym.c_str() << " button to collect and store waypoint." << std::endl;
		// std::cout << "Press " << end_button_sym.c_str() << " button to end waypoint collection." << std::endl;
		// std::cout << std::endl;

	// Select a start key
		// while(activation_key != 1)
		// {
		// 	std::cin >> activation_key;

		// 	if(activation_key == 1)
		// 	{
		// 		ROS_INFO("Waypoint collection started!");
		// 		continue_collection = true;
		// 	}
		// 	else
		// 	{
		// 		continue_collection = false;
		// 		ROS_INFO("Invalid key");
		// 	}
		// }

	ROS_INFO("Waypoint collection started!");
	continue_collection = true;

	if(coordFile.is_open())
	{
		ros::spinOnce();
		while(continue_collection)
		{
			ros::spinOnce();
			if (gps_received)
			{
				ros::spinOnce();
				time_current = ros::Time::now();

				if((collect_request == true) && (time_current - time_last > duration_min))
				{
					// Check that there was sufficient change in position between points
					// This makes the move_base navigation smoother and stops points from being collected twice
					double difference_lat = abs((lati_point - lati_last)*pow(10,6))*pow(10,-6);
					double difference_long = abs((longi_point - longi_last)*pow(10,6))*pow(10,-6);

					if( (difference_lat > min_coord_change) || (difference_long > min_coord_change))
					{
						//write waypoint
						ROS_INFO("You have collected another waypoint!");
						std::cout << std::endl;
						numWaypoints++;
						coordFile << std::fixed << std::setprecision(8) << lati_point << " " << longi_point << std::endl;
						lati_last = lati_point;
						longi_last = longi_point;
					}

					else
					{//do not write waypoint
						ROS_WARN("Waypoint not saved, you have not moved enough");
						ROS_WARN("New Latitude: %f   Last Latitude: %f \n", lati_point, lati_last );
						ROS_WARN("New Longitude: %f   Last Longitude: %f \n", longi_point, longi_last );
					}
					time_last = time_current;
				}
				else{}
				gps_received = false;

				//ROS_INFO("Press %s button to end waypoint collection or 3 to continue:", end_button_sym.c_str());
				//activation_key = 0;

				//while (activation_key != 2 && activation_key != 3)
				//{
					//std::cin >> activation_key;
				if (points_counter == 3)	
				{
					// if(activation_key == 2)
					// {
						ROS_INFO("Waypoint collection Ended!");
						continue_collection = false;
						coordFile << std::fixed << std::setprecision(8) << lati_last << " " << longi_last << std::endl;

					// }
					// else if(activation_key == 3)
					// {
					// 	ROS_INFO("Waypoint collection continued!");
					// 	continue_collection = true;
					// }
					// else
					// {
					// 	ROS_INFO("Invalid Option!");
					// }
				}
				else
				{
					ROS_INFO("Waypoint collection continued!");
					continue_collection = true;
				}
				points_counter += 1;
				ros::spinOnce();
			}
			else
			{
			}
		}
	
		coordFile.close();
		ROS_INFO("End request registered.");
	}
	else
	{
		ROS_ERROR("Unable to open file.");
		ROS_INFO("Exiting..");
	}

	ROS_INFO("Closed waypoint file, you have collected %d waypoints.", numWaypoints);
	ROS_INFO("Ending node...");

	// After completing tasks:
    ROS_INFO("Setting parameter 'collect_goals_finished' to true");
    n.setParam("collect_goals_finished", true);

	ros::shutdown();
	return 0;
}

