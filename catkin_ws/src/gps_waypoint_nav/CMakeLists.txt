cmake_minimum_required(VERSION 3.0.3)
project(gps_waypoint_nav)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  roslib
  roslaunch
  robot_localization
)

catkin_package()
roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(gps_waypoint src/gps_waypoint.cpp)
add_executable(collect_gps_waypoints src/collect_gps_waypoints.cpp)
add_executable(waypoint_nav src/waypoint_nav)

target_link_libraries(gps_waypoint ${catkin_LIBRARIES})
target_link_libraries(collect_gps_waypoints ${catkin_LIBRARIES})
target_link_libraries(waypoint_nav ${catkin_LIBRARIES})

install(DIRECTORY  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
