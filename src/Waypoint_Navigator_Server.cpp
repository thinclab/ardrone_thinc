/*
 * Execute move commands sent from client in "move" function.
 */

#include "ros/ros.h"

bool move(ardrone_thinc::Waypoint_Navigator::Request &req, ardrone_thinc::Waypoint_Navigator::Response &res) {
    
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_navigator_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("waypoint_navigator_server", move);
    ROS_INFO("Ready to move.");
    ros::spin();
    return 0;
}
