/*
 * Prompt for grid input. Pass info to server to execute
 */

#include "ros/ros.h"
#include "ArdroneThinc.hpp"
#include <cstdlib>

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_navigator_client");
    if (argc != 3) {
        ROS_INFO("usage: waypoint_navigator_client X Y");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ArdroneThinc::Waypoint_Navigator>("waypoint_navigator");
    ArdroneThinc::Waypoint_Navigator srv;
    srv.request.x = atoll(argv[1]);
    srv.request.y = atoll(argv[2]);
    if (client.call(srv))
        ROS_INFO("Response: %ld", (long int)srv.response.success);
    else {
        ROS_ERROR("Failed to call service waypoint_navigator");
        return 1;
    }

    return 0;
}
