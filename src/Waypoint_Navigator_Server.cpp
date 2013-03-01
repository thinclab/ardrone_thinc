/*
 * Execute move commands sent from client in "move" function.
 */

#include "ros/ros.h"

bool move(ardrone_thinc::Waypoint_Navigator::Request &req, ardrone_thinc::Waypoint_Navigator::Response &res) {
    //check for valid grid cell & look for drone, then move   
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

/*
 * Determine if the given grid cell (x,y) is valid.
 * Return true if it is, otherwise return false. 
 * Valid values are (0,0) -> (columns, rows)
 */
bool drone_nav::is_valid_grid_cell(int x, int y) {
    if (x < 0 || y < 0 || x > columns || y > rows)
        return false;

    return true;
}

/*
 * Find the drone in the vector of drones with the
 * given id.
 */
drone* drone_nav::find_drone(string id) {
    drone* d;
    for(std::vector<int>::size_type i = 0; i != drones.size(); i++) {
        if (drones[i]->id == id) {
            d = drones[i];
        }
    }

    return d;
}

