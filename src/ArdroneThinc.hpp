#ifndef ARDRONE_THINC_HPP_GUARD
#define ARDRONE_THINC_HPP_GUARD

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "drone.hpp"
#include "ardrone_thinc/Waypoint_Navigator.h"
#include <vector> 

using namespace std; 

class ArdroneThinc {
    public:
        vector<ros::ServiceServer> waypoint_navigator_services; 
        vector<ros::ServiceClient> waypoint_navigator_clients; 
        vector<ros::Publisher> launch_publishers;
        vector<ros::Publisher> land_publishers;
        vector<ros::Publisher> reset_publishers;
        vector<ros::Publisher> twist_publishers;
        vector<ros::Publisher> thresh_publishers;
        vector<ros::Subscriber> cam_subscribers;
        vector<ros::ServiceClient> camchannel_clients;
        vector<ros::ServiceClient> flattrim_clients;

        std_msgs::Empty empty_msg;
        geometry_msgs::Twist twist_msg;
        vector<drone*> drones; 
        int columns; 
        int rows; 

        void CamCallback(const sensor_msgs::ImageConstPtr& rosimg);
        bool Waypoint_Navigator_Callback(ardrone_thinc::Waypoint_Navigator::Request 
            &req, ardrone_thinc::Waypoint_Navigator::Response &res);
        bool is_valid_grid_cell(int, int); 
//        bool is_right_color(int, int, ---color?---); 
        void move(int, char); 
       
};

#endif
