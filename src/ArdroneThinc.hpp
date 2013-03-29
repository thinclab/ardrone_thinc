#ifndef ARDRONE_THINC_HPP_GUARD
#define ARDRONE_THINC_HPP_GUARD

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "drone.hpp"
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_thinc/Waypoint_Navigator.h"
#include <vector> 

using namespace std; 

class ArdroneThinc {
    public:
        ros::ServiceServer waypoint_navigator_service; 
        ros::ServiceClient waypoint_navigator_client; 
        vector<ros::Publisher> launch_publishers;
        vector<ros::Publisher> land_publishers;
        vector<ros::Publisher> reset_publishers;
        vector<ros::Publisher> twist_publishers;
        vector<ros::Publisher> thresh_publishers;
        vector<ros::Subscriber> cam_subscribers;
        vector<ros::Subscriber> navdata_subscribers;
        vector<ros::ServiceClient> camchannel_clients;
        vector<ros::ServiceClient> flattrim_clients;

        std_msgs::Empty empty_msg;
        geometry_msgs::Twist twist_msg;
        vector<drone*> drones; 
        int columns; 
        int rows; 

        // navdata readings
        double rotx, roty;
        int sonar;

        // subscriber callbacks
        void CamCallback(const sensor_msgs::ImageConstPtr& rosimg);
        void NavdataCallback(const ardrone_autonomy::NavdataConstPtr& nav);

        // service callback
        bool Waypoint_Navigator_Callback(ardrone_thinc::Waypoint_Navigator::Request 
            &req, ardrone_thinc::Waypoint_Navigator::Response &res);

        // helper functions
        bool is_valid_grid_cell(int, int); 
        void move(int, char); 
       
};

#endif
