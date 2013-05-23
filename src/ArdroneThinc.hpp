#ifndef ARDRONE_THINC_HPP_GUARD
#define ARDRONE_THINC_HPP_GUARD

// core
#include "ros/ros.h"

// messages
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"

// services
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_thinc/Waypoint.h"

// persistent storage
#include "opencv2/imgproc/imgproc.hpp"

//sockets
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <pthread.h>

using namespace std; 

namespace smsg = std_msgs;
using ros::Publisher;
using ros::Subscriber;
using ros::ServiceClient;
using ros::ServiceServer;
using geometry_msgs::Twist;
using sensor_msgs::ImageConstPtr;
using ardrone_autonomy::NavdataConstPtr;
using ardrone_thinc::Waypoint;

// move enum
enum dir { LEFT, RIGHT, UP, DOWN };

class ArdroneThinc {
    public:
        // ros topics and services
        Publisher launch_pub;
        Publisher land_pub;
        Publisher reset_pub;
        Publisher twist_pub;
        Publisher thresh_pub;
        Subscriber cam_sub;
        Subscriber nav_sub;
        ServiceClient camchan_cli;
        ServiceClient trim_cli;
        ServiceClient waypoint_cli; 
        ServiceServer waypoint_srv;

        // reusable messages
        smsg::Empty empty_msg;
        Twist twist_msg;

        // grid information
        int columns, rows; 
        int x_scale, y_scale; 

        // grid position, interoperability id
        int x, y, id;
        
        // callback persistent storage
        double rotx, roty;
        int sonar;
        vector<cv::Vec3f> circles; 

        // subscriber callbacks
        void CamCallback(const ImageConstPtr& rosimg);
        void NavdataCallback(const NavdataConstPtr& nav);
        
        // service callbacks
        bool WaypointCallback(Waypoint::Request &req, Waypoint::Response &res);

        // helper functions
        void move(enum dir d); 

        vector<cv::Vec3f> img_vec;
};

#endif
