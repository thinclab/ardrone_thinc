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
#include "ardrone_thinc/PrintNavdata.h"

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
using ardrone_thinc::PrintNavdata;

// move enum
enum dir { LEFT, RIGHT, UP, DOWN, HOV };

/**
 * @file	ArdroneThinc.hpp
 * @author  	David Millard, Emily Wall, Casey Hetzler
 * @version	1.0
 *
 * @section LICENSE
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/licenses/quick-guide-gplv3.html
 *
 * @section DESCRIPTION
 * ArdroneThinc defines a ROS node, utilized by SmartMain GaTACDroneControl for controlling drone clients and keeping track of data, such as navdata.
 * Made for cooperative use with UGA THINC Lab's "ardrone_thinc" package and Autonomy Lab's "ardrone_autonomy" package.
 */

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
        ServiceClient printnavdata_cli; 
        ServiceServer printnavdata_srv;
        // reusable messages
        smsg::Empty empty_msg;
        Twist twist_msg;

        // grid information
        int columns, rows; 
        int x_scale, y_scale; 

	//Real or simulated drones	
	bool simDrones;

        // grid position, interoperability id
        int x, y, id;
        
        // callback persistent storage
        double rotx, roty;
        int sonar;
        float batteryPercent, vx, vy, vz;
	string batteryCurrent, sonarCurrent, forwardVelocityCurrent, sidewaysVelocityCurrent, verticalVelocityCurrent, tagsCountCurrent, tagsTypeCurrent;
	unsigned int tags_count;        //tag detection
	vector<unsigned int> tags_type;         //tag detection
        vector<cv::Vec3f> circles; 

        // subscriber callbacks
        void CamCallback(const ImageConstPtr& rosimg);
        void NavdataCallback(const NavdataConstPtr& nav);
        
        // service callbacks
        bool WaypointCallback(Waypoint::Request &req, Waypoint::Response &res);
 	bool PrintNavdataCallback(PrintNavdata::Request &req, PrintNavdata::Response &res);

        // helper functions
        void move(enum dir d); 

        vector<cv::Vec3f> img_vec;
};

#endif
