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
 * ArdroneThinc defines a ROS node, utilized by SmartMain and GaTACDroneControl for controlling drone clients and keeping track of data, such as navdata.
 * Made for cooperative use with UGA THINC Lab's "ardrone_thinc" package and Autonomy Lab's "ardrone_autonomy" package.
 */

class ArdroneThinc {
    public:
	
        /**
	 * @brief Publisher for launch topic, uses takeoff service
	 */
        Publisher launch_pub;
	
	/**
	 * @brief Publisher for land topic, uses land service
	 */
        Publisher land_pub;

	/**
	 * @brief Publisher for reset topic, uses reset service
	 */
        Publisher reset_pub;

	/**
	 * @brief Publisher for twist topic, uses cmd_vel topic
	 */
        Publisher twist_pub;

	/**
	 * @brief Publisher for image topic, uses img_thresh topic
	 */
        Publisher thresh_pub;

	/**
	 * @brief Subscriber for camera topic, uses image_raw topic
	 */
        Subscriber cam_sub;

	/**
	 * @brief Subscriber for navdata topic, uses navdata topic
	 */
        Subscriber nav_sub;

	/**
	 * @brief Service client for setcamchannel service
	 */
        ServiceClient camchan_cli;

	/**
	 * @brief Service client for flattrim service
	 */
        ServiceClient trim_cli;

	/**
	 * @brief Service client for waypoint service
	 */
        ServiceClient waypoint_cli; 

	/**
	 * @brief Service server for waypoint service
	 */
        ServiceServer waypoint_srv;

	/**
	 * @brief Service client for printnavdata service
	 */
        ServiceClient printnavdata_cli; 

	/**
	 * @brief Service server for printnavdata service
	 */
        ServiceServer printnavdata_srv;

	/**
	 * @brief Empty message, reused for communication between ArdroneThinc and SmartMain
	 */
        smsg::Empty empty_msg;
	/**
	 * @brief Twist message, used in conjunction with cmd_vel topic to implement Waypoint service
	 */
        Twist twist_msg;

        // grid information

	/**
	 * @brief Columns in drone's grid
	 */
        int columns;

	/**
	 * @brief Rows in drone's grid
	 */
	int rows; 
        int x_scale;
	int y_scale; 

	//Real or simulated drones
	/**
	 * @brief Boolean that tells whether this drone's node is a real or simulated drone
	 */	
	bool simDrones;

        // grid position, interoperability id

	/**
	 * @brief Drone's current x position
	 */
        int x;

	/**
	 * @brief Drone's current y position
	 */
	int y;

	/**
	 * @brief Drone's unique ID
	 */
	int id;
        
        // callback persistent storage
        double rotx, roty;
	
	/**
	 * @brief Drone's current sonar reading
	 */
        int sonar;

	/**
	 * @brief Drone's current battery reading
	 */
        float batteryPercent; 

	/**
	 * @brief Drone's current sideways velocity reading
	 */
	float vx;

	/**
	 * @brief Drone's current forward velocity reading
	 */
	float vy;

	/**
	 * @brief Drone's current vertical velocity reading
	 */
	float vz;

	/**
	 * @brief Drone's current battery, represented as human-readable string
	 */
	string batteryCurrent;

	/**
	 * @brief Drone's current sonar, represented as human-readable string
	 */
	string sonarCurrent;

	/**
	 * @brief Drone's current forward velocity, represented as human-readable string
	 */
	string forwardVelocityCurrent;

	/**
	 * @brief Drone's current sideways velocity, represented as human-readable string
	 */
	string sidewaysVelocityCurrent;

	/**
	 * @brief Drone's current vertical velocity, represented as human-readable string
	 */
	string verticalVelocityCurrent;
	
	/**
	 * @brief Drone's current tag count, represented as human-readable string
	 */
	string tagsCountCurrent;

	/**
	 * @brief Drone's current tag spotted type, represented as human-readable string
	 */
	string tagsTypeCurrent;

	/**
	 * @brief Drone's current tag count reading
	 */
	unsigned int tags_count;        

	/**
	 * @brief Drone's current tag type reading, vector
	 */
	vector<unsigned int> tags_type;         

	/**
	 * @brief Circles, used in deprecated CamCallback
	 */
        vector<cv::Vec3f> circles; 

	/**
	 * @brief Image vector, used in deprecated CamCallback
	 */
        vector<cv::Vec3f> img_vec;

        // subscriber callbacks

	/**
	 * Camera Callback function. Now deprecated. Formerly meant to help drone center self on grid cell.
	 * @param rosimg The sensor message (image) returned by the currently toggled camera
	 */
        void CamCallback(const ImageConstPtr& rosimg);

	/**
	 * Navdata Callback function. Sets drone's data members to values in navdata message
	 * @param nav The navdata message returned, with all navdata members available
	 */
        void NavdataCallback(const NavdataConstPtr& nav);
        
        // service callbacks

	/**
	 * Waypoint Callback function. Supplies move function with requested coordinates for drone's movement
	 * @param &req Waypoint request sent, with drone ID and desired location
	 * @param &res Waypoint response sent back, now empty; formerly printed new location on completion of movement
	 * @return Boolean denoting whether the call was successful
	 */
        bool WaypointCallback(Waypoint::Request &req, Waypoint::Response &res);

	/**
	 * PrintNavdata Callback function. Prints all relevant drone data members to drone-specific text file, for reading/requesting by GaTAC server/client
	 * @param &req PrintNavdata request, an empty message
	 * @param &res PrintNavdata response, an empty message
	 * @return Boolean denoting whether the call was successful
	 */
 	bool PrintNavdataCallback(PrintNavdata::Request &req, PrintNavdata::Response &res);

        // helper functions

	/**
	 * Move function, updates drone's position and publishes Twist messages to cmd_vel topic
	 * @param dir d The direction the drone will move
	 */
        void move(enum dir d); 
};

#endif
