#ifndef ARDRONE_THINC_HPP_GUARD
#define ARDRONE_THINC_HPP_GUARD

// core
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

// messages
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

// services
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_thinc/Waypoint.h"
#include "uga_tum_ardrone/filter_state.h"
#include "std_srvs/Empty.h"

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
using uga_tum_ardrone::filter_state;

/**
 * Enumerated movement directions
 */
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

        virtual bool LandAtHome(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) = 0;

        virtual bool LandHere(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) = 0;

        virtual void stop() = 0;

        virtual bool Takeoff(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) = 0;

        /**
         * Waypoint Callback function. Supplies move function with requested coordinates for drone's movement
         * @param &req Waypoint request sent, with drone ID and desired location
         * @param &res Waypoint response sent back, now empty; formerly printed new location on completion of movement
         * @return Boolean denoting whether the call was successful
         */
        virtual bool WaypointCallback(Waypoint::Request &req, Waypoint::Response &res) = 0;

        virtual void PublishPosition() = 0;


};

class ArdroneThincInSim : public ArdroneThinc {
    public:

        ArdroneThincInSim(int grid_count_x, int grid_count_y, int startx, int starty, double desired_elev_in_meters, double grid_size_x_in_meters, double grid_size_y_in_meters);

        // subscriber callbacks

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
        virtual bool WaypointCallback(Waypoint::Request &req, Waypoint::Response &res);

        virtual void PublishPosition();

        // helper functions

        virtual bool LandAtHome(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

        virtual bool LandHere(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

        virtual void stop();

        virtual bool Takeoff(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

    private:

        // these are the variables used to estimate the drone's position
        double estX, estY, estZ; // in meters
        double aX, aY, aZ; // in meters a second second
        double lastTimestamp; // in microseconds !

        double goalX, goalY, goalZ; // in meters
        bool hovering;
        bool flying;
        double ardroneMass; // in kilograms
        double tolerance;
        double k; // in newtons per meter

        double vtheta;

        ros::Time lastPositionPublish;
        int lastPublishX, lastPublishY;

        void estimateState(double deltat);
        void springBasedCmdVel(double deltat);

        void getStateFor(double x, double y, int & X, int & Y);
        void getCenterOf(int X, int Y, double & x, double & y);
        double distanceToGoal();

        bool stopped;


        int startx, starty;

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
        * @brief Publisher for position topic
        */
        Publisher pose_pub;

        /**
        * @brief Subscriber for navdata topic, uses navdata topic
        */
        Subscriber nav_sub;

        /**
        * @brief Service client for flattrim service
        */
        ServiceClient trim_cli;

        /**
        * @brief Service server for waypoint service
        */
        ServiceServer waypoint_srv;

        /**
        * @brief Service client for waypoint service
        */
        ServiceClient waypoint_cli;

        ServiceServer takeoff_srv;

        ServiceClient takeoff_cli;

        ServiceServer land_srv;

        ServiceServer land_here_srv;

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
        * @brief Drone's unique ID
        */
        int id;

        // callback persistent storage
        double rotx, roty, rotz;

        /**
        * @brief Drone's current sonar reading
        */
        double sonar;

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


};



class ArdroneThincInReality : public ArdroneThinc {
    public:

        ArdroneThincInReality(int grid_count_x, int grid_count_y, int startx, int starty, double desired_elev_in_meters, double grid_size_x_in_meters, double grid_size_y_in_meters);



        /**
         * Waypoint Callback function. Supplies move function with requested coordinates for drone's movement
         * @param &req Waypoint request sent, with drone ID and desired location
         * @param &res Waypoint response sent back, now empty; formerly printed new location on completion of movement
         * @return Boolean denoting whether the call was successful
         */
        virtual bool WaypointCallback(Waypoint::Request &req, Waypoint::Response &res);

        virtual void PublishPosition();

        // helper functions


        virtual bool LandAtHome(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

        virtual bool LandHere(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

        virtual void stop();

        virtual bool Takeoff(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);



        void PoseCallback(const uga_tum_ardrone::filter_stateConstPtr& fs);

        void TumCommandCallback(const std_msgs::StringConstPtr& msg);

    private:


       bool stopped;


        int startx, starty;

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


        Publisher tum_command;

        /**
        * @brief Publisher for position topic
        */
        Publisher pose_pub;

        Subscriber tum_pose;


        /**
        * @brief Service client for flattrim service
        */
        ServiceClient trim_cli;

        /**
        * @brief Service server for waypoint service
        */
        ServiceServer waypoint_srv;

        /**
        * @brief Service client for waypoint service
        */
        ServiceClient waypoint_cli;

        ServiceServer takeoff_srv;

        ServiceClient takeoff_cli;

        ServiceServer land_srv;

        ServiceServer land_here_srv;


        bool transformBuilt;

        tf::Vector3 grid_to_world_scale;
        double ptam_scale;


        tf::Vector3 cur_goal;
        tf::Vector3 cur_pos;

        double tolerance;

        int cols;
        int rows;

        std_msgs::Empty empty_msg;

        bool command_queue_clear;

        Subscriber tum_pose_sub;

        bool is_flying;
        bool has_takenoff;

        ros::Time lastPositionPublish;
        int lastPublishX, lastPublishY;
};

#endif
