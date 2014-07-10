// ros
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_srvs/Empty.h"

// ardrone_thinc
#include "ArdroneThinc.hpp"

// misc
#include <fstream>
#include <cmath>
#include <iostream>

// degree to radian helper macro
#define D2R(a) (a*M_PI/180)


namespace ssrv = std_srvs;

using std::cout;
using std::endl;
using sensor_msgs::ImageConstPtr;
using ardrone_autonomy::NavdataConstPtr;
using ardrone_autonomy::Navdata;
using ardrone_thinc::Waypoint;
using ardrone_thinc::PrintNavdata;

/**
 * @file	ArdroneThinc.cpp
 * @author  	David Millard, Emily Wall, Vince Capparell, Casey Hetzler
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

ArdroneThincInSim::ArdroneThincInSim(int cols, int rows, int startx, int starty, double desired_elev_in_meters, double grid_size_x_in_meters, double grid_size_y_in_meters) {

    k = 15;
    tolerance = 0.3;
    ardroneMass = 0.5;
    hovering = false;
    flying = false;

    // hard code these for now
    x_scale = -grid_size_x_in_meters;
    y_scale = grid_size_y_in_meters;

    this->startx = startx;
    this->starty = starty;

    getCenterOf(startx, starty, this->estX, this->estY);

    goalX = this->estX;
    goalY = this->estY;
    goalZ = desired_elev_in_meters;

    rotz = 0;
    vtheta = 0;
    stopped = false;

    this->columns = cols;
    this->rows = rows;

    ros::NodeHandle n;

	// publishers
    this->launch_pub = n.advertise<smsg::Empty>("ardrone/takeoff", 5);
    this->land_pub = n.advertise<smsg::Empty>("ardrone/land", 5);
    this->reset_pub = n.advertise<smsg::Empty>("ardrone/reset", 5);
    this->twist_pub = n.advertise<Twist>("cmd_vel", 10);

    // subscribers
    this->nav_sub = n.subscribe<Navdata>("ardrone/navdata", 1, &ArdroneThincInSim::NavdataCallback, this);

    // services
    this->waypoint_srv = n.advertiseService("waypoint", &ArdroneThincInSim::WaypointCallback, this);
    this->printnavdata_srv = n.advertiseService("printnavdata", &ArdroneThincInSim::PrintNavdataCallback, this);

    // service clients
    this->trim_cli = n.serviceClient<ssrv::Empty>("ardrone/flattrim");


    // let roscore catch up
    ros::Duration(1.5).sleep();


    // call flat trim - calibrate to flat surface
    ssrv::Empty trim_req;
    this->trim_cli.call(trim_req);

    // takeoff and hover
    this->twist_msg.linear.x = 0;
    this->twist_msg.linear.y = 0;
    this->twist_msg.linear.z = 0;
    this->twist_msg.angular.x = 0;
    this->twist_msg.angular.y = 0;
    this->twist_msg.angular.z = 0;

    takeoff();

}

void ArdroneThincInSim::stop() {
    stopped = true;
}

void ArdroneThincInSim::takeoff() {
    this->launch_pub.publish(this->empty_msg);
}

void ArdroneThincInSim::land() {

    this->land_pub.publish(this->empty_msg);
}

/**
 * Navdata Callback function. Sets drone's data members to values in navdata message
 * @param nav The navdata message returned, with all navdata members available
 */
void ArdroneThincInSim::NavdataCallback(const NavdataConstPtr& nav) {
    this->rotx = nav->rotX;
    this->roty = nav->rotY;
//    this->vtheta = (nav->rotZ - this->rotz) / ((nav->tm - this->lastTimestamp) / 1000000.0);
    this->vtheta = (nav->rotZ - this->rotz) / ((nav->tm - this->lastTimestamp));
    if (isnan(this->vtheta))
        this->vtheta = 0;
    this->rotz = nav->rotZ;

    this->sonar = nav->altd;
    this->batteryPercent = nav->batteryPercent;
    this->vx = nav->vx;
    this->vy = nav->vy;
    this->vz = nav->vz;
    this->aX = nav->ax;
    this->aY = nav->ay;
    this->tags_count = nav->tags_count;
    this->tags_type = nav->tags_type;
    this->flying = (nav->state >= 3 && nav->state <= 5) || nav->state == 7;

    estimateState(nav->tm - this->lastTimestamp);
    springBasedCmdVel(nav->tm - this->lastTimestamp);

    this->lastTimestamp = nav->tm;

}

/**
 * PrintNavdata Callback function. Prints all relevant drone data members to drone-specific text file, for reading/requesting by GaTAC server/client
 * @param &req PrintNavdata request, an empty message
 * @param &res PrintNavdata response, an empty message
 * @return Boolean denoting whether the call was successful
 */
bool ArdroneThincInSim::PrintNavdataCallback(PrintNavdata::Request &req, PrintNavdata::Response &res) {
	cout<< "Navdata print request"<< endl;

    float batteryConvert = this->batteryPercent;
    stringstream ss0 (stringstream::in | stringstream::out);
    ss0 << batteryConvert;
    string batteryString = ss0.str();
    this->batteryCurrent = "Battery percent: " + batteryString;

    float forVelocConvert = this->vy;
   stringstream ss1 (stringstream::in | stringstream::out);
    ss1 << forVelocConvert;
    string forVelocString = ss1.str();
    this->forwardVelocityCurrent = "Forward velocity: " + forVelocString;

    float sideVelocConvert = this->vx;
    stringstream ss2 (stringstream::in | stringstream::out);
    ss2 << sideVelocConvert;
    string sideVelocString = ss2.str();
    this->sidewaysVelocityCurrent = "Sideways velocity: " + sideVelocString;

    float vertVelocConvert = this->vz;
    stringstream ss3 (stringstream::in | stringstream::out);
    ss3 << vertVelocConvert;
    string vertVelocString = ss3.str();
    this->verticalVelocityCurrent  = "Vertical velocity: " + vertVelocString;

    int sonarConvert = this->sonar;
  stringstream ss4 (stringstream::in | stringstream::out);
    ss4 << sonarConvert;
    string sonarString = ss4.str();
    this->sonarCurrent  = "Sonar reading: " + sonarString;

    int tagsCountConvert = this->tags_count;
  stringstream ss5 (stringstream::in | stringstream::out);
    ss5 << tagsCountConvert;
    string tagsCountString = ss5.str();
    this->tagsCountCurrent= "Tags spotted, count: " + tagsCountString;

    if(this->id == 0)
    {
    ofstream file ("currentNavdata0.txt");
    if (file.is_open())
    {
    file << this->batteryCurrent <<"\n";
    file << this->forwardVelocityCurrent <<"\n";
    file << this->sidewaysVelocityCurrent <<"\n";
    file << this->verticalVelocityCurrent <<"\n";
    file << this->sonarCurrent <<"\n";
    file << this->tagsCountCurrent <<"\n";
    }
    }
    else if(this->id == 1)
    {
    ofstream file ("currentNavdata1.txt");
    if (file.is_open())
    {
    file << this->batteryCurrent <<"\n";
    file << this->forwardVelocityCurrent <<"\n";
    file << this->sidewaysVelocityCurrent <<"\n";
    file << this->verticalVelocityCurrent <<"\n";
    file << this->sonarCurrent <<"\n";
    file << this->tagsCountCurrent <<"\n";
    }
    }
    else if(this->id == 2)
    {
    ofstream file ("currentNavdata2.txt");
    if (file.is_open())
    {
    file << this->batteryCurrent <<"\n";
    file << this->forwardVelocityCurrent <<"\n";
    file << this->sidewaysVelocityCurrent <<"\n";
    file << this->verticalVelocityCurrent <<"\n";
    file << this->sonarCurrent <<"\n";
    file << this->tagsCountCurrent <<"\n";
    }
    }

return true;
}

/**
 * Waypoint Callback function. Supplies move function with requested coordinates for drone's movement
 * @param &req Waypoint request sent, with drone ID and desired location
 * @param &res Waypoint response sent back, now empty; formerly printed new location on completion of movement
 * @return Boolean denoting whether the call was successful
 */
bool ArdroneThincInSim::WaypointCallback(Waypoint::Request &req, Waypoint::Response &res) {
    cout << "waypoint request: ";
    cout << req.x << ", " << req.y << endl;

    // ensure valid grid cell
    if(req.x < 0 || req.y < 0 || req.x >= this->columns || req.y >= this->rows) {
        return false;
    }

    // calculate the center of the square
    // set the goal
    // loop until we are within tolerance of it or are no longer flying

    getCenterOf(req.x, req.y, this->goalX, this->goalY);


    while (flying && distanceToGoal() > tolerance) {

        ros::Duration(0.5).sleep();

        if(stopped) {
             return false;
        }

    }

    return true;


}

void ArdroneThincInSim::getStateFor(double x, double y, int & X, int & Y) {
    X = (int)(y / y_scale);
    Y = (int)(x / x_scale);
}

void ArdroneThincInSim::getCenterOf(int X, int Y, double & x, double & y) {
    x = ((double)Y + .5) * y_scale;
    y = ((double)X + .5) * x_scale;
}


double ArdroneThincInSim::distanceToGoal() {
   double distX = goalX - estX;
   double distY = goalY - estY;
   double distZ = goalZ - estZ;

   return  sqrt(distX*distX + distY*distY + distZ*distZ );

}

void ArdroneThincInSim::estimateState(double deltat) {
    deltat /= 1000000;
    this->estX += this->vx / 1000 * deltat + 0.5 * this->aX * 9.8 * deltat * deltat;
    this->estY += this->vy / 1000 * deltat + 0.5 * this->aY * 9.8 * deltat * deltat;
    this->estZ = this->sonar / 1000;

}


void ArdroneThincInSim::springBasedCmdVel(double deltat) {

    if (flying == true) {
        deltat /= 100000; // This is wrong, but for some reason makes the simulation right GO ROS!!!
        double distX = goalX - estX;
        double distY = goalY - estY;
        double distZ = goalZ - estZ;

        // check if we're within tolerance, if so then hover
        double c = 2 * sqrt( k * ardroneMass );

        if (distX*distX + distY*distY + distZ*distZ + rotz*rotz < this->tolerance*this->tolerance) {
            if (! this->hovering) {
                this->twist_msg.linear.x = 0;
                this->twist_msg.linear.y = 0;
                this->twist_msg.linear.z = 0;
                this->twist_msg.angular.x = 0;
                this->twist_msg.angular.y = 0;
                this->twist_msg.angular.z = 0;
                this->twist_pub.publish(this->twist_msg);
                this->hovering = true;
            }
            return;
        }
        this->hovering = false;


        double springForceX = k * distX;
        double dampingForceX = -c * this->vx / 10000;
        double springForceY = k * distY;
        double dampingForceY = -c * this->vy / 10000;
        double springForceZ = k * distZ;
        double dampingForceZ = -c * this->vz / 1000;

        double totalForceX = springForceX + dampingForceX;
        double totalForceY = springForceY + dampingForceY;
        double totalForceZ = springForceZ + dampingForceZ;

        double accelX = totalForceX / ardroneMass;
        double accelY = totalForceY / ardroneMass;
        double accelZ = totalForceZ / ardroneMass;


        double deltaVx = accelX * deltat;
        double deltaVy = accelY * deltat;
        double deltaVz = accelZ * deltat;


        double distTheta = -rotz;
        double springForceTheta = k * distTheta;
        double dampingForceTheta = -c * this->vtheta;
        double totalForceTheta = springForceTheta + dampingForceTheta;
        double accelTheta = totalForceTheta / ardroneMass;
        double deltaTheta = accelTheta * deltat;


        this->twist_msg.linear.x = this->vx / 10000 + deltaVx;
        this->twist_msg.linear.y = this->vy / 10000 + deltaVy;
        this->twist_msg.linear.z = this->vz / 1000 + deltaVz;
        this->twist_msg.angular.x = 0;
        this->twist_msg.angular.y = 0;
        this->twist_msg.angular.z = this->vtheta + deltaTheta;
        if (isnan(this->twist_msg.angular.z))
            this->twist_msg.angular.z = 0;


        this->twist_pub.publish(this->twist_msg);
    }

}
