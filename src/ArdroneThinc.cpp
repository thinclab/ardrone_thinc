// ros
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "ardrone_autonomy/Navdata.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"

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
    ardroneMass = 1.48;
    hovering = false;
    flying = false;

    // hard code these for now
    x_scale = -grid_size_x_in_meters;
    y_scale = grid_size_y_in_meters;

    this->startx = startx;
    this->starty = starty;

    this->lastTimestamp = -1;

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
    this->takeoff_srv = n.advertiseService("takeoff_thinc_smart", &ArdroneThincInSim::Takeoff, this);
    this->land_srv = n.advertiseService("land_at_home", &ArdroneThincInSim::LandAtHome, this);

    // service clients
    this->trim_cli = n.serviceClient<ssrv::Empty>("ardrone/flattrim");
    this->waypoint_cli = n.serviceClient<Waypoint>("waypoint");
    this->takeoff_cli = n.serviceClient<ssrv::Empty>("takeoff_thinc_smart");

    // let roscore catch up
    ros::Duration(1.5).sleep();


    // takeoff and hover
    this->twist_msg.linear.x = 0;
    this->twist_msg.linear.y = 0;
    this->twist_msg.linear.z = 0;
    this->twist_msg.angular.x = 0;
    this->twist_msg.angular.y = 0;
    this->twist_msg.angular.z = 0;

}

void ArdroneThincInSim::stop() {
    stopped = true;
}

bool ArdroneThincInSim::Takeoff(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {

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
    this->twist_pub.publish(this->twist_msg);

    this->launch_pub.publish(this->empty_msg);

    while (! flying ) {

        ros::Duration(0.25).sleep();

        if(stopped) {
             return false;
        }

    }

    ardrone_thinc::Waypoint gohome;
    gohome.request.x = startx;
    gohome.request.y = starty;
    gohome.request.z = goalZ;

    this->waypoint_cli.call(gohome);

    return true;
}

bool ArdroneThincInSim::LandAtHome(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {

    ardrone_thinc::Waypoint gohome;
    gohome.request.x = startx;
    gohome.request.y = starty;
    gohome.request.z = -1;

    this->waypoint_cli.call(gohome);
    this->land_pub.publish(this->empty_msg);

    return true;
}

/**
 * Navdata Callback function. Sets drone's data members to values in navdata message
 * @param nav The navdata message returned, with all navdata members available
 */
void ArdroneThincInSim::NavdataCallback(const NavdataConstPtr& nav) {
    if (this->lastTimestamp < 0) {
	this->lastTimestamp = nav->tm;
	this->rotz = nav->rotZ;
	return;
    }

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
    if(req.x < 0 || req.y < 0 || req.x >= this->columns || req.y >= this->rows || req.z > 4.9) {
        return false;
    }

    // calculate the center of the square
    // set the goal
    // loop until we are within tolerance of it or are no longer flying

    getCenterOf(req.x, req.y, this->goalX, this->goalY);

    if (req.z > 0)
        this->goalZ = req.z;

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


ArdroneThincInReality::ArdroneThincInReality(int cols, int rows, int startx, int starty, double desired_elev_in_meters, double grid_size_x_in_meters, double grid_size_y_in_meters) {

    ros::NodeHandle n;

	// publishers
    this->launch_pub = n.advertise<smsg::Empty>("ardrone/takeoff", 5);
    this->land_pub = n.advertise<smsg::Empty>("ardrone/land", 5);
    this->reset_pub = n.advertise<smsg::Empty>("ardrone/reset", 5);
    this->tum_command = n.advertise<std_msgs::String>("tum_ardrone/com", 5);

    // subscribers
    this->tum_pose = n.subscribe<tum_ardrone::filter_state>("ardrone/predictedPose", 1, &ArdroneThincInReality::PoseCallback, this);
    this->tum_pose_sub = n.subscribe<std_msgs::String>("tum_ardrone/com", 1, &ArdroneThincInReality::TumCommandCallback, this);

    // services
    this->waypoint_srv = n.advertiseService("waypoint", &ArdroneThincInReality::WaypointCallback, this);
    this->printnavdata_srv = n.advertiseService("printnavdata", &ArdroneThincInReality::PrintNavdataCallback, this);
    this->takeoff_srv = n.advertiseService("takeoff_thinc_smart", &ArdroneThincInReality::Takeoff, this);
    this->land_srv = n.advertiseService("land_at_home", &ArdroneThincInReality::LandAtHome, this);

    // service clients
    this->trim_cli = n.serviceClient<ssrv::Empty>("ardrone/flattrim");
    this->waypoint_cli = n.serviceClient<Waypoint>("waypoint");
    this->takeoff_cli = n.serviceClient<ssrv::Empty>("takeoff_thinc_smart");

    // let roscore catch up
    ros::Duration(1.5).sleep();

    tolerance = 0.5;

    // need to initialize the transform from the grid's coordinate system to tum's

    // wait until ptam's state goes to 3, 4, or 5, read in the elevation from navdata, now build a transform from the grid coordinate (i am in square x,y) to the estimated pose x,y
    // and from the sonar's elevation to tum's z component.
    // we can then request to go to a certain point in grid space by transforming into tum space

    transformBuilt = false;

    grid_to_world_scale = tf::Vector3(grid_size_x_in_meters, grid_size_y_in_meters, 1.0);

    cout << "Init params: " << cols << " " << rows << " " << startx << " " << starty << " " << desired_elev_in_meters << " " << grid_size_x_in_meters << " " << grid_size_y_in_meters << endl;

    cur_goal = tf::Vector3(startx, starty, desired_elev_in_meters);

    this->cols = cols;
    this->rows = rows;

    this->startx = startx;
    this->starty = starty;

    this->command_queue_clear = false;

}


bool ArdroneThincInReality::WaypointCallback(Waypoint::Request &req, Waypoint::Response &res) {

    cout << "waypoint request: ";
    cout << req.x << ", " << req.y << endl;

    // ensure valid grid cell
    if(req.x < 0 || req.y < 0 || req.x >= this->cols|| req.y >= this->rows || req.z > 4.9) {
        cout << "Invalid Request: " << req.x << ", " << req.y << ", " << req.z << endl;
        return false;
    }

    if (! is_flying) {
        cout << "Not yet flying, taking off " << endl;
        ssrv::Empty takeoff_req;
        this->takeoff_cli.call(takeoff_req);
    }

    while (! transformBuilt) {

        ros::Duration(0.5).sleep();

        if(stopped) {
             return false;
        }

    }


    tf::Vector3 newGridPos(req.x, req.y, req.z);

    if (req.z <= 0) {
        newGridPos.setZ(cur_goal.z());
    }

    cur_goal = tf::Vector3(newGridPos.x(), newGridPos.y(), newGridPos.z());

    newGridPos *= grid_to_world_scale / ptam_scale;

    char msg[256];
	sprintf(msg, "c goto %f %f %f %f\n", newGridPos.x(), newGridPos.y(), newGridPos.z(), 0.0);

    cout << msg << endl;

    std_msgs::String outmsg;
    outmsg.data = std::string(msg);

    tum_command.publish(outmsg);

    ros::Duration(1).sleep();

    while (!command_queue_clear) {
        ros::Duration(0.5).sleep();

        cout <<"(" << cur_goal.x() << ", " << cur_goal.y() << ", " << cur_goal.z() << ") (" << cur_pos.x() << ", " << cur_pos.y() << ", " << cur_pos.z() << ")" << endl;
        if(stopped) {
             return false;
        }
    }

    return true;
}

void ArdroneThincInReality::TumCommandCallback(const std_msgs::StringConstPtr& msg) {

    // look for a status message

    int queue_size;
    if(sscanf(msg->data.c_str(),"u c Controlling (Queue: %d)", &queue_size)) {
        this->command_queue_clear = ! queue_size;
    }

}

bool ArdroneThincInReality::PrintNavdataCallback(PrintNavdata::Request &req, PrintNavdata::Response &res) {
    return false;
}

bool ArdroneThincInReality::LandAtHome(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {

    std_msgs::String outmsg;
    outmsg.data = std::string("c clearCommands");
    tum_command.publish(outmsg);

    ardrone_thinc::Waypoint gohome;
    gohome.request.x = startx;
    gohome.request.y = starty;
    gohome.request.z = -1;

    this->waypoint_cli.call(gohome);


    outmsg.data = std::string("c setInitialReachDist 0.075");
    tum_command.publish(outmsg);

    outmsg.data = std::string("c setStayWithinDist 0.09");
    tum_command.publish(outmsg);

    outmsg.data = std::string("c setStayTime 1");
    tum_command.publish(outmsg);

    gohome.request.x = startx;
    gohome.request.y = starty;
    gohome.request.z = 0.33;

    this->waypoint_cli.call(gohome);

    outmsg.data = std::string("c land");
    tum_command.publish(outmsg);

    outmsg.data = std::string("c stop");
    tum_command.publish(outmsg);

//    outmsg.data = std::string("f reset");
//    tum_command.publish(outmsg);

    this->land_pub.publish(this->empty_msg);

    return true;
}

void ArdroneThincInReality::stop() {
    stopped = true;
    this->land_pub.publish(this->empty_msg);
}

bool ArdroneThincInReality::Takeoff(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {

    // call flat trim - calibrate to flat surface
    ssrv::Empty trim_req;
    this->trim_cli.call(trim_req);

    cout << "Begin takeoff" << endl;
    std_msgs::String outmsg;

    outmsg.data = std::string("c clearCommands");
    tum_command.publish(outmsg);

    outmsg.data = std::string("c autoInit 500 800 4000 0.5");
    tum_command.publish(outmsg);

    outmsg.data = std::string("c setInitialReachDist 0.1");
    tum_command.publish(outmsg);

    outmsg.data = std::string("c setStayWithinDist 0.15");
    tum_command.publish(outmsg);

    outmsg.data = std::string("c setStayTime 1");
    tum_command.publish(outmsg);

    outmsg.data = std::string("c start");
    tum_command.publish(outmsg);


    while (! transformBuilt) {

        ros::Duration(0.5).sleep();

        if(stopped) {
             return false;
        }

    }

    cout << "End takeoff" << endl;

    return true;
}



void ArdroneThincInReality::PoseCallback(const tum_ardrone::filter_stateConstPtr& fs) {

    if (fs->droneState >= 3 && fs->droneState <= 7) {
        is_flying = true;
    } else {
        is_flying = false;
        transformBuilt = false;
    }

    if (!transformBuilt && is_flying) {
        // Is tum ready and are we flying?  If so, pull the x, y, z and yaw out and build the transform

        if (fs->ptamState >= 3 && fs->ptamState <= 4) {
            // subtract the starting state scaled properly
            // add the current x and y from tum
            // save the orientation

            ptam_scale = fs->scale;

            cout << "Building Transformation: " << fs->ptamState << endl;
            char msg[256];
            sprintf(msg, "c setReference %f %f %f %f", -startx * grid_to_world_scale.x() / fs->scale, -starty * grid_to_world_scale.y() / fs->scale, 0.0, fs->yaw);

            std_msgs::String outmsg;
            outmsg.data = std::string(msg);

            cout << msg << endl;
            tum_command.publish(outmsg);

            tf::Vector3 newGridPos = cur_goal;
            newGridPos *= grid_to_world_scale / ptam_scale;

            sprintf(msg, "c goto %f %f %f %f", newGridPos.x(), newGridPos.y(), newGridPos.z(), 0.0);

            cout << msg << endl;
            outmsg.data = std::string(msg);
            tum_command.publish(outmsg);

            transformBuilt = true;
        } else {
            return;
        }
    }

    ptam_scale = fs->scale;

    cur_pos = ptam_scale * tf::Vector3(fs->x, fs->y, fs->z) / grid_to_world_scale + tf::Vector3(startx, starty, 0);

}
