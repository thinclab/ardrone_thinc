// ros
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"

// opencv2
#include "sensor_msgs/image_encodings.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

// C++
#include <cstdlib>
#include <stdio.h>
#include <string.h>


// ardrone_autonomy
#include "ardrone_autonomy/CamSelect.h"
#include "ardrone_autonomy/Navdata.h"

// ardrone_thinc
#include "ArdroneThinc.hpp"

using namespace cv;
using namespace std;

namespace smsg = std_msgs;
namespace ssrv = std_srvs;
using geometry_msgs::Twist;
using sensor_msgs::Image;
using ardrone_autonomy::Navdata;
using ardrone_autonomy::CamSelect;

/**
 * @file	SmartMain.cpp
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
 * SmartMain creates an ArdroneThinc ROS node and allows communication with that node via the GaTACDroneControl API.
 * Made for cooperative use with UGA THINC Lab's "ardrone_thinc" package and Autonomy Lab's "ardrone_autonomy" package.
 */

/**
 * Main method. This begins a node, initializes all subscribers/publishers/members, and compels the drone to take off and wait for futrther command
 * @param argc Count of command line arguments
 * @param *argv[] Array of command line arguments
 * @return Returns 0 when ROS stops spinning and exits the main function
 */
int main(int argc, char *argv[]) {
    // ros initialization
    ros::init(argc, argv, "thinc_main");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::AsyncSpinner spinner(2);

    // data container
    ArdroneThinc at;
    at.simDrones = false;

    // handle usage
    if (argc != 7) {
        cout << "usage: ";
        cout << argv[0];
        cout << " <cols> <rows> <drone-id> <drone-col> <drone-row> <if flying simulated drones, last argument is 's', else last argument is 'r'";
        cout << endl;
        exit(1);
    }
    if (argc == 7 && (strcmp(argv[6],"s") == 0))
    {
     //we are simulating
     at.simDrones = true;

    // grid size
    at.columns = atoi(argv[1]);
    at.rows = atoi(argv[2]);

    // initial position and id
    at.id = atoi(argv[3]);
    at.x = atoi(argv[4]);
    at.y = atoi(argv[5]);
    }
    else {
     //we are not simulating, these are real drones
     at.simDrones = false;

    // grid size
    at.columns = atoi(argv[1]);
    at.rows = atoi(argv[2]);

    // initial position and id
    at.id = atoi(argv[3]);
    at.x = atoi(argv[4]);
    at.y = atoi(argv[5]);
    }

	// publishers
    at.launch_pub = n.advertise<smsg::Empty>("ardrone/takeoff", 5);
    at.land_pub = n.advertise<smsg::Empty>("ardrone/land", 5);
    at.reset_pub = n.advertise<smsg::Empty>("ardrone/reset", 5);
    at.twist_pub = n.advertise<Twist>("cmd_vel", 10);
    at.thresh_pub = n.advertise<Image>("img_thresh", 10);

    // subscribers
    at.cam_sub = n.subscribe<Image>("ardrone/image_raw", 1, &ArdroneThinc::CamCallback, &at);
    at.nav_sub = n.subscribe<Navdata>("ardrone/navdata", 1, &ArdroneThinc::NavdataCallback, &at);

    // services
    at.waypoint_srv = n.advertiseService("waypoint", &ArdroneThinc::WaypointCallback, &at);
    at.printnavdata_srv = n.advertiseService("printnavdata", &ArdroneThinc::PrintNavdataCallback, &at);

    // service clients
    at.camchan_cli = n.serviceClient<CamSelect>("ardrone/setcamchannel", 1);
    at.trim_cli = n.serviceClient<ssrv::Empty>("ardrone/flattrim");
    at.waypoint_cli = n.serviceClient<Waypoint>("waypoint");
    at.printnavdata_cli = n.serviceClient<PrintNavdata>("printnavdata");

    // let roscore catch up
    ros::Duration(1.0).sleep();

    // set camchannel on drone and takeoff
    if(ros::ok()) {
        // set camera to bottom
        ardrone_autonomy::CamSelect camchan_req;
        camchan_req.request.channel = 1;
        at.camchan_cli.call(camchan_req);

        // call flat trim - calibrate to flat surface
        ssrv::Empty trim_req;
        at.trim_cli.call(trim_req);
        if(at.simDrones == true){
            // takeoff and hover
            at.twist_msg.linear.x = 0;
            at.twist_msg.linear.y = 0;
            at.twist_msg.linear.z = 0;
            at.twist_msg.angular.x = 0;
            at.twist_msg.angular.y = 0;
            at.twist_msg.angular.z = 0;
            at.launch_pub.publish(at.empty_msg);

            ardrone_thinc::Waypoint waypoint_msg;
            waypoint_msg.request.x = 0;
            waypoint_msg.request.y = 0;
            waypoint_msg.request.z = 0;
            waypoint_msg.request.id = 0;
        }
    }

    spinner.start();
    ros::waitForShutdown();
    if(at.id == 0)
	remove("currentNavdata0.txt");
    else if(at.id == 1)
	remove("currentNavdata1.txt");
    else if(at.id == 2)
	remove("currentNavdata2.txt");
    return 0;
}
