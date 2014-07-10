// ros
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
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
#include <signal.h>

// ardrone_autonomy
#include "ardrone_autonomy/CamSelect.h"
#include "ardrone_autonomy/Navdata.h"

// ardrone_thinc
#include "ArdroneThinc.hpp"

using namespace cv;
using namespace std;

namespace smsg = std_msgs;
using geometry_msgs::Twist;
using sensor_msgs::Image;
using ardrone_autonomy::Navdata;
using ardrone_autonomy::CamSelect;

ArdroneThinc * at;
ros::AsyncSpinner  *spinner;

void mySigintHandler(int sig)
{
  at->stop();  //this is needed because these ros dipshits can't correctly handle threads, probably can be removed after they buy a clue
  spinner->stop();
  ros::shutdown();
}

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
    spinner = new ros::AsyncSpinner(2);

    // data container

    // handle usage
    if (argc != 8) {
        cout << "usage: ";
        cout << argv[0];
        cout << " <cols> <rows> <drone-col> <drone-row> <size of col in meters> <size of row in meters> <elevation in meters> <if flying simulated drones, last argument is 's', else last argument is 'r'";
        cout << endl;
        exit(1);
    }


    // grid size
    int columns = atoi(argv[1]);
    int rows = atoi(argv[2]);
    int startcol = atoi(argv[3]);
    int startrow = atoi(argv[4]);
    double colsize = atof(argv[5]);
    double rowsize = atof(argv[6]);
    double elev = atof(argv[7]);


    if (strcmp(argv[8],"s") == 0)
    {
        //we are simulating
        at = new ArdroneThincInSim(columns, rows, startcol, startrow, elev, colsize, rowsize);

    }

    signal(SIGINT, mySigintHandler);

    spinner->start();
    ros::waitForShutdown();

    return 0;
}
