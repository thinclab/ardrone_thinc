// sockets
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

// ros
#include "ros/ros.h"
#include "ardrone_thinc/Waypoint.h"

using std::cout;
using std::endl;
using ardrone_thinc::Waypoint;

#define BUFLEN 64
#define LOCALPORTBASE 5000
/**
 * @file	SocketMain.cpp
 * @author  	David Millard, Emily Wall, Vince Capparell
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
 * SocketMain creates a thinc_sock ROS node and allows communication with ArdroneThinc and SmartMain.
 * Made for cooperative use with UGA THINC Lab's "ardrone_thinc" package and Autonomy Lab's "ardrone_autonomy" package.
 */


/**
 * Main method, sets up socket to communicate with thinc_smart node.
 * @return Returns 0 to end the process
 */
int main(int argc, char **argv) {
        // initialize a thinc_sock node with an anonymous name, so multiple instances can run at the same time (for multiple drones)
	ros::init(argc, argv, "thinc_sock", ros::init_options::AnonymousName);
	ros::NodeHandle n;

	if (argc != 4) {
		cout << "usage: " << argv[0] << "<remote-ip> <remote-port> <id>";
		exit(1);
	}

	char *host = argv[1];
	char *port = argv[2];
	struct addrinfo *srv, *cli;
	struct addrinfo hints;

	int id = atoi(argv[3]);
	char localport[5];
	sprintf(localport, "%d", LOCALPORTBASE + id);

	char way_addr[24];
	sprintf(way_addr, "/drone%d/waypoint", id);
	cout << "connected to thinc_smart over " << way_addr << endl;
	ros::ServiceClient way_cli = n.serviceClient < Waypoint > (way_addr);

	// specify socket parameters
	bzero(&hints, sizeof(struct addrinfo));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = 0;
	hints.ai_protocol = IPPROTO_UDP;

	// fill in 'srv' and 'cli' objects with info from 'hints'
	getaddrinfo("localhost", localport, &hints, &srv);
	getaddrinfo(host, port, &hints, &cli);

	// create socket
	int s;
	if ((s = socket(srv->ai_family, srv->ai_socktype, srv->ai_protocol)) < 0) {
		cout << "error creating socket" << endl;
		exit(1);
	}

	// bind socket
	if (bind(s, srv->ai_addr, srv->ai_addrlen) < 0) {
		cout << "error connecting to port" << endl;
		exit(1);
	}

	// feedback
	cout << "listening on UDP port " << localport << endl;
	int nameflags = NI_NUMERICHOST | NI_NUMERICSERV;
	char rhost[64], rserv[64];
	int salen = sizeof(struct sockaddr);
	getnameinfo(cli->ai_addr, salen, rhost, 64, rserv, 64, nameflags);
	cout << "sending obs to " << rhost << " on UDP port " << rserv << endl;

	// read actions for socket, execute, and return observations
	while (ros::ok()) {
		char rbuf[BUFLEN];
		if (recvfrom(s, rbuf, 3, 0, NULL, NULL) < 0) {
			cout << "error receiving from client" << endl;
			exit(1);
		}

		Waypoint waypoint;

		// calculate integer coordinates of destination cell
		int coords[3];
		coords[0] = rbuf[0] - '0';
		coords[1] = rbuf[1] - '0';
		coords[2] = rbuf[2] - '0';

		// set waypoint parameters and call waypoint service
		waypoint.request.x = coords[0];
		waypoint.request.y = coords[1];
		waypoint.request.z = coords[2];
		way_cli.call(waypoint);

		// send waypoint response back
		char sbuf[BUFLEN];

		if (sendto(s, sbuf, BUFLEN, 0, cli->ai_addr, cli->ai_addrlen) < 0) {
			cout << "error sending to client" << endl;
			exit(1);
		}
	}

	// close socket and clean up address information
	freeaddrinfo(srv);
	freeaddrinfo(cli);
	close(s);
	return 0;
}
