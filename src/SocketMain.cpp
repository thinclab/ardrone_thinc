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

int main(int argc, char **argv) {
    ros::init(argc, argv, "thinc_sock");
    ros::NodeHandle n;

    if(argc != 4) {
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
    ros::ServiceClient way_cli = n.serviceClient<Waypoint>(way_addr);

    // socket parameters
    bzero(&hints, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = 0;
    hints.ai_protocol = IPPROTO_UDP;

    // address resolution
    getaddrinfo("localhost", localport, &hints, &srv);
    getaddrinfo(host, port, &hints, &cli);

    // socket
    int s;
    if((s = socket(srv->ai_family, srv->ai_socktype, srv->ai_protocol)) < 0) {
       cout << "error creating socket" << endl;
       exit(1);
    }

    // bind
    if(bind(s, srv->ai_addr, srv->ai_addrlen) < 0) {
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
    while(ros::ok()) {
        // read action
        char rbuf[BUFLEN];
        if(recvfrom(s, rbuf, 4, 0, NULL, NULL) < 0) {
            cout << "error receiving from client" << endl;
            exit(1);
        }

        cout << (int) rbuf[0] << " ";
        cout << (int) rbuf[1] << " ";
        cout << (int) rbuf[2] << " ";
        cout << (int) rbuf[3] << endl;
        // execute with on thinc_smart
        Waypoint waypoint;
        waypoint.request.x = rbuf[0];
        waypoint.request.y = rbuf[1];
        waypoint.request.z = rbuf[2];
        waypoint.request.id = rbuf[3];
        way_cli.call(waypoint);


        char sbuf[BUFLEN];
        //sbuf[0] = 97;
        //sbuf[1] = 98;
        //sbuf[2] = 99;
        //sbuf[3] = 100;
        //sbuf[4] = 0;
        sbuf[0] = waypoint.response.x;
        sbuf[1] = waypoint.response.y;
        sbuf[2] = waypoint.response.z;
        sbuf[3] = 0;
        if(sendto(s, sbuf, BUFLEN, 0, cli->ai_addr, cli->ai_addrlen) < 0) {
            cout << "error sending to client" << endl;
            exit(1);
        }
    }

    freeaddrinfo(srv);
    freeaddrinfo(cli);
    close(s);
    return 0;
}
