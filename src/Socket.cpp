//ros
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"


//sockets
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>

int main(int argc, char**argv) {
    ros::init(argc, argv, "rocket_sockets");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    int sockfd;
    int n;
    struct sockaddr_in servaddr;
    struct sockaddr_in cliaddr;
    socklen_t len;
    char mesg[1000];

    sockfd=socket(AF_INET, SOCK_DGRAM, 0);

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(32000);
    bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr));

    for (;;) {
        len = sizeof(cliaddr);
        n = recvfrom(sockfd, mesg, 1000, 0, (struct sockaddr*)&cliaddr, &len);
        sendto(sockfd, mesg, n, 0, (struct sockaddr*)&cliaddr, sizeof(cliaddr));
        printf("-------------------------------------------------------\n");
        mesg[n] = 0;
        printf("Received the following:\n");
        printf("%s", mesg);
        printf("-------------------------------------------------------\n");
    }

    return 0; 
}
