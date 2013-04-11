/*//ros
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"
#include "ardrone_autonomy/Navdata.h"

//sockets
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>*/

#include "Socket.hpp"

int main(int argc, char**argv) {
    ros::init(argc, argv, "rocket_sockets");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    
    Socket* sock = new Socket(); 
    struct Msg_Cmd cmd; 
    //test the Parse_Msg function
    char* test[3] = {"0", "8", "3"}; 
    cmd = sock->Parse_Msg(test);
    cout << "id: " << cmd.id << endl;  
    cout << "x: " << cmd.x << endl; 
    cout << "y: " << cmd.y << endl; 

    int sockfd;
    int n;
    struct sockaddr_in servaddr;
    struct sockaddr_in cliaddr;
    socklen_t len;
    char* msg[1000]; //char* msg[] or char msg[] ?

    sockfd=socket(AF_INET, SOCK_DGRAM, 0);

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(32000);
    bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr));

    for (;;) {
        len = sizeof(cliaddr);
        n = recvfrom(sockfd, msg, 1000, 0, (struct sockaddr*)&cliaddr, &len);
        /*cmd = sock.Parse_Msg(msg);*/
        sendto(sockfd, msg, n, 0, (struct sockaddr*)&cliaddr, sizeof(cliaddr));
        printf("-------------------------------------------------------\n");
        msg[n] = 0;
        printf("Received the following:\n");
        printf("%s", msg);
        printf("-------------------------------------------------------\n");
    }

    return 0; 
}

/*
 * Parse the message received to translate into
 * drone movements.
 * Expected message format: idxy where the 
 * first number indicates drone id, followed by 
 * the numerical x coordinate, and lastly the  
 * numerical y coordinate.
 */
Msg_Cmd Socket::Parse_Msg(char* msg[]) {
    struct Msg_Cmd cmd; 
    int id, x, y; 

    id = atoi(msg[0]); 
    x = atoi(msg[1]); 
    y = atoi(msg[2]); 
 
    cmd.id = id; 
    cmd.x = x; 
    cmd.y = y; 

    return cmd; 
}
