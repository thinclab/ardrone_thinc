#ifndef SOCKET_HPP_
#define SOCKET_HPP_

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
#include <stdlib.h>
#include <string.h>

using namespace std; 

struct Msg_Cmd {
    int id; 
    int x;  
    int y; 
};

class Socket {

    public: 
        Msg_Cmd Parse_Msg(char* msg[]); 

};

#endif
