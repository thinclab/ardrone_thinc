#ifndef SOCKET_HPP_
#define SOCKET_HPP_

#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>

class Socket {

    public:
        Socket(void); 
        int sockfd; 
        int n; 
        struct sockaddr_in servaddr;
        struct sockaddr_in cliaddr; 
        socklen_t len; 
        char mesg[1000];    
};

#endif
