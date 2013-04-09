#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include "Socket.hpp"

Socket::Socket(void) {
   
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(32000);
    bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr));

    for (;;) {
        len = sizeof(cliaddr);
        n = recvfrom(sockfd, mesg, 1000, 0, (struct sockaddr*)&cliaddr, &len);
        sendto(sockfd, mesg, n, 0, (struct sockaddr*)&cliaddr, sizeof(cliaddr));
        //send back boolean to determine if we are over the other drone
        printf("-------------------------------------------------------\n");
        mesg[n] = 0;
        printf("Received the following:\n");
        printf("%s", mesg);
        printf("-------------------------------------------------------\n");
    }
}
