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
#include <stdio.h>

using std::cout;
using std::cin;
using std::endl;

#define BUFLEN 64
#define LOCALPORTBASE 5050

int main(int argc, char **argv) {
    if(argc != 4) {
        cout << "usage: " << argv[0];
        cout << " <remote-ip> <remote-port> <id>" << endl;
        exit(1);
    }

    char *host = argv[1];
    char *port = argv[2];
    struct addrinfo *srv, *cli;
    struct addrinfo hints;

    int id = atoi(argv[3]);
    char localport[5];
    sprintf(localport, "%d", LOCALPORTBASE + id);

    // socket parameters
    bzero(&hints, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = 0;
    hints.ai_protocol = IPPROTO_UDP;

    // address resolution
    getaddrinfo(host, port, &hints, &srv);
    getaddrinfo("localhost", localport, &hints, &cli);

    // socket
    int s;
    if((s = socket(cli->ai_family, cli->ai_socktype, cli->ai_protocol)) < 0) {
       cout << "error creating socket" << endl;
       exit(1);
    }

    // bind
    if(bind(s, cli->ai_addr, cli->ai_addrlen) < 0) {
        cout << "error connecting to port" << endl;
        exit(1);
    }

    // feedback
    cout << "listening on UDP port " << localport << endl;
    int nameflags = NI_NUMERICHOST | NI_NUMERICSERV;
    char rhost[64], rserv[64];
    int salen = sizeof(struct sockaddr);
    getnameinfo(srv->ai_addr, salen, rhost, 64, rserv, 64, nameflags);
    cout << "sending obs to " << rhost << " on UDP port " << rserv << endl;

    // read actions for socket, execute, and return observations
    while(true) {
        // read action
        char rbuf[BUFLEN], sbuf[BUFLEN];
        int x, y, z;
        cout << "x: ";
        scanf("%d", &x); sbuf[0] = x;
        cout << "y: ";
        scanf("%d", &y); sbuf[1] = y;
        cout << "z: ";
        scanf("%d", &z); sbuf[2] = z;
        if(sendto(s, sbuf, 3, 0, srv->ai_addr, srv->ai_addrlen) < 0) {
            cout << "error sending to client" << endl;
            exit(1);
        }
        if(recvfrom(s, rbuf, BUFLEN, 0, NULL, NULL) < 0) {
            cout << "error receiving from client" << endl;
            exit(1);
        }
        cout << "observation:" << endl;
        cout << "  x: " << (int) rbuf[0];
        cout << "  y: " << (int) rbuf[1];
        cout << "  z: " << (int) rbuf[2];
        cout << endl;
    }

    freeaddrinfo(srv);
    freeaddrinfo(cli);
    close(s);
    return 0;
}
