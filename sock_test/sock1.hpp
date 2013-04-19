#ifndef SOCK1_H_
#define SOCK1_H_

//sockets
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <netdb.h>
//#include <sys/types.h>
//#include <arpa/inet.h>

using namespace std;


struct Msg_Cmd {
    int x;
    int y;
    int z;
    int id;
};

class sock1 {
    public: 
        Msg_Cmd unpack(unsigned char*);
        unsigned char* pack(int, int, int, int);
};

#endif
