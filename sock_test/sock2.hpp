#ifndef SOCK2_H_
#define SOCK2_H_

//sockets
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <netdb.h>

using namespace std;


struct Msg_Cmd {
    int x;
    int y;
    int z;
    int id;
};

class sock2 {
    public: 
        int* unpack(unsigned char*);
        unsigned char* pack(int, int, int, int);
};

#endif
