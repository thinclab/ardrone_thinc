#include "sock2.hpp"

// test file to control the drones

int main(int argc, char *argv[]) {
    if (argc != 4) {
        fprintf(stderr, "requires local port, remote ip, and remote port\n");
        exit(1); 
    }

    int port_number = atol(argv[1]); //local port to run on
    char* remote = argv[2]; //remote ip address
    int remote_port = atol(argv[3]); //remote port to run on

    hostent * record = gethostbyname(remote);
    if (record == NULL) { 
        herror("gethostbyname failed"); 
        exit(1); 
    }
    in_addr * addressptr = (in_addr *) record->h_addr;

    sock2* s2 = new sock2(); 
//    struct Msg_Cmd cmd;
    int *response;
    int sockfd;
    int n;
    struct sockaddr_in servaddr;
    struct sockaddr_in cliaddr;
    socklen_t len;
    unsigned char* msg;

    sockfd=socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket creation"); 
        exit(1); 
    }

    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(port_number);

    int b = bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr));
    if (b < 0) {
        perror("bind"); 
        exit(1); 
    }

    cliaddr.sin_family = AF_INET; 
    cliaddr.sin_addr = *addressptr; 
    cliaddr.sin_port = htons(remote_port); 

    unsigned char buf[1024];

    for (int i = 0; i < 5; i++) {
        //msg = s2->pack(4, 3, 2, 1); 
        switch (i) {
            case 0:
                msg = s2->pack(1, 1, 1, 0);
                break; 
            case 1: 
                msg = s2->pack(9, 9, 1, 1); 
                break; 
            case 2: 
                msg = s2->pack(5, 5, 1, 0); 
                break; 
            case 3: 
                msg = s2->pack(3, 6, 1, 1); 
                break; 
            case 4:
                msg = s2->pack(8, 2, 1, 0); 
                break; 
        }

        int s = sendto(sockfd, msg, 17, 0, (struct sockaddr*)&cliaddr, sizeof(cliaddr));
        if (s < 0) {
            perror("sendto"); 
            exit(1); 
        }
        printf("-------------------------------------------------------\n");
//        msg[n] = 0;
        printf("Sock2 sent a message");
//        printf("%s", msg);
        printf("\n-------------------------------------------------------\n");

        struct sockaddr_in incoming; 
        len = sizeof(incoming);


        n = recvfrom(sockfd, buf, sizeof(buf)-1, 0, (struct sockaddr*)&incoming, &len);
        if (n < 0) {
            perror("recvfrom"); 
            exit(1);
        }

//        buf[n] = 0; 
        response = s2->unpack(buf);
        printf("Sock2 Received: \n"); 
        printf("success: %d, observation: %d", response[0], response[1]);
   
    }

    return 0; 
}

/* 
 * Unpack the message we receive. Parse it into a message
 * command to later translate into a waypoint service call.
 */
int* sock2::unpack(unsigned char* bytes) {
    int res[2];
    res[0] = bytes[3] << 24  | bytes[2] << 16  | bytes[1] << 8 | bytes[0];
    res[1] = bytes[7] << 24  | bytes[6] << 16  | bytes[5] << 8 | bytes[4];
    return res;
}

/* 
 * Pack the return message.
 */
unsigned char* sock2::pack(int x, int y, int z, int id) {
    unsigned char bytes[16];
    bytes[3]  = (x & 0xff000000) >> 24;
    bytes[2]  = (x & 0xff0000)   >> 16;
    bytes[1]  = (x & 0xff00)     >> 8;
    bytes[0]  = (x & 0xff);
    bytes[7]  = (y & 0xff000000) >> 24;
    bytes[6]  = (y & 0xff0000)   >> 16;
    bytes[5]  = (y & 0xff00)     >> 8;
    bytes[4]  = (y & 0xff);
    bytes[11]  = (z & 0xff000000) >> 24;
    bytes[10]  = (z & 0xff0000)   >> 16;
    bytes[9]  = (z & 0xff00)     >> 8;
    bytes[8]  = (z & 0xff);
    bytes[15]  = (id & 0xff000000) >> 24;
    bytes[14]  = (id & 0xff0000)   >> 16;
    bytes[13]  = (id & 0xff00)     >> 8;
    bytes[12]  = (id & 0xff);
    return bytes;
}

