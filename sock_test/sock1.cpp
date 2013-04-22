#include "sock1.hpp"

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


    sock1* s1 = new sock1(); 
    struct Msg_Cmd cmd;
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
    
    for (;;) {
        unsigned char buf[100];
        struct sockaddr_in incoming; 
        unsigned int socklen = sizeof(incoming); 
        n = recvfrom(sockfd, &buf, 100, 0, (struct sockaddr*)&incoming, &socklen);
        if (n < 0) {
            perror("recvfrom"); 
            exit(1); 
        }
        buf[n] = 0; 
        printf("\n-------------------------------------------------------\n");
        printf("Sock1 Received:\n");
        cmd = s1->unpack(buf); 
        printf("x: %d, y: %d, z: %d, id: %d", cmd.x, cmd.y, cmd.z, cmd.id);
        printf("\n-------------------------------------------------------\n");
        msg = s1->pack(0, 1); 
        sendto(sockfd, msg, 8, 0, (struct sockaddr*)&cliaddr, sizeof(cliaddr));
        printf("Sock1 Sent back a message"); 
    }

    return 0; 
}

/* 
 * Unpack the message we receive. Parse it into a message
 * command to later translate into a waypoint service call.
 */
Msg_Cmd sock1::unpack(unsigned char* bytes) {
    Msg_Cmd res;
    res.x = bytes[3] << 24  | bytes[2] << 16  | bytes[1] << 8 | bytes[0];
    res.y = bytes[7] << 24  | bytes[6] << 16  | bytes[5] << 8 | bytes[4];
    res.z = bytes[11] << 24 | bytes[10] << 16 | bytes[9] << 8 | bytes[8];
    res.id = bytes[15] << 24 | bytes[14] << 16 | bytes[13] << 8 | bytes[12];
    return res;
}

/* 
 * Pack the return message.
 */
unsigned char* sock1::pack(int x, int y) {
    unsigned char bytes[16];
    bytes[3]  = (x & 0xff000000) >> 24;
    bytes[2]  = (x & 0xff0000)   >> 16;
    bytes[1]  = (x & 0xff00)     >> 8;
    bytes[0]  = (x & 0xff);
    bytes[7]  = (y & 0xff000000) >> 24;
    bytes[6]  = (y & 0xff0000)   >> 16;
    bytes[5]  = (y & 0xff00)     >> 8;
    bytes[4]  = (y & 0xff);

    return bytes;
}

