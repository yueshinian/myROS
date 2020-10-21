#ifndef HEARTBEAT_SEND_SOCKET_H
#define HEARTBEAT_SEND_SOCKET_H

#include<netinet/in.h>   // sockaddr_in
#include<sys/types.h>    // socket
#include<sys/socket.h>   // socket
#include<arpa/inet.h>
#include<sys/ioctl.h>
#include<unistd.h>
#include<iostream>
#include<string>
#include<cstdlib>
#include<cstdio>
#include<cstring>
using namespace std;

#include "control_flag.h"
#include <sys/time.h>
#define     _in             const &
#define     _out            &
enum Type_Send {HEART_HEART, OTHER_OTHER};

struct PACKET_HEAD_SEND
{
    Type_Send type_send;
    int length_send;
};

//void* send_heart(void* arg);


void hrt_create_socket(std::string _in ClientIP,
                   int _in ClientPort,
                   int _out client_socket);
void hrt_send_send(int _in client_socket);
void hrt_send_close_fd(int _in Client_socket);

#endif // HEARTBEAT_SEND_H
