#ifndef HEARTBEAT_RECV_SOCKET_H
#define HEARTBEAT_RECV_SOCKET_H

#include <QThread>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <cstdlib>
#include <cstdio>
#include <cstring>

#include <stdlib.h>
#include <stdio.h>


#include "control_flag.h"
#define     _in             const &
#define     _out            &
#define     LISTEN_QUEUE    20

using namespace std;

enum Type_Recv {HEART, OTHER};

struct PACKET_HEAD_RECV
{
    Type_Recv type_recv;
    int length_recv;
};

void* heart_handler(void* arg);

int create_heart_recv_socket(std::string _in ServerIP,
                  int _in ServerPort,
                  int _out server_socket,
                  int _out client_socket);

void close_recv_socket(int _in listen_fd, int _in new_fd);
void Recv( int _in new_fd);
void Run();



#endif // HEARTBEAT_RECV_SOCKET_H
