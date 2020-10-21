#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#include "control_flag.h"

#define     _in             const &
#define     _out            &
#define     LISTEN_QUEUE    20

int create_socket(std::string _in ServerIP,
                  int _in ServerPort,
                  int _out server_socket,
                  int _out client_socket);

void close_socket(int _in server_socket, int _in client_socket);

void TanslateToJointStateString( std::vector<std::string> _in name,
                                 std::vector<double> _in position,                               
                                 std::string _in node_check,
                                 float _in pressure,
                                 std::string _in mobot_mode,
                                 std::string _in vehicle_mode,
                                 float _in Distance,
                                 std::string _out output);


int send_message( int _in client_socket, std::string _in CommandString );

