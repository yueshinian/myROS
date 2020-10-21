#ifndef CONTROL_FLAG_H
#define CONTROL_FLAG_H

#include <iostream>
#include <stdio.h>
#include <sys/time.h>

#include <vector>
#include <string.h>
#include <stdlib.h>

extern bool HeartbeatFlag;
extern signed int recv_flag;
extern signed int recv_done_flag;
extern signed int heartbeatrecv_flag;
extern signed int hrt_recv_num;
extern signed int hrt_send_num;
extern signed int second;
extern pthread_t recv_id;
extern pthread_t send_id;
extern int hrt_send_error_count;
extern std::string CommandString;
extern std::vector<double>  joints;
extern std::vector<double> cartesians;
extern std::vector<double> moves;
extern std::vector<double> swings;
extern std::vector<double>  lspb;
extern std::vector<double>  nuclear;
extern double grippers;
extern int frameID;
extern int commandid;
extern std::vector<std::string> names;
extern std::vector<double> position;
extern std::string Joint_state;
extern std::string check_node;
extern float Pressure;
extern std::string mobot_mode;
extern std::string vehicle_mode;
extern int laser_flag;
extern float Distance;

#endif // CONTROL_FLAG_H
