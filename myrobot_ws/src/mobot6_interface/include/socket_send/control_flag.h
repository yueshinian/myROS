#ifndef CONTROL_FLAG_H
#define CONTROL_FLAG_H

#include <iostream>
#include <stdio.h>
#include <sys/time.h>

#include <vector>
#include <string.h>
#include <stdlib.h>
#include "nav_msgs/Odometry.h"

extern signed int HeartbeatFlag;
extern signed int heartbeatrecv_flag;
extern signed int hrt_recv_num;
extern signed int hrt_send_num;
extern int hrt_send_error_count;
extern std::string CommandString;
extern std::vector<std::string> names;
extern std::vector<double> position;
extern std::string Joint_state;
extern std::string check_node;
extern float Pressure;
extern std::string mobot_mode;
extern std::string vehicle_mode;
extern std::string interface_states_nuclear;
extern int seq;
extern int point1_seq;
extern int point2_seq;
extern float intensity_max;
extern float point1_forward_intensity;
extern float point1_inverse_intensity;
extern float point2_forward_intensity;
extern float point2_inverse_intensity;
extern float rotation_max;
extern float point1_rotation;
extern float point2_rotation;
extern float Distance;
extern double odom_position_x;
extern double odom_position_y;
extern double odom_twist_linear_x;
extern double odom_twist_angular_z;
extern double odom_orientation_x;
extern double odom_orientation_y;
extern double odom_orientation_z;
extern double odom_orientation_w;
extern nav_msgs::Odometry odom;
extern float nav_x ;
extern float nav_y ;
extern float nav_z ;
extern float nav_theta ;
extern float p1_x ;
extern float p1_y ;
extern float p1_z ;
extern float p1_theta ;

#endif // CONTROL_FLAG_H
