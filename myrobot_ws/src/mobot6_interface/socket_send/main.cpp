/* Author: Hongbiao */

#include <ros/ros.h>
#include "../include/socket_send/Server_Thread.h"
#include <QApplication>
#include "moc_Server_Thread.cpp"
#include "../include/socket_send/socket_node.hpp"
#include "nav_msgs/Odometry.h"

signed int HeartbeatFlag = 0;
signed int heartbeatrecv_flag = 0;
signed int hrt_recv_num=0;
signed int hrt_send_num=0;
std::string CommandString;
std::vector<std::string> names;
std::vector<double> position(11,0.0);
std::string Joint_state = "Null";
std::string check_node = "Null";
float Pressure = 0.0;
std::string mobot_mode = "Null";
std::string vehicle_mode = "Null";
std::string interface_states_nuclear;
int seq=0;
int point1_seq=0;
int point2_seq=0;
float intensity_max=0;
float point1_forward_intensity=0;
float point1_inverse_intensity=0;
float point2_forward_intensity=0;
float point2_inverse_intensity=0;
float rotation_max=0;
float point1_rotation=0;
float point2_rotation=0;
float Distance=0.0;
nav_msgs::Odometry odom;
float nav_x =0;
float nav_y =0;
float nav_z =0;
float nav_theta =0;
float p1_x =0;
float p1_y =0;
float p1_z =0;
float p1_theta =0;
/*double odom_position_x = 0.0;
double odom_position_y = 0.0;
double odom_twist_linear_x = 0.0;
double odom_twist_angular_z = 0.0;
double odom_orientation_x = 0.0;
double odom_orientation_y = 0.0;
double odom_orientation_z = 0.0;
double odom_orientation_w = 0.0;*/



ServerThread        *Server_thread;


int main(int argc, char** argv)
{
  QApplication a(argc, argv);
  //heartbeat_thread_recv = new HeartbeatRecvThread();
 // heartbeat_thread_recv->start();
  //heartbeat_thread_send = new HeartbeatSendThread();
  //heartbeat_thread_send->start();
  //Client_thread = new ClientThread();
  //Client_thread->start();
  Server_thread = new ServerThread();
  Server_thread->start();
  QNode   qnode(argc,argv);
  qnode.init();
  return a.exec();
}

