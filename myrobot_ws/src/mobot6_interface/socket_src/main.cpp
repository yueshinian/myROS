/* Author: Hongbiao */

#include <ros/ros.h>
#include "../include/mobot6_interface/heartbeat_recv_thread.h"
#include "../include/mobot6_interface/heartbeat_send_thread.h"
#include "../include/mobot6_interface/Client_Thread.h"
#include <QApplication>
#include "moc_heartbeat_recv_thread.cpp"
#include "moc_heartbeat_send_thread.cpp"
#include "moc_Client_Thread.cpp"
#include "../include/mobot6_interface/socket_node.hpp"

bool HeartbeatFlag = true;
signed int recv_flag =0;
signed int recv_done_flag=0;
signed int heartbeatrecv_flag = 0;
signed int hrt_recv_num=0;
signed int hrt_send_num=0;
signed int second = 0;
pthread_t recv_id;
pthread_t send_id;
int hrt_send_error_count=0;
std::string CommandString;
std::vector<double>  joints(6,0.0);
std::vector<double> cartesians(6,0.0);
std::vector<double> moves(4,0.0);
std::vector<double> swings(2,0.0);
std::vector<double>  lspb(61,0.0);
std::vector<double>  nuclear(3,0.0);
double grippers = 0.0;
int frameID = 7;
int commandid = 8;
int laser_flag=0;


HeartbeatRecvThread *heartbeat_thread_recv;
HeartbeatSendThread *heartbeat_thread_send;
ClientThread        *Client_thread;


int main(int argc, char** argv)
{
  QApplication a(argc, argv);
  heartbeat_thread_recv = new HeartbeatRecvThread();
  heartbeat_thread_recv->start();
  heartbeat_thread_send = new HeartbeatSendThread();
  heartbeat_thread_send->start();
  Client_thread = new ClientThread();
  Client_thread->start();
  //Server_thread = new ServerThread();
  //Server_thread->start();
  QNode   qnode(argc,argv);
  qnode.init();
  return a.exec();
}

