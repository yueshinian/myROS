#ifndef HEARTBEAT_RECV_THREAD_H
#define HEARTBEAT_RECV_THREAD_H

#include "heartbeat_recv_socket.h"
#include <ros/ros.h>

#define heart_recv_IP "192.168.1.100"

class HeartbeatRecvThread: public QThread
{
    Q_OBJECT
public:
    HeartbeatRecvThread()
    {

    }

    ~HeartbeatRecvThread()
    {

    }

    void run()
    {

        create_heart_recv_socket(heart_recv_IP,15000,listen_fd,new_fd);
        Run();
	hrt_recv_num=0;
        while(1)
        {
          if(heartbeatrecv_flag==0)
          {
              pthread_join(recv_id,NULL);//退出线程,释放所占用的资源
              cout << "111" << endl;
              close_recv_socket(listen_fd,new_fd);
              sleep(2);

              while(1)
              {
                  create_heart_recv_socket(heart_recv_IP,15000,listen_fd,new_fd);
                  cout<<"new_Heartbeat_flag="<<HeartbeatFlag<<endl;
                  Run();
		  hrt_recv_num=0;
                  while(1)
                  {
		      hrt_recv_num++;
		      if(hrt_recv_num>4)
			  hrt_recv_num=7;
		          cout<<"hrt_recv_num"<<hrt_recv_num<<endl;
                      if(heartbeatrecv_flag==1)
                      {
                          Recv( new_fd);
                      }
                      else
		      {
			  hrt_recv_num=0;
                          break;
		      }
                  }
                  break;
              }
		hrt_recv_num=0;
          }
          if(heartbeatrecv_flag==1)
	  {
            Recv(new_fd);
	    hrt_recv_num++;
	    if(hrt_recv_num>4)
		hrt_recv_num=7;
	  }
        }


    }
 public:
    int listen_fd;
    int new_fd;

};

#endif // HEARTBEAT_RECV_THREAD_H
