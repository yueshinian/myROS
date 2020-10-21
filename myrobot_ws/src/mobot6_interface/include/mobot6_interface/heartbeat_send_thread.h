#ifndef HEARTBEAT_SEND_THREAD_H
#define HEARTBEAT_SEND_THREAD_H

#include <QThread>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>

#include "heartbeat_send_socket.h"

#define hrt_send_IP "192.168.1.2"



class HeartbeatSendThread: public QThread
{
    Q_OBJECT
public:
    HeartbeatSendThread()
    {

    }
    ~HeartbeatSendThread()
    {

    }
    void run()
    {
        hrt_create_socket(hrt_send_IP,8000,Client_socket);
	hrt_send_num=0;
	hrt_send_error_count=0;
        while(1)
        {
            if(HeartbeatFlag == false)
            {
                cout << "HeartbeatFlag is" << HeartbeatFlag << endl;
                cout << "000" << endl;
                hrt_send_close_fd(Client_socket);
                sleep(2);
		hrt_send_error_count=0;
                while(1)
                {
                    hrt_create_socket(hrt_send_IP,8000,Client_socket);
                    HeartbeatFlag = true;
		    hrt_send_num=0;
                    while(1)
                    {
                        hrt_send_send(Client_socket);
			hrt_send_num++;
			if(hrt_send_num>4)
				hrt_send_num=7;
		          cout<<"hrt_send_num"<<hrt_send_num<<endl;
                        if(HeartbeatFlag == false)
                        {
                            break;
			    hrt_send_num=0;
                        }
                    }
                    break;
                }
            }
            else
	      {
              hrt_send_send(Client_socket);
	      hrt_send_num++;
			if(hrt_send_num>4)
				hrt_send_num=7;
	      }
        }
    }
public:
    int Client_socket;
};

#endif // HEARTBEAT_SEND_THREAD_H
