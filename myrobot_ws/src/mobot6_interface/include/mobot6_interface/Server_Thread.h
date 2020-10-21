#ifndef SERVERTHREAD_H
#define SERVERTHREAD_H
#include <QThread>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include "Sendmsgs_Socket.h"


// variables in mainwindow.cpp
#define SERVER_THREAD_LOOP  40
//real_time display



// variables in ServerSocket.cpp
#define     _in             const &
#define     SERVER_IP       "192.168.1.100"
#define     SERVER_PORT     5000
#define     LISTEN_QUEUE    20



class ServerThread : public QThread
{
    Q_OBJECT
public:
    ServerThread()
    {

    }
    void run()
    {
	int error_num=0;

        while(1)
        {
            while(!(HeartbeatFlag==true&&heartbeatrecv_flag==1&&hrt_recv_num>=7&&hrt_send_num>=7))
	    {}
	    if(HeartbeatFlag==true&&heartbeatrecv_flag==1&&hrt_recv_num>=7&&hrt_send_num>=7)
            {
                // 2. create the server socket
                int resp = create_socket(SERVER_IP, SERVER_PORT, mServerSocket, mClientSocket);
                while ( resp < 0 )
                {
                    std::cout << "Create socket failed, retry after 1 seconds." << std::endl;
                    sleep(1);
                    resp = create_socket(SERVER_IP, SERVER_PORT, mServerSocket, mClientSocket);
                }
	     

            }
        while(HeartbeatFlag==true&&heartbeatrecv_flag==1&&hrt_recv_num>=7&&hrt_send_num>=7)
        {
            struct timeval tv;
            struct timezone tz;
            gettimeofday(&tv, &tz);
            double start = (double)(tv.tv_sec)*1000.0 + (double)(tv.tv_usec)/1000.0;
            TanslateToJointStateString(names,position,check_node,
                                       Pressure,mobot_mode,
                                       vehicle_mode,Distance,Joint_state);
            int resp = send_message(mClientSocket,Joint_state);
	    Distance = 0.0;
            if (resp < 0)
            {
                    std::cout << "[SOCKET_ERROR] send_message: cannot send command string." << std::endl;
		    error_num++;
		    if(error_num>10)
		    {
			break;
		    }
            }
            //  get the end time in msecs, control the sleep time (default: 500ms)
            gettimeofday(&tv, &tz);
            double end = (double)(tv.tv_sec)*1000.0 + (double)(tv.tv_usec)/1000.0;
            if ( (end-start) < SERVER_THREAD_LOOP )
            {
                msleep( SERVER_THREAD_LOOP-(end-start) );
                //std::cout<<"control loop time: "<<(end-start)<<"ms."<<std::endl;
            }
            //else
                //std::cout<<"[Over Time] control loop over time: "<<(end-start-SERVER_THREAD_LOOP)<<"ms."<<std::endl;

        }
        // close the server_socket and client_socket
        close_socket(mServerSocket, mClientSocket);
        sleep(4);
        std::cout<<"Get out of the run process loop."<<std::endl;
        }
    }

public:
    int mServerSocket;
    int mClientSocket;
    //QMutex pause;
};


#endif // SERVERTHREAD_H
