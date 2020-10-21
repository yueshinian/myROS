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
#define SERVER_THREAD_LOOP  50
//real_time display



// variables in ServerSocket.cpp
#define     _in             const &
#define     SERVER_IP       "192.168.1.100"
#define     SERVER_PORT     5000
#define     LISTEN_QUEUE    20
#define		JOINT1_ARM_NAME         "joint_turret_rotate"
#define		JOINT2_ARM_NAME         "joint_bigarm_pitch"
#define		JOINT3_ARM_NAME         "joint_forearm_pitch"
#define		JOINT4_ARM_NAME         "joint_forearm_rotate"
#define		JOINT5_ARM_NAME         "joint_wrist_pitch"
#define		JOINT6_ARM_NAME         "joint_wrist_rotate"
#define		JOINT7_ARM_NAME         "joint_rod_left1"
#define		JOINT8_ARM_NAME         "joint_swingarm_left"
#define		JOINT9_ARM_NAME         "joint_swingarm_right"
#define		JOINT10_ARM_NAME        "base_l_wheel_joint"
#define		JOINT11_ARM_NAME        "base_r_wheel_joint"
std::string     JOINT_NAME[] = { JOINT1_ARM_NAME, JOINT2_ARM_NAME, JOINT3_ARM_NAME,
                                 JOINT4_ARM_NAME,JOINT5_ARM_NAME, JOINT6_ARM_NAME,
                                 JOINT7_ARM_NAME, JOINT8_ARM_NAME,JOINT9_ARM_NAME,
                                 JOINT10_ARM_NAME,JOINT11_ARM_NAME};


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
            while(!(HeartbeatFlag==1&&heartbeatrecv_flag==1&&hrt_recv_num>=7&&hrt_send_num>=7))
            {

            }
            if(HeartbeatFlag==1&&heartbeatrecv_flag==1&&hrt_recv_num>=7&&hrt_send_num>=7)
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
            error_num=0;
        while(HeartbeatFlag==1&&heartbeatrecv_flag==1&&hrt_recv_num>=7&&hrt_send_num>=7)
        {
            struct timeval tv;
            struct timezone tz;
            gettimeofday(&tv, &tz);
            double start = (double)(tv.tv_sec)*1000.0 + (double)(tv.tv_usec)/1000.0;
            TanslateToJointStateString(position,check_node,
                                       Pressure,mobot_mode,
                                       vehicle_mode,Distance,odom,
                                       interface_states_nuclear,seq,
                                       intensity_max,rotation_max,point1_seq,
                                       point1_rotation,point1_forward_intensity,
                                       point1_inverse_intensity,point2_seq,
                                       point2_rotation,point2_forward_intensity,
                                       point2_inverse_intensity,nav_x,nav_y,nav_z,nav_theta,
				       p1_x,p1_y,p1_z,p1_theta,Joint_state);
            int resp = send_message(mClientSocket,Joint_state);
            Distance = 0.0;
            if (resp < 0)
            {

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
            else
                std::cout<<"[Over Time] control loop over time: "<<(end-start-SERVER_THREAD_LOOP)<<"ms."<<std::endl;

        }
        // close the server_socket and client_socket
        close_socket(mServerSocket, mClientSocket);
        sleep(2);
        std::cout<<"Get out of the run process loop."<<std::endl;
        }
    }

public:
    int mServerSocket;
    int mClientSocket;
    //QMutex pause;
};


#endif // SERVERTHREAD_H
