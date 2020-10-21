#ifndef CLIENT_THREAD_H
#define CLIENT_THREAD_H

#include <QThread>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>

#include <vector>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "control_flag.h"
#include "Revmsgs_Socket.h"

// variables in ServerSocket.cpp
#define     _in             const &
#define     CLIENT_IP       "192.168.1.2"
#define     CLIENT_PORT     4000

class ClientThread: public QThread
{
    Q_OBJECT
public:

    ClientThread()
    {


    }

    ~ClientThread()
    {

    }

    void run()
    {
        int error_num;
        int Not_send  = 0;
        std::string COMMAND;
        std::string AUXCMD;
        std::vector<double> VALUES;
        while(1)
        {
            while(!(HeartbeatFlag==true&&heartbeatrecv_flag==1&&hrt_recv_num>=7&&hrt_send_num>=7))
	    {}
	    if(HeartbeatFlag==true&&heartbeatrecv_flag==1&&hrt_recv_num>=7&&hrt_send_num>=7)
            {
            int rep = create_socket(CLIENT_IP, CLIENT_PORT, nClientSocket);
            Not_send=0;
            }
            error_num=0;
            while(HeartbeatFlag==true&&heartbeatrecv_flag==1&&recv_flag>=0&&hrt_recv_num>=7&&hrt_send_num>=7)
            {
              receive_msgs(nClientSocket);
              recv_done_flag = 1;
                /*for (size_t i = 0; i < JOINT_DOF; ++i)
                  joints[i] = 0.0;
                for (size_t i = 0; i < CARTESIAN_DOF; ++i)
                  cartesians[i] = 0.0;
                for (size_t i = 0; i < BASE_DOF; ++i)
                  moves[i] = 0.0;
                for (size_t i = 0; i < SWING_DOF; ++i)
                  swings[i] = 0.0;
                lspb.resize(61,0.0);
                grippers  = 0.0;
                commandid = 8;*/
                        // resolve the command string
                int resp = ResolveCommandString( CommandString, COMMAND, AUXCMD, VALUES );
                if ( resp <= 0 )
                {
                      ROS_ERROR( "can not resolve the command string" );
                      Not_send=1;
                      error_num++;
                      if(error_num>100)
                          break;
                      continue;
                }
                else
                      error_num=0;
                        // update the corresponding massage accoording to the command string
                if ( COMMAND == "STOP" )
                {
                      commandid = 0;
                }
                else if ( COMMAND == "MOVE" )
                {
                      commandid = 1;
                }
                else if ( COMMAND == "ARMJ" )
                {
                      commandid = 2;
                      for (size_t i = 0; i < VALUES.size(); i++)
                             joints[i] = VALUES[i];
                      int resp_frame;
                      MatchFrame( AUXCMD, MATCH_MODE_ARM, resp_frame );
                      if ( resp_frame < 0 )
                      {
                          ROS_ERROR( "can not match the frame string" );
                          Not_send=1;
                          continue;
                      }
                      frameID = resp_frame;
                 }
                 else if (COMMAND == "ARMC")
                 {
                      commandid = 3;
                      for (size_t i = 0; i < VALUES.size(); i++)
                          cartesians[i] = VALUES[i];

                          int resp_frame;
                          MatchFrame( AUXCMD, MATCH_MODE_ARM, resp_frame );
                          if ( resp_frame < 0 )
                          {
                                ROS_ERROR( "can not match the frame string" );
                                Not_send=1;
                                continue;
                          }
                      frameID = resp_frame;
                   }
                   else if ( COMMAND == "BASE" )
                   {
                      commandid = 4;
                      for (size_t i = 0; i < VALUES.size(); ++i)
                           moves[i] = VALUES[i];
                   }
                   else if ( COMMAND == "SWING" )
                   {
                      commandid = 5;
                      for (size_t i = 0; i < VALUES.size(); ++i)
                           swings[i] = VALUES[i];

                      int swing_mode;
                      MatchFrame( AUXCMD, MATCH_MODE_SWING, swing_mode );
                      if ( swing_mode < 0 )
                           {
                               ROS_ERROR( "can not match the swing_mode string, %s", AUXCMD.c_str() );
                               Not_send=1;
                               continue;
                           }

                      switch (swing_mode)
                      {
                           case 1:   // "SWINGHOME"
                           {
                               for (size_t i = 0; i < VALUES.size(); ++i)
                                     swings[i] = 6.0;
                               break;
                           }
                           case 2:   // "SWINGHOMESTOP"
                           {
                               for (size_t i = 0; i < VALUES.size(); ++i)
                                      swings[i] = 7.0;
                               break;
                           }
                           case 3:   // "SWINGRESET"
                           {
                               for (size_t i = 0; i < VALUES.size(); ++i)
                                      swings[i] = 8.0;
                               break;
                           }
                           case 0:
                           default:
                           {
                               break;
                           }
                      }
                   }
                   else if ( COMMAND == "GRIP" )
                   {
                            commandid = 6;
                            grippers = VALUES[0];
                   }
                   else if ( COMMAND == "HOME" )
                   {
                            commandid = 9;
                            for (size_t i = 0; i < VALUES.size(); ++i)
                                joints[i] = VALUES[i];

                            int resp_frame;
                            MatchFrame( AUXCMD, MATCH_MODE_HOME, resp_frame );
                            if ( resp_frame < 0 )
                            {
                                ROS_ERROR( "can not match the HOME string" );
                                Not_send=1;
                                continue;
                            }
                            frameID = resp_frame;
                   }
                   else if ( COMMAND == "HOMESTOP" )
                   {
                            commandid = 10;
                   }
                   else if ( COMMAND == "NO_LIMIT" )
                   {
                            commandid = 11;
                   }
                   else if ( COMMAND == "SPLINE" )
                   {
                            commandid = 12;
                   }
                   else if ( COMMAND == "MOVJ_LSPB" )
                   {
                            commandid = 13;
                      lspb.resize(VALUES.size(),0.0);
                      for (size_t i = 0; i < VALUES.size(); i++)
                                lspb[i] = VALUES[i];

                   }
                   else if ( COMMAND == "MCHI_LSPB" )
                   {
                            commandid = 14;
                      lspb.resize(VALUES.size(),0.0);
                      for (size_t i = 0; i < VALUES.size(); i++)
                                lspb[i] = VALUES[i];
                   }
                   else if ( COMMAND == "NUCLEAR" )
                   {
                       commandid = 16;
                       for (size_t i = 0; i < 3; i++)
                             nuclear[i] = VALUES[i];
                   }
                   else if ( COMMAND == "LASER" )
                   {
                       laser_flag=1;
                   }
                   else
                   {
                            for (size_t i = 0; i < JOINT_DOF; ++i)
                                joints[i] = 0.0;
                            for (size_t i = 0; i < CARTESIAN_DOF; ++i)
                                cartesians[i] = 0.0;
                            for (size_t i = 0; i < BASE_DOF; ++i)
                                moves[i] = 0.0;
                            for (size_t i = 0; i < SWING_DOF; ++i)
                                swings[i] = 0.0;
                            for (size_t i = 0; i < 3; ++i)
                                nuclear[i] = 0.0;
                            grippers  = 0.0;
                            commandid = 8;
                   }
            }
            std::cout<<"1"<<std::endl;
            sleep(4);
            close_socket( nClientSocket );
        }
    }

private:

    int nClientSocket;
};

#endif // CLIENT_THREAD_H
