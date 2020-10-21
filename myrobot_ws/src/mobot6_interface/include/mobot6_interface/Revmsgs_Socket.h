#include <QThread>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include "control_flag.h"

#define     _in             const &
#define     _out            &

#define		COMMAND_ID_0            "STOP"
#define		COMMAND_ID_1            "MOVE"
#define		COMMAND_ID_2            "ARMJ"
#define		COMMAND_ID_3            "ARMC"
#define		COMMAND_ID_4            "BASE"
#define		COMMAND_ID_5            "SWING"
#define		COMMAND_ID_6            "GRIP"
#define		COMMAND_ID_7            "COORD"
#define		COMMAND_ID_8            "OTHERS"
#define         COMMAND_ID_9            "HOME"
#define         COMMAND_ID_10           "HOMESTOP"
#define         COMMAND_ID_11           "NO_LIMIT"
#define         COMMAND_ID_12           "SPLINE"
#define         COMMAND_ID_13           "MOVJ_LSPB"
#define         COMMAND_ID_14           "MCHI_LSPB"
#define         COMMAND_ID_15           "LASER"
#define         COMMAND_ID_16           "NUCEAR"



#define		MATCH_FRAME_0           "BASE_LINK"
#define		MATCH_FRAME_1           "LINK_1"
#define		MATCH_FRAME_2           "LINK_2"
#define		MATCH_FRAME_3           "LINK_3"
#define		MATCH_FRAME_4           "LINK_4"
#define		MATCH_FRAME_5           "LINK_5"
#define		MATCH_FRAME_6           "LINK_6"
#define		MATCH_FRAME_7           "LINK_7"
#define		MATCH_FRAME_8           "END_EFFECTOR"

#define		MATCH_HOME_0            "AXIS1"
#define		MATCH_HOME_1            "AXIS2"
#define		MATCH_HOME_2            "AXIS3"
#define		MATCH_HOME_3            "AXIS4"
#define		MATCH_HOME_4            "AXIS5"
#define		MATCH_HOME_5            "AXIS6"
#define		MATCH_HOME_6            "AXIS7"
#define		MATCH_HOME_7            "ENDEFFECTOR"
#define		MATCH_HOME_8            "AXISALL"

#define         MATCH_SWING_0           "MOVE"
#define         MATCH_SWING_1           "HOME"
#define         MATCH_SWING_2           "HOMESTOP"
#define         MATCH_SWING_3           "RESET"


#define		JOINT_DOF               6
#define		CARTESIAN_DOF           6
#define		BASE_DOF                4
#define		SWING_DOF               2
#define   LOOP_TIME               0.020
const int   MATCH_MODE_ARM   = 1;
const int   MATCH_MODE_HOME  = 2;
const int   MATCH_MODE_SWING = 3;

int create_socket(std::string _in ClientIP,
                   int _in ClientPort,
                   int _out client_socket);


void receive_msgs(int client_socket);
int ResolveCommandString( std::string _in input,
                           std::string _out COMMAND,
                           std::string _out AUXCMD,
                           std::vector<double> _out VALUES );
void MatchFrame( std::string _in FRAME, int _in MODE, int _out ID );
void close_socket(int client_socket);

