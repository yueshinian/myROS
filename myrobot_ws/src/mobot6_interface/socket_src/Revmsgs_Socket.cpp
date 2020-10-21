#include "../include/mobot6_interface/Revmsgs_Socket.h"

std::string     MATCH_SWING[] = { MATCH_SWING_0, MATCH_SWING_1,
                                  MATCH_SWING_2, MATCH_SWING_3 };

std::string     COMMAND_ID[] = { COMMAND_ID_0, COMMAND_ID_1, COMMAND_ID_2,
                                 COMMAND_ID_3, COMMAND_ID_4, COMMAND_ID_5,
                                 COMMAND_ID_6, COMMAND_ID_7, COMMAND_ID_8,
                                 COMMAND_ID_9,COMMAND_ID_10,COMMAND_ID_11,
                                 COMMAND_ID_12,COMMAND_ID_13,COMMAND_ID_14,
                                 COMMAND_ID_15};

int             MAP_FRAME[]   = { 0, 0, 0, 0, 0, 0, 0, 0, 0};//{ 0, 7, 7, 7, 7, 7, 7, 7, 7}
std::string     MATCH_FRAME[] = { MATCH_FRAME_0, MATCH_FRAME_1, MATCH_FRAME_2,
                                  MATCH_FRAME_3, MATCH_FRAME_4, MATCH_FRAME_5,
                                  MATCH_FRAME_6, MATCH_FRAME_7, MATCH_FRAME_8 };

int             MAP_HOME[]   = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
std::string     MATCH_HOME[] = { MATCH_HOME_0, MATCH_HOME_1, MATCH_HOME_2,
                                 MATCH_HOME_3, MATCH_HOME_4, MATCH_HOME_5,
                                 MATCH_HOME_6, MATCH_HOME_7, MATCH_HOME_8 };


int create_socket(std::string _in ClientIP,
                   int _in ClientPort,
                   int _out client_socket)
{
    //åå»ºå¥æ¥å­
    client_socket = socket(AF_INET, SOCK_STREAM, 0);
    //åæå¡å¨ï¼ç¹å®çIPåç«¯å£ï¼åèµ·è¯·æ±
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));  //æ¯ä¸ªå­èé½ç¨0å¡«å
    serv_addr.sin_family = AF_INET;  //ä½¿ç¨IPv4å°å
    serv_addr.sin_addr.s_addr = inet_addr( ClientIP.c_str() );  //å·ä½çIPå°å"192.168.1.1"
    serv_addr.sin_port = htons(ClientPort);  //ç«¯å£1234

    recv_flag=-1;
    std::cout<<"ko"<<std::endl;

    int res = connect(client_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    int num1=0;
    while(res < 0)
    {
        std::cout<<"Waiting for connecting.\n"<<std::endl;
        sleep(1);
        res = connect(client_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
	num1++;
	if(num1>10)
	{
		return -1;
		break;
	}
    }

    recv_flag=res;
    std::cout<<"ok"<<std::endl;
    return 0;

}

void receive_msgs(int client_socket)
{
    char Buffer_recv[550];
    memset(Buffer_recv,0,550);
    std::cout<<"1"<<std::endl;
    socklen_t length_recv = recv(client_socket, Buffer_recv, 550, 0);
    std::cout<<"2"<<std::endl;
    if( (int)length_recv < 0)
    {
         printf("recv msg error: %s(errno: %d)\n", strerror(errno), errno);
         sleep(1);
    }
    CommandString= Buffer_recv;
    std::cout << " Receive message: " << Buffer_recv<< std::endl;
}


int ResolveCommandString( std::string _in input,
                           std::string _out COMMAND,
                           std::string _out AUXCMD,
                           std::vector<double> _out VALUES )
{
    std::vector<std::string> result;
    boost::split( result, input, boost::is_any_of(" "), boost::algorithm::token_compress_on );
    COMMAND = result[0];
    AUXCMD  = "";
    VALUES.clear();

    if (COMMAND == "ZERO")
    {
        AUXCMD = result[1];
    }
    else if ((COMMAND == "GRIP")||(COMMAND == "BASE")||(COMMAND == "MOVJ_LSPB")||(COMMAND == "MCHI_LSPB")||(COMMAND == "NUCLEAR"))
    {
        for (size_t i = 1; i < result.size(); ++i)
            VALUES.push_back( atof(result[i].c_str()) );
    }
    else if ((COMMAND == "ARMJ")||(COMMAND == "ARMC")||(COMMAND == "SWING")||(COMMAND == "HOME"))
    {
        AUXCMD = result[1];
        for (size_t i = 2; i < result.size(); ++i)
        {
            VALUES.push_back( atof(result[i].c_str()) );
        }
    }
    else
    {
        if ((COMMAND != "STOP")&&(COMMAND != "MOVE")&&(COMMAND != "HOMESTOP")&&(COMMAND != "NO_LIMIT")&&(COMMAND != "SPLINE")&&(COMMAND != "LASER"))
        {
            std::cout << "invalid command string" << std::endl;
            return -1;
        }
    }
    return result.size();
}

void MatchFrame( std::string _in FRAME, int _in MODE, int _out ID )
{
    if ( MODE == MATCH_MODE_ARM )
    {
        for (size_t i = 0; i < 9; ++i)
        {
            if ( FRAME == MATCH_FRAME[i] )
            {
                ID = MAP_FRAME[i];
                break;
            }
            else
                ID = -1;
        }
    }
    else if ( MODE == MATCH_MODE_HOME )
    {
        for (size_t i = 0; i < 9; ++i)
        {
            if ( FRAME == MATCH_HOME[i] )
            {
                ID = MAP_HOME[i];
                break;
            }
            else
                ID = -1;
        }
    }
    else if ( MODE == MATCH_MODE_SWING )
    {
        for (size_t i = 0; i < 4; ++i)
        {
            if ( FRAME == MATCH_SWING[i] )
            {
                ID = i;
                break;
            }
            else
                ID = -1;
        }
    }
    else
        ID = -1;
    return;
}

void close_socket(int client_socket)
{
    close(client_socket);
}

