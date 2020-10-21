
#include "../include/socket_send/Sendmsgs_Socket.h"

template < typename T > std::string to_string( const T& n )
{
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
}

int create_socket(std::string _in ServerIP,
                  int _in ServerPort,
                  int _out server_socket,
                  int _out client_socket)
{
    // create the server socket description
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    while (server_socket < 0)
    {
        std::cout << "[SOCKET_ERROR] Create socket failed, exiting." <<std::endl;
        sleep(1);
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
    }

    // bind IP and Port with the server socket description
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(ServerIP.c_str());// "127.0.0.1"
    server_addr.sin_port = htons(ServerPort);

    int opt = 1;
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    while( bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0 )
    {
        std::cout << "[SOCKET_ERROR] Server bind failed, exiting." << std::endl;
        sleep(1);
        bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
    }

    // server socket start to listen
    while( listen(server_socket, LISTEN_QUEUE) < 0 )
    {
        std::cout << "[SOCKET_ERROR] Server listen failed, exiting." << std::endl;
        sleep(1);
        listen(server_socket, LISTEN_QUEUE);
    }

    // server socket start to accept client sockets
    struct sockaddr_in client_addr;
    socklen_t client_addr_length = sizeof(client_addr);

    client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_addr_length);
    while ( client_socket < 0 )
    {
        std::cout << "[SOCKET_ERROR] create_socket: cannot connect to client socket." << std::endl;
        sleep(1);
        client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_addr_length);
    }

        std::cout << "Connect to recvmsg client successfully." <<std::endl;

    return 0;
}

void close_socket(int _in server_socket, int _in client_socket)
{
    close(client_socket);
    close(server_socket);
}

int send_message( int _in client_socket, std::string _in CommandString )
{
    char Buffer_send[350];
    strcpy(Buffer_send, CommandString.c_str());
    socklen_t send_length = send(client_socket, Buffer_send, 350, 0);
    if ( (int)send_length < 0 )
    {
        std::cout << "[SOCKET_ERROR] send_message: cannot send command string." << std::endl;
        return -1;
    }
    
    //std::cout << " Send message: " << Buffer_send << std::endl;
    memset(Buffer_send ,0,350);
    return 0;
}

void TanslateToJointStateString(std::vector<double> _in position,
                                std::string _in node_check,
                                float _in pressure,
                                std::string _in mobot_mode,
                                std::string _in vehicle_mode,
                                float _in Distance,
                                nav_msgs::Odometry _in odometry,
                                std::string nuclear_interface_state,
                                int seq,
                                float intensity_max,
                                float rotation_max,
                                int point1_seq,
                                float point1_rotation,
                                float point1_forward_intensity,
                                float point1_inverse_intensity,
                                int point2_seq,
                                float point2_rotation,
                                float point2_forward_intensity,
                                float point2_inverse_intensity,
				float nav_com_x,
				float nav_com_y,
				float nav_com_z,
  				float nav_com_theta,
				float p1_x,
				float p1_y,
				float p1_z,
  				float p1_theta,
                                std::string _out output)
{

  std::vector<std::string> input;
  input.clear();
  for(int i=0;i<11;i++)
  {
      input.push_back(to_string((int)(position[i]*1000)/1000.0));
  }
  input.push_back(node_check);
  input.push_back(to_string((int)(pressure*1000)/1000.0));
  input.push_back(mobot_mode);
  input.push_back(vehicle_mode);
  input.push_back(to_string(Distance));
  input.push_back(to_string((int)(odometry.pose.pose.position.x*1000)/1000.0));
  input.push_back(to_string((int)(odometry.pose.pose.position.y*1000)/1000.0));
  input.push_back(to_string((int)(odometry.pose.pose.orientation.x*1000)/1000.0));
  input.push_back(to_string((int)(odometry.pose.pose.orientation.y*1000)/1000.0));
  input.push_back(to_string((int)(odometry.pose.pose.orientation.z*1000)/1000.0));
  input.push_back(to_string((int)(odometry.pose.pose.orientation.w*1000)/1000.0));
  input.push_back(to_string((int)(odometry.twist.twist.linear.x*1000)/1000.0));
  input.push_back(to_string((int)(odometry.twist.twist.angular.z*1000)/1000.0));
  input.push_back(nuclear_interface_state);
  input.push_back(to_string(seq));
  input.push_back(to_string((int)(intensity_max*10)/10.0));
  input.push_back(to_string((int)(rotation_max*10)/10.0));
  input.push_back(to_string(point1_seq));
  input.push_back(to_string((int)(point1_rotation*10)/10.0));
  input.push_back(to_string((int)(point1_forward_intensity*10)/10.0));
  input.push_back(to_string((int)(point1_inverse_intensity*10)/10.0));
  input.push_back(to_string(point2_seq));
  input.push_back(to_string((int)(point2_rotation*10)/10.0));
  input.push_back(to_string((int)(point2_forward_intensity*10)/10.0));
  input.push_back(to_string((int)(point2_inverse_intensity*10)/10.0));
  input.push_back(to_string((int)(nav_com_x*10)/10.0));
  input.push_back(to_string((int)(nav_com_y*10)/10.0));
  input.push_back(to_string((int)(nav_com_z*10)/10.0));
  input.push_back(to_string((int)(nav_com_theta*10)/10.0));
  input.push_back(to_string((int)(p1_x*10)/10.0));
  input.push_back(to_string((int)(p1_y*10)/10.0));
  input.push_back(to_string((int)(p1_z*10)/10.0));
  input.push_back(to_string((int)(p1_theta*10)/10.0));
  output = boost::algorithm::join( input, " ");
}

