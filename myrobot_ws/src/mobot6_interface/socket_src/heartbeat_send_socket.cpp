#include "../include/mobot6_interface/heartbeat_send_socket.h"

void hrt_create_socket(std::string _in ClientIP,
                   int _in ClientPort,
                   int _out Client_socket)
{
  //创建套接字
  int res;
  Client_socket = socket(AF_INET, SOCK_STREAM, 0);
  //向服务器（特定的IP和端口）发起请求
  struct sockaddr_in serv_addr;
  memset(&serv_addr, 0, sizeof(serv_addr));  //每个字节都用0填充
  serv_addr.sin_family = AF_INET;  //使用IPv4地址
  serv_addr.sin_addr.s_addr = inet_addr( ClientIP.c_str() );  //具体的IP地址"192.168.1.1"
  serv_addr.sin_port = htons(ClientPort);  //端口1234

  cout << "Connecting......" << endl;
  unsigned long cmd_value = 1;
  ioctl(Client_socket, FIONBIO, &cmd_value);
  res = connect(Client_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
  /*hrt_send_close_fd(Client_socket);
  Client_socket = socket(AF_INET, SOCK_STREAM, 0);
  //向服务器（特定的IP和端口）发起请求
  memset(&serv_addr, 0, sizeof(serv_addr));  //每个字节都用0填充
  serv_addr.sin_family = AF_INET;  //使用IPv4地址
  serv_addr.sin_addr.s_addr = inet_addr( ClientIP.c_str() );  //具体的IP地址"192.168.1.1"
  serv_addr.sin_port = htons(ClientPort);  //端口1234

  cout << "Connecting......" << endl;
  res = connect(Client_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr));*/
  int num_count=0;
  while(res < 0)
  {
      cout << "Can not Connect to Server IP! retry after 1 seconds.\n";
      sleep(1);
      res = connect(Client_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
      num_count++;
      if(num_count>3)
      {
        //hrt_send_close_fd(Client_socket);
        //Client_socket = socket(AF_INET, SOCK_STREAM, 0);
        num_count=0;
        break;
      }
  }
  cout << "Connect to Server successfully." << endl;

}
void hrt_send_send(int _in client_socket)
{
  PACKET_HEAD_SEND head_send;
  head_send.type_send = HEART_HEART;
  socklen_t send_length=send(client_socket, &head_send, sizeof(head_send), 0);
  if((int)send_length<0)
  {

    std::cout << "[SOCKET_ERROR] send_message: cannot send." << std::endl;
    hrt_send_error_count++;
    if(hrt_send_error_count>2)
      {
      HeartbeatFlag = false;
      hrt_send_error_count=0;
      }

  }
  else
    //std::cout << "success send" << std::endl;
  sleep(1);     // 定时1秒
}

void hrt_send_close_fd(int _in Client_socket)
{
    close(Client_socket);
}

// thread function
/*void* send_heart(void* arg)
{
    cout << "The heartbeat sending thread started.\n";

    int count = 0;  // 测试
    while(1)
    {
        PACKET_HEAD_SEND head_send;
        head_send.type_send = HEART_HEART;
        head_send.length_send = 0;
        socklen_t send_length=send(hrt_send_Client_socket, &head_send, sizeof(head_send), 0);
        if((int)send_length<0)
        {

          std::cout << "[SOCKET_ERROR] send_message: cannot send." << std::endl;
          count++;
          if(count>6)
            {
            HeartbeatFlag = false;
            }

        }
        else
          std::cout << "success send" << std::endl;
        sleep(1);     // 定时1秒
        if(HeartbeatFlag == false)
        {
            pthread_exit(0);
        }
    }
}*/

