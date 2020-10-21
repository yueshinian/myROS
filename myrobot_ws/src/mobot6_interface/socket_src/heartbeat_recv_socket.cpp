#include "../include/mobot6_interface/heartbeat_recv_socket.h"
#include <QThread>
#include <sys/time.h>
#include "errno.h"



//HeartbeatRecvSocket::HeartbeatRecvSocket(int port)
int create_heart_recv_socket(std::string _in ServerIP,
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

  std::cout << "Connect to client successfully." <<std::endl;
  second =0;
  heartbeatrecv_flag=1;
  return 0;

}

void close_recv_socket(int _in server_socket, int _in client_socket)
{
    close(client_socket);
    close(server_socket);
}

void Recv(int _in new_fd)
{
  //if(FD_ISSET(new_fd, &working_set))
  //{
        PACKET_HEAD_RECV head_recv;
        head_recv.type_recv=OTHER;
        recv(new_fd, &head_recv, sizeof(head_recv), 0);   // 先接受包头
        if(head_recv.type_recv == HEART)
        {
            //mmap[new_fd].second = 0;       // 每次收到心跳包，count置0
            second = 0;
            //std::cout << "Received heart-beat from client.\n";
            sleep(1);
        }
        else
        {
          std::cout << "Received heart-beat failed.\n";
          sleep(1);
                  // 数据包，通过head.length确认数据包长度
        }
 // }
}

void Run()
{

    //Accept();
    //pthread_t recv_id;     // 创建心跳检测线程
    int ret = pthread_create(&recv_id, NULL, heart_handler,NULL);
    if(ret != 0)
    {
        cout << "Can not create heart-beat checking thread.\n";
        sleep(1);
        ret = pthread_create(&recv_id, NULL, heart_handler, NULL);
    }
}

void* heart_handler(void* arg)
{
    cout << "The heartbeat checking thread started.\n";
   // HeartbeatRecvSocket* s = (HeartbeatRecvSocket*)arg;
    while(1)
    {
      if(second == 3)   // 1s*5没有收到心跳包，判定客户端掉线
      {
          cout << "The client has be offline.\n";
          HeartbeatFlag = false;   // 断开连接不可发送消息
          recv_flag=-1;
          heartbeatrecv_flag=0;
          cout << "startout\n";
          pthread_exit(0);
          //sleep(5);
      }
      else if(second < 3 && second >= 0)
      {
          second += 1;
      }
      //cout << "startout\n";
      //pthread_testcancel();
      //cout << "out\n";
      sleep(1);   // 定时1秒
      //pthread_testcancel();

    }
}
