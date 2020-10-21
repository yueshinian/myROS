#include <stdio.h>
#include <string.h>
#include <sys/types.h>
//#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "mobot6_interface/command_list.h"
#define   datalen     5	     //向激光测距模块写入的命令长度

bool laser_start_flag = 0;

void open_port(int& fd)
{
	fd=open("/dev/ttyUSB0",O_RDWR|O_NOCTTY|O_NDELAY);
	if(fd<0)
		printf("open port error\n");
	if(fcntl(fd,F_SETFL,0)<0)
		printf("block state failed!\n");
	if(isatty(STDIN_FILENO)==0)
		printf("standard input is not a terminal device\n");
}

void set_port(int fd)
{
	struct termios newtio;
	bzero(&newtio,sizeof(newtio));
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	newtio.c_cflag |= CS8;
	newtio.c_cflag &= ~PARENB;
	newtio.c_cflag &= ~CSTOPB;
	cfsetispeed(&newtio,B9600);	//读速度9600bits
	cfsetospeed(&newtio,B9600);	//发速度9600bits
	newtio.c_cc[VTIME]=0;
	newtio.c_cc[VMIN]=0;
	tcflush(fd,TCIOFLUSH);
	int i=tcsetattr(fd,TCSANOW,&newtio);
	if(i!=0)
	{
		printf("set port error\n");
	}
	else
	{
		printf("set port done!\n");
	}
}

void message_callback( const mobot6_interface::command_list& msg)
{
	laser_start_flag = msg.laser_flag;
}

int main( int argc, char **argv)
{
	ros::init(argc,argv,"laser");
	ros::NodeHandle NodeHandle;
	ros::Publisher pub = NodeHandle.advertise<std_msgs::Float32>("laser_distance", 3,false );
	ros::Subscriber sub = NodeHandle.subscribe("command_input",1,&message_callback);
	ROS_INFO ( "initializing ROS succeed" );
	ros::Rate loop(1);
	int fd=0;
	char send_data[datalen] = "/0";
	char* buff=(char*)malloc(1024);
	send_data[0]=0x80;
	send_data[1]=0x06;
	send_data[2]=0x02;
	send_data[3]=0x78;
	int32_t baut_rate=9600;
	int data_bits=8;
	int stop_bit=1;
	char measure[7] = "/0";
	float num;
	std_msgs::Float32 laser_dis;
	//打开串口设备ttyUSB0
	open_port(fd);
	//对串口读写模式进行设置
	set_port(fd);

	while(ros::ok())
	{
		ros::spinOnce();
                printf("laser_start_flag=%d \n",laser_start_flag);
                //std::cout<<laser_start_flag<<std::endl;
		if(laser_start_flag == 1)
		{
			write(fd,send_data,datalen);//向串口设备下发命令
			while(1)
			{
				int j=read(fd,buff,11);
				//printf("%d",j);
				if(j!=0)
					break;
			}

			for(int i=0;i<7;i++)
			{
				measure[i]=buff[i+3];
			}

			sscanf(measure,"%f",&num);
			ROS_INFO("laser distance = %f",num);
	
			laser_dis.data = num;
			pub.publish(laser_dis);	
                        laser_start_flag=0;
		}
		else
		{
			laser_dis.data = 0;
			pub.publish(laser_dis);	
		}
		loop.sleep();
	}
	close(fd);
	return 0;
}
