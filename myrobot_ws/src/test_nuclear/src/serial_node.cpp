    #include "ros/ros.h"
    #include <serial/serial.h>
    #include <std_msgs/String.h>
    #include <std_msgs/Empty.h>
    #include <string>
    #include <iostream>
    #include <sstream>
    #include <boost/algorithm/string.hpp>

    #define     STP_STRING_MOVE     "{\"Seq\":1,\"DeviceID\":\"C0001\",\"Command\":\"COM_STP_POINT\",\"Data\":[{\"Point\":\"CMD_POS\",\"Speed\":CMD_SPEED}]}\\n"
    #define     STP_STRING_HOME     "{\"Seq\":1,\"DeviceID\":\"C0001\",\"Command\":\"COM_STP_ZERO\"}\\n"
    #define     STP_STRING_STOP     "{\"Seq\":1,\"DeviceID\":\"C0001\",\"Command\":\"COM_STP_STOP\"}\\n"
    #define     STP_STRING_REQ      "{\"Seq\":1,\"DeviceID\":\"C0001\",\"Command\":\"COM_VAL_REQ\"}\\n"

    serial::Serial ros_ser;       //声明串口对象
    //回调函数
    void jointstatesCallback(const std_msgs::String::ConstPtr& msg)
    {
	 std::string input_ = msg->data;
         ROS_INFO_STREAM("Writing: " << input_);
         ros_ser.write(input_);         //发送串口数据
    }


    int main (int argc, char** argv)
    {
        ros::init(argc, argv, "my_serial_node");  //初始化节点
        ros::NodeHandle n;                        //声明节点句柄

        ROS_INFO("test information");
        int a = 1;
        double b = 30.0;
        double c = 360.0;
        double d = c/b;
        int e = int(d);
        std::cout << e << " " << d << std::endl;
        while(1);

         //订阅主题/joint_states，并配置回调函数
         ros::Subscriber command_sub = n.subscribe("/serial_cmd", 1000, jointstatesCallback);

         std::string test_ = STP_STRING_REQ;
         std::vector<std::string> split_result;
         boost::split( split_result, test_, boost::is_any_of(" \":{}[],"),
                       boost::algorithm::token_compress_on );
         for (size_t i=0; i<split_result.size(); ++i)
             std::cout<<i+1<<": "<<split_result[i]<<"\t";
         std::cout<<std::endl;

         try
         {
             //设置串口属性，并打开串口
             ros_ser.setPort("/dev/ttyUSB0");
             ros_ser.setBaudrate(115200);
             serial::Timeout to = serial::Timeout::simpleTimeout(1000);
             ros_ser.setTimeout(to);
             ros_ser.open();
         }
         catch (serial::IOException& e)
         {
             ROS_ERROR("Unable to open port: %s.", e.what());
             return -1;
         }

         //检测串口是否已经打开，并给出提示信息
         if(ros_ser.isOpen()){
             ROS_INFO_STREAM("Serial Port opened");
         }else{
             return -1;
         }

         //指定循环的频率
         ros::Rate loop_rate(10);
	 ros_ser.write(STP_STRING_HOME);

         while(ros::ok()){

             //处理ROS的信息，比如订阅消息,并调用回调函数
             ros::spinOnce();

             if(ros_ser.available()){
                 ROS_INFO_STREAM("Reading from serial port");
                 std_msgs::String serial_data;
                 //获取串口数据
                 serial_data.data = ros_ser.read(ros_ser.available());
                 ROS_INFO_STREAM("Read: " << serial_data.data);
                 //将串口数据发布到主题sensor
                 //sensor_pub.publish(serial_data);
             }
             loop_rate.sleep();
         }
     }



