#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mobot6_interface/Keyboard.h>
#include "geometry_msgs/Twist.h"
#include <string>
#include <iostream>


template<typename T, size_t N>
char(&ArraySizeHelper(const T(&array)[N]))[N];
#define ARRAY_SIZE(array) (sizeof(ArraySizeHelper(array)))

#define JOINT_DOF   6
std::vector<double> joints(6,0.0);
std::vector<double> cartesians(6,0.0);
std::vector<double> moves(4,0.0);
std::vector<double> swings(4,0.0);
double grippers=0.0;
int framesID=7;
int commandID=4;
std::vector<double> lspb;
int i=0;
// test array: the last element is duration time
void cmd_vel_callback(const geometry_msgs::Twist& msg)
{
  moves.clear();
  moves.push_back(msg.linear.x);
  moves.push_back(0.0);
  moves.push_back(0.0);
  moves.push_back(msg.angular.z);
  i=0;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "veh_cmd_vel");
    ros::NodeHandle nh;
    ros::Publisher  PubArmCmd = nh.advertise<mobot6_interface::Keyboard>(
                "/keyboard_input", 3, true);
    ros::Subscriber  sub_vel = nh.subscribe("cmd_vel", 3, &cmd_vel_callback);
    //
    mobot6_interface::Keyboard    cmd_input;
    lspb.clear();
    ros::Rate loop_rate(20);
    //
    while(ros::ok())
    {
      ros::spinOnce();
      i++;
      cmd_input.joints=joints;
      cmd_input.cartesians=cartesians;
      cmd_input.moves=moves;
      cmd_input.swings=swings;
      cmd_input.grippers=grippers;
      cmd_input.framesID=framesID;
      cmd_input.commandID=commandID;
      cmd_input.lspb=lspb;
      PubArmCmd.publish(cmd_input);
      if(i>=2)
      {
        moves.clear();
        for(int j=0;j<4;j++)
        {
          moves.push_back(0.0);
        }
        i=0;
      }

      loop_rate.sleep();
    }

    return 1;
}



