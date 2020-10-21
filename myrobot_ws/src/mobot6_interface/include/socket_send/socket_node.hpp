#include <ros/ros.h>
#include <string>
#include "vector"
#include <stdlib.h>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#include <math.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <QThread>
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "mobot6_interface/heart_reconnection.h"
#include "mobot6_interface/Intensity.h"
#include "mobot6_interface/PointIntensity.h"

#include "control_flag.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/
#define     _in             const &
#define     _out            &


/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {

public:
  QNode(int argc, char** argv):
    init_argc(argc),
    init_argv(argv)
  {

  }
  virtual ~QNode()
  {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
  wait();
  }
  bool init()
  {
    ros::init(init_argc,init_argv,"socket");
    if ( ! ros::master::check() ) {
      return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    // Add your ros communications here.
    sub_1 = n.subscribe("joint_states", 1, &QNode::poseMessageReceived, this );
    sub_2 = n.subscribe("check_node", 3, &QNode::check_nodeReceived, this );
    sub_3 = n.subscribe("interface_pressure", 3, &QNode::check_pressureReceived, this );
    sub_4 = n.subscribe("interface_states", 3, &QNode::interface_statesReceived, this );
    sub_5 = n.subscribe("interface_states_vehicle", 3, &QNode::interface_states_vehicleReceived, this );
    sub_6 = n.subscribe("laser_distance", 3, &QNode::distanceReceived, this );
    sub_heart = n.subscribe("heart_connect", 3, &QNode::heartReceived, this );
    sub_odom = n.subscribe("odom", 3, &QNode::odomReceived, this );
    sub_nuclear_interface_state = n.subscribe("interface_states_nuclear", 3, &QNode::interface_states_nuclear_Received, this );
    sub_nuclear_intensity = n.subscribe("nuclear_intensity", 3, &QNode::nuclear_intensity_Received, this );
    sub_nuclear_intensity_point1 = n.subscribe("nuclear_intensity_point1", 3, &QNode::nuclear_intensity_point1_Received, this );
    sub_nuclear_intensity_point2 = n.subscribe("nuclear_intensity_point2", 3, &QNode::nuclear_intensity_point2_Received, this );
    sub_posestamped = n.subscribe("vehicle_nav_command", 3, &QNode::nav_com_Received, this );
    start();
    return true;
  }
  bool init(const std::string &master_url, const std::string &host_url)
  {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"socket");
    if ( ! ros::master::check() ) {
      return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    start();
    return true;
  }

//#########################################
//###  2. define the Function quat2yaw  ###
//#########################################
double quat2yaw(const geometry_msgs::Quaternion msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    return yaw;
}


  void poseMessageReceived( const sensor_msgs::JointState& msg)
  {
     names.clear();
     position.clear();
     names=msg.name;
     position=msg.position;
  }
  
  void heartReceived( const mobot6_interface::heart_reconnection& msg)
  {
     HeartbeatFlag=msg.HeartbeatFlag;
     heartbeatrecv_flag=msg.heartbeatrecv_flag;
     hrt_recv_num=msg.hrt_recv_num;
     hrt_send_num=msg.hrt_send_num;
  }

  void check_nodeReceived( const std_msgs::String& msg)
  {
     check_node=msg.data;
  }
  void check_pressureReceived( const std_msgs::Float32& msg)
  {
     Pressure=msg.data;
  }
  void interface_statesReceived( const std_msgs::String& msg)
  {
     std::string interface_states;
     std::vector<std::string> result;
     interface_states=msg.data;
     boost::split( result, interface_states, boost::is_any_of(" \t"), boost::algorithm::token_compress_on );
     //mobot_mode.clear();
     mobot_mode=result[0];
    // if(mobot_mode!="MOVE"&&mobot_mode!="INIT"&&mobot_mode!="STOP"&&mobot_mode!="HOME")
        //mobot_mode = "NO_MATCH";

  }
  void interface_states_vehicleReceived( const std_msgs::String& msg)
  {
     std::string interface_states_vehicle;
     std::vector<std::string> result;
     interface_states_vehicle=msg.data;
     boost::split( result, interface_states_vehicle, boost::is_any_of(" \t"),        boost::algorithm::token_compress_on );
     //vehicle_mode.clear();
     vehicle_mode=result[0];
     //if(vehicle_mode!="VELOCITY"&&vehicle_mode!="HOME"&&vehicle_mode!="RESET")
        //vehicle_mode = "NO_MATCH";
  }

  void distanceReceived( const std_msgs::Float32& msg)
  {
      Distance=msg.data;
      //std::cout<<Distance<<std::endl;
  }

void odomReceived( const nav_msgs::Odometry& msg)
  {
      odom=msg;
  }

void interface_states_nuclear_Received(const std_msgs::String& msg)
{
  interface_states_nuclear=msg.data;
}

void nuclear_intensity_Received(const mobot6_interface::Intensity& msg)
{
  seq=msg.detect_seq;
  rotation_max=msg.detect_rotation_max;
  intensity_max=msg.detect_intensity_max;
}

void nuclear_intensity_point1_Received(const mobot6_interface::PointIntensity& msg)
{
  p1_x=msg.pose.position.x;
  p1_y=msg.pose.position.y;
  p1_z=msg.pose.position.z;
  p1_theta=quat2yaw(msg.pose.orientation);
  point1_seq=msg.detect_seq;
  point1_rotation=msg.detect_rotation;
  point1_forward_intensity=msg.forward_intensity;
  point1_inverse_intensity=msg.inverse_intensity;
}

void nuclear_intensity_point2_Received(const mobot6_interface::PointIntensity& msg)
{
  point2_seq=msg.detect_seq;
  point2_rotation=msg.detect_rotation;
  point2_forward_intensity=msg.forward_intensity;
  point2_inverse_intensity=msg.inverse_intensity;
}

void nav_com_Received(const geometry_msgs::PoseStamped& msg)
{
  nav_x=msg.pose.position.x;
  nav_y=msg.pose.position.y;
  nav_z=msg.pose.position.z;
  nav_theta=quat2yaw(msg.pose.orientation);
}

  void run()
  {
      ros::Rate loop_rate(25);
      while ( 1 )
      {
          ros::spinOnce();
          loop_rate.sleep();
      }
      std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
      ros::waitForShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }


private:
	int init_argc;
	char** init_argv;
  ros::Subscriber sub_1;
  ros::Subscriber sub_2;
  ros::Subscriber sub_3;
  ros::Subscriber sub_4;
  ros::Subscriber sub_5;
  ros::Subscriber sub_6;
  ros::Subscriber sub_heart;
  ros::Subscriber sub_odom;
  ros::Subscriber sub_nuclear_interface_state;
  ros::Subscriber sub_nuclear_intensity;
  ros::Subscriber sub_nuclear_intensity_point1;
  ros::Subscriber sub_nuclear_intensity_point2;
  ros::Subscriber sub_posestamped;
};

