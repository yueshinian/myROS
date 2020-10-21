#include <ros/ros.h>
#include <string>
#include "vector"
#include <stdlib.h>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#include <QThread>
#include "mobot6_interface/command_list.h"
#include "mobot6_interface/heart_reconnection.h"
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
    pub = n.advertise<mobot6_interface::command_list>( "/command_input", 1, true );
    pub_heart = n.advertise<mobot6_interface::heart_reconnection>( "/heart_connect", 1, true );
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

  void run()
  {
      ros::Rate loop_rate(20);
      mobot6_interface::command_list   KeyMsg;
      mobot6_interface::heart_reconnection heart_reconnetion_flag;
      int cnt = 0;
      while ( 1 )
      {
          KeyMsg.joints = joints;
          KeyMsg.lspb   = lspb;
          KeyMsg.cartesians = cartesians;
          KeyMsg.moves = moves;
          KeyMsg.swings = swings;
          KeyMsg.nuclear = nuclear;
          KeyMsg.grippers = grippers;
          KeyMsg.framesID =  frameID;
          KeyMsg.commandID = commandid;
          KeyMsg.laser_flag = laser_flag;
          heart_reconnetion_flag.heartbeatrecv_flag=heartbeatrecv_flag;
          heart_reconnetion_flag.HeartbeatFlag=HeartbeatFlag;
          heart_reconnetion_flag.hrt_recv_num=hrt_recv_num;
          heart_reconnetion_flag.hrt_send_num=hrt_send_num;
          pub_heart.publish(heart_reconnetion_flag);
          if(recv_done_flag == 1&& HeartbeatFlag==1)
          {
              pub.publish( KeyMsg );
              cnt++;
     	      if(cnt > 1)
	      {
              recv_done_flag = 0;
              for (size_t i = 0; i < 6; ++i)
                  joints[i] = 0.0;
              for (size_t i = 0; i < 6; ++i)
                  cartesians[i] = 0.0;
              for (size_t i = 0; i < 4; ++i)
                  moves[i] = 0.0;
              for (size_t i = 0; i < 2; ++i)
                  swings[i] = 0.0;
              lspb.resize(7,0.0);
              grippers  = 0.0;
              commandid = 8;
              laser_flag=0;
	      cnt = 0;
	      }
          }
          loop_rate.sleep();
      }
      std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
      ros::waitForShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }

private:
	int init_argc;
	char** init_argv;
  ros::Publisher pub;
  ros::Publisher pub_heart;
};

