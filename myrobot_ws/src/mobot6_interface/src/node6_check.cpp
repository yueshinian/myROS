#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <log4cxx/logger.h> 

#define _in   const &   // input parameter of right value
#define _inc  const     // input parameter of right value
#define _out        &   // output parameter of left value

#define _wt         &   // return value of writable left value
#define _rd   const &   // return value of readonly left value
#define _ret  const     // return by value (use c++ complier optimaization)

#define     STATE_LOOP                      0.500
#define     MANIPULATOR_STATES              "/arm_states"                   // coord pub
#define     VEHICLE_STATES                  "/vehicle_states"               // coord pub
#define     INTERFACE_STATES                "/interface_states"             // inter pub, coord sub
#define     INTERFACE_STATES_VEHICLE        "/interface_states_vehicle"     // inter pub, coord sub

#define     NODE_STATES                     "/check_node"


//#############################################
//###  define the class JointStateHandller  ###
//#############################################
class StringHandllerOnce
{
public:
    void callback(const std_msgs::String::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    StringHandllerOnce()
        : mCrtMsg()
        , mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const std_msgs::String & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    std_msgs::String mCrtMsg;
    bool mNewMsg;
};

//##########################
//###   main  function   ###
//##########################
int main(int argc, char **argv)
{
    ros::init( argc, argv, "node_check" );
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle  NodeHandle;
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
                ros::console::g_level_lookup[ros::console::levels::Debug]);

    StringHandllerOnce  ARM_COORD_HANDLLER;
    StringHandllerOnce  VEHICLE_COORD_HANDLLER;
    StringHandllerOnce  ARM_INTER_HANDLLER;
    StringHandllerOnce  VEHICLE_INTER_HANDLLER;

    ros::Subscriber  sub_coord_manipulator = NodeHandle.subscribe<std_msgs::String>(
                MANIPULATOR_STATES, 3, &StringHandllerOnce::callback, &ARM_COORD_HANDLLER );
    ros::Subscriber  sub_coord_vehicle = NodeHandle.subscribe<std_msgs::String>(
                VEHICLE_STATES, 3, &StringHandllerOnce::callback, &VEHICLE_COORD_HANDLLER);
    ros::Subscriber  sub_inter_manipulator = NodeHandle.subscribe<std_msgs::String>(
                INTERFACE_STATES, 3, &StringHandllerOnce::callback, &ARM_INTER_HANDLLER );
    ros::Subscriber  sub_inter_vehicle = NodeHandle.subscribe<std_msgs::String>(
                INTERFACE_STATES_VEHICLE, 3, &StringHandllerOnce::callback, &VEHICLE_INTER_HANDLLER);

    ros::Publisher  pNodeCheck = NodeHandle.advertise<std_msgs::String>( NODE_STATES, 3, true);

    // main loop start
    ROS_INFO( "main loop starts" );
    while(ros::ok())
    {
        double start = ros::Time::now().toSec();
        std_msgs::String output;
        ros::spinOnce();

        // get the node_states of manipulator and vehicle
        if (ARM_COORD_HANDLLER.newMsg() && VEHICLE_COORD_HANDLLER.newMsg()
                && ARM_INTER_HANDLLER.newMsg() && VEHICLE_INTER_HANDLLER.newMsg())
        {
            output.data = "DONE";
        }
        else if (!ARM_COORD_HANDLLER.newMsg())
        {
            output.data = "1";
        }
        else if (!ARM_INTER_HANDLLER.newMsg())
        {
            output.data = "2";
        }
        else if (!ARM_INTER_HANDLLER.newMsg())
        {
            output.data = "3";
        }
        else if (!VEHICLE_INTER_HANDLLER.newMsg())
        {
            output.data = "4";
        }
        else
        {
            output.data = "-111";
        }
        pNodeCheck.publish(output);

        // time control
        double sleep_time = STATE_LOOP - ( ros::Time::now().toSec() - start );
        if ( sleep_time > 0 )
        {
            ros::Duration ( sleep_time ).sleep();
            ROS_DEBUG("cost Time: %lf ms", (STATE_LOOP-sleep_time)*1000);
        }
        else
            ROS_WARN ( "control loop over time: %f s", -sleep_time );
    } // main loop ends
    ROS_INFO( "main-loop ends" );

    ros::waitForShutdown();
    return 0;
}
