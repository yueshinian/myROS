#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <log4cxx/logger.h> 

// define the input and output label macros
#define _in   const &   // input parameter of right value
#define _inc  const     // input parameter of right value
#define _out        &   // output parameter of left value

#define _wt         &   // return value of writable left value
#define _rd   const &   // return value of readonly left value
#define _ret  const     // return by value (use c++ complier optimaization)

// define the corresponding topics
#define     STATE_LOOP                          0.150
#define     INTERFACE_JOINT_STATES_ARM          "/joint_states_arm"

// define the active links and joints of manipulator
#define     JOINT_ARM_NUM           7
#define     JOINT1_ROS_AXIS         "joint_turret_rotate"
#define     JOINT2_ROS_AXIS         "joint_bigarm_pitch"
#define     JOINT3_ROS_AXIS         "joint_forearm_pitch"
#define     JOINT4_ROS_AXIS         "joint_forearm_rotate"
#define     JOINT5_ROS_AXIS         "joint_wrist_pitch"
#define     JOINT6_ROS_AXIS         "joint_wrist_rotate"
#define     JOINT7_ROS_AXIS         "joint_rod_left1"
std::string JOINTS_ROS_AXIS[] = { JOINT1_ROS_AXIS, JOINT2_ROS_AXIS, JOINT3_ROS_AXIS, JOINT4_ROS_AXIS,
                                  JOINT5_ROS_AXIS, JOINT6_ROS_AXIS, JOINT7_ROS_AXIS };

//##########################
//###   main  function   ###
//##########################
int main(int argc, char **argv)
{
    ros::init( argc, argv, "fake_arm_states" );
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle  NodeHandle;
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
                ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::Publisher  pArmJointState = NodeHandle.advertise<sensor_msgs::JointState>(
                INTERFACE_JOINT_STATES_ARM, 1024 );

    std::vector<std::string> feedback_names( JOINT_ARM_NUM );
    std::vector<double> feedback( JOINT_ARM_NUM );
    std::vector<double> velocity( JOINT_ARM_NUM );
    for (size_t i = 0; i < JOINT_ARM_NUM; ++i)
    {
        feedback_names[i] = JOINTS_ROS_AXIS[i];
        feedback[i] = 0.0;
        velocity[i] = 0.0;
    }

    sensor_msgs::JointState arm_states;
    arm_states.name = feedback_names;
    arm_states.position = feedback;
    arm_states.velocity = velocity;

    // main loop start
    ROS_INFO( "main loop starts" );
    while(ros::ok())
    {
        double start = ros::Time::now().toSec();

        // fill in the arm_states and publish to "/joint_states_arm"
        arm_states.header.stamp = ros::Time::now();
        pArmJointState.publish( arm_states );

        double sleep_time = STATE_LOOP - ( ros::Time::now().toSec() - start );
        if ( sleep_time > 0 )
        {
            ros::Duration( sleep_time ).sleep();
            ROS_DEBUG("Cost Time: %lf ms", (STATE_LOOP-sleep_time)*1000);
        }
        else
            ROS_WARN ( "control loop over time: %f s", -sleep_time );
    }
    ROS_INFO( "main-loop ends" );

    ros::waitForShutdown();
    return 0;
}
