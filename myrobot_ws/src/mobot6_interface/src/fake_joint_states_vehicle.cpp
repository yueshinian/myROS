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
#define     INTERFACE_JOINT_STATES_VEHICLE      "/joint_states_vehicle"

// define the active links and joints of vehicle
#define     JOINT_VEHICLE_NUM           4
#define     JOINT_LEFT_SWINGARM         "joint_swingarm_left"
#define     JOINT_RIGHT_SWINGARM        "joint_swingarm_right"
#define     JOINT_LEFT_WHEEL            "base_l_wheel_joint"
#define     JOINT_RIGHT_WHEEL           "base_r_wheel_joint"
std::string VEHICLE_JOINT[] = {JOINT_LEFT_SWINGARM, JOINT_RIGHT_SWINGARM, JOINT_LEFT_WHEEL, JOINT_RIGHT_WHEEL};

//##########################
//###   main  function   ###
//##########################
int main(int argc, char **argv)
{
    ros::init( argc, argv, "fake_vehicle_states" );
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle  NodeHandle;
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
                ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::Publisher  pVehicleJointState = NodeHandle.advertise<sensor_msgs::JointState>(
                INTERFACE_JOINT_STATES_VEHICLE, 1024 );

    std::vector<std::string> feedback_names( JOINT_VEHICLE_NUM );
    std::vector<double> feedback( JOINT_VEHICLE_NUM );
    std::vector<double> velocity( JOINT_VEHICLE_NUM );
    for (size_t i = 0; i < JOINT_VEHICLE_NUM; ++i)
    {
        feedback_names[i] = VEHICLE_JOINT[i];
        feedback[i] = 0.0;
        velocity[i] = 0.0;
    }

    sensor_msgs::JointState vehicle_states;
    vehicle_states.name = feedback_names;
    vehicle_states.position = feedback;
    vehicle_states.velocity = velocity;

    // main loop start
    ROS_INFO( "main loop starts" );
    while(ros::ok())
    {
        double start = ros::Time::now().toSec();

        // fill in the vehicle_states and publish to "/joint_states_vehicle"
        vehicle_states.header.stamp = ros::Time::now();
        pVehicleJointState.publish( vehicle_states );

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
