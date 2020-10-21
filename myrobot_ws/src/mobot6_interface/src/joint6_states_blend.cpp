#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <log4cxx/logger.h>


#define _in   const &   // input parameter of right value
#define _inc  const     // input parameter of right value
#define _out        &   // output parameter of left value

#define _wt         &   // return value of writable left value
#define _rd   const &   // return value of readonly left value
#define _ret  const     // return by value (use c++ complier optimaization)

#define     STATE_LOOP                       0.150
#define     INTERFACE_JOINT_STATES_ARM      "/joint_states_arm"
#define     INTERFACE_JOINT_STATES_VEHICLE  "/joint_states_vehicle"
#define     INTERFACE_JOINT_STATES          "/joint_states"  //20180505


//#############################################
//###  define the class JointStateHandller  ###
//#############################################
class JointStateHandller
{
public:
    void callback(const sensor_msgs::JointState::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    JointStateHandller()
        : mCrtMsg()
        , mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const sensor_msgs::JointState & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    sensor_msgs::JointState mCrtMsg;
    bool mNewMsg;
};


//##############################################
//###  define the function BlendJointStates  ###
//##############################################
int BlendJointStates( std::vector<sensor_msgs::JointState> _in input, sensor_msgs::JointState _out output )
{
    // check the feasibility of the input vector (sensor_msgs::JointState)
    if (input.size() <= 0)
        ROS_ERROR("ERROR <BlendJointStates>: the input vector has no availuable data.");

    // join the vector into a blended message
    std::vector<std::string> joint_name;
    std::vector<double> joint_position;
    std::vector<double> joint_velocity;
    for (size_t i = 0; i < input.size(); ++i)
    {
        joint_name.insert( joint_name.end(), input[i].name.begin(), input[i].name.end());
        joint_position.insert( joint_position.end(), input[i].position.begin(), input[i].position.end() );
        joint_velocity.insert( joint_velocity.end(), input[i].velocity.begin(), input[i].velocity.end() );
    }

    // fill in the output
    output.header.stamp = ros::Time::now();
    output.name = joint_name;
    output.position = joint_position;
    output.velocity = joint_velocity;

    return output.name.size();
}


//#######################################
//###  define the struct JointStates  ###
//#######################################
struct JointStates
{
    sensor_msgs::JointState  current_manipulator_state;
    sensor_msgs::JointState  current_vehicle_state;
    sensor_msgs::JointState  last_manipulator_state;
    sensor_msgs::JointState  last_vehicle_state;
    sensor_msgs::JointState  jointStateBlend;
};


//##########################
//###   main  function   ###
//##########################
int main(int argc, char **argv)
{
    ros::init( argc, argv, "joint_states_blend" );
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle  NodeHandle;
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
                ros::console::g_level_lookup[ros::console::levels::Debug]);

    JointStateHandller  JOINT_STATE_MANIPULATOR_HANDLLER;
    JointStateHandller  JOINT_STATE_VEHICLE_HANDLLER;
    JointStates         joint_state;


    ros::Subscriber  sub_joint_manipulator = NodeHandle.subscribe<sensor_msgs::JointState>(
                INTERFACE_JOINT_STATES_ARM, 3, &JointStateHandller::callback,
                &JOINT_STATE_MANIPULATOR_HANDLLER );
    ros::Subscriber  sub_joint_vehicle = NodeHandle.subscribe<sensor_msgs::JointState>(
                INTERFACE_JOINT_STATES_VEHICLE, 3, &JointStateHandller::callback,
                &JOINT_STATE_VEHICLE_HANDLLER);
    ros::Publisher  pJointState = NodeHandle.advertise<sensor_msgs::JointState>(
                INTERFACE_JOINT_STATES, 3, true);

    // init the joint_state of manipulator and vehicle
    while(ros::ok())
    {
        ros::spinOnce();
        if (JOINT_STATE_MANIPULATOR_HANDLLER.newMsg() && JOINT_STATE_VEHICLE_HANDLLER.newMsg())
        {
            joint_state.current_manipulator_state = JOINT_STATE_MANIPULATOR_HANDLLER.fetchMsg();
            joint_state.current_vehicle_state = JOINT_STATE_VEHICLE_HANDLLER.fetchMsg();
            joint_state.last_manipulator_state = JOINT_STATE_MANIPULATOR_HANDLLER.fetchMsg();
            joint_state.last_vehicle_state = JOINT_STATE_VEHICLE_HANDLLER.fetchMsg();;
            break;
        }
        ros::Duration(0.5).sleep();
        ROS_INFO("waiting for joint state update.");
    }

    // main loop start
    ROS_INFO( "main loop starts" );
    while(ros::ok())
    {
        double start = ros::Time::now().toSec();

        // handle the condition that the communication is connected
        if (JOINT_STATE_MANIPULATOR_HANDLLER.newMsg() || JOINT_STATE_VEHICLE_HANDLLER.newMsg())
        {
            // get the manipulator states
            if (JOINT_STATE_MANIPULATOR_HANDLLER.newMsg())
            {
                joint_state.current_manipulator_state = JOINT_STATE_MANIPULATOR_HANDLLER.fetchMsg();
                joint_state.last_manipulator_state = joint_state.current_manipulator_state;
            }
            else
            {
                joint_state.current_manipulator_state = joint_state.last_manipulator_state;
                ROS_INFO("can not get the joint_states of manipulator.");
            }

            // get the vehicle states
            if (JOINT_STATE_VEHICLE_HANDLLER.newMsg())
            {
                joint_state.current_vehicle_state = JOINT_STATE_VEHICLE_HANDLLER.fetchMsg();
                joint_state.last_vehicle_state = joint_state.current_vehicle_state;
            }
            else
            {
                joint_state.current_vehicle_state = joint_state.last_vehicle_state;
                ROS_INFO("can not get the joint_states of vehicle.");
            }

            // blend and publish the message
            std::vector<sensor_msgs::JointState> input_state;
            input_state.push_back( joint_state.current_manipulator_state );
            input_state.push_back( joint_state.current_vehicle_state );

            sensor_msgs::JointState output;
            int blend_num = BlendJointStates( input_state, output );
            if (blend_num <= 0)
                ROS_ERROR("can not get a normal blend response");
            else
                ROS_INFO("get a message with %d joint states", blend_num);

            pJointState.publish( output );
        }

        // handle the condition that the communication is disconnected
        else
            ROS_INFO("waiting for joint state update.");

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
