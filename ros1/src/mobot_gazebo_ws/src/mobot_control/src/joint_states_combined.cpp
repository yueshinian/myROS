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

#define     STATE_LOOP                  0.02
#define     JOINT_WHEEL_NUM             4

#define     JOINT_STATES_VEHICLE        "/mobot_vehicle/joint_states"
#define     JOINT_STATES_ODOMETRY       "/mobot_wheel/joint_states"
#define     JOINT_STATES_COMBINED       "/joint_states_combined"


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
    // 
    if (input.size() <= 0)
        ROS_ERROR("ERROR <BlendJointStates>: the input vector has no availuable data.");
    // 
    std::vector<std::string> joint_name;
    std::vector<double> joint_position;
    std::vector<double> joint_velocity;
    for (size_t i = 0; i < input.size(); ++i)
    {
        joint_name.insert( joint_name.end(), input[i].name.begin(), input[i].name.end());
        joint_position.insert( joint_position.end(), input[i].position.begin(), input[i].position.end() );
        joint_velocity.insert( joint_velocity.end(), input[i].velocity.begin(), input[i].velocity.end() );
    }
    // 
    output.header.stamp = ros::Time::now();
    output.name = joint_name;
    output.position = joint_position;
    output.velocity = joint_velocity;

    return output.name.size();
}


//##############################################
//###  define the function GenerateVelocity  ###
//##############################################
int GenerateVelocity(sensor_msgs::JointState _in current_state,
                     sensor_msgs::JointState _in last_state,
                     sensor_msgs::JointState _out output)
{
    if ((current_state.name.size()<=0) || (last_state.name.size()<=0))
    {
        ROS_ERROR("ERROR <GenerateVelocity>: the input vector has no availuable data.");
        return -1;
    }
    if ( current_state.name.size() != last_state.name.size() )
    {
        ROS_ERROR("ERROR <GenerateVelocity>: the input ststes don't have equal number of members.");
        return -1;
    }
    // 
    if (current_state.header.stamp.toSec() == last_state.header.stamp.toSec())
    {
        output = current_state;
    }
    else
    {
        output = current_state;
        output.velocity.resize( current_state.name.size() );
        int count = 0;
        for (int i=0; i<current_state.name.size(); ++i)
        {
            for (int j=0; j<last_state.name.size(); ++j)
            {
                if (current_state.name[i] == last_state.name[j])
                {
                    output.velocity[i] = (current_state.position[i] - last_state.position[j]) / (current_state.header.stamp.toSec() - last_state.header.stamp.toSec());
                    count++;
                }
            }
        }
        //
        if (count != current_state.name.size())
        {
            ROS_ERROR("ERROR <GenerateVelocity>: cannot recognize all the joint members.");
            return -1;
        }
    }
    //
    return 1;
}


//#######################################
//###  define the struct JointStates  ###
//#######################################
struct JointStates
{
    sensor_msgs::JointState  current_vehicle_state;
    sensor_msgs::JointState  current_odometry_state;
    sensor_msgs::JointState  last_vehicle_state;
    sensor_msgs::JointState  last_odometry_state;
    sensor_msgs::JointState  jointStateCombined;
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

    JointStateHandller  JOINT_STATES_VEHICLE_HANDLLER;
    JointStateHandller  JOINT_STATES_ODOMETRY_HANDLLER;
    JointStates         joint_state;

    std::vector<sensor_msgs::JointState> s_Last_Odom_JointStates( JOINT_WHEEL_NUM );

    ros::Subscriber  subJointStateVehicle = NodeHandle.subscribe<sensor_msgs::JointState>(
                JOINT_STATES_VEHICLE, 3, &JointStateHandller::callback,
                &JOINT_STATES_VEHICLE_HANDLLER );
    ros::Subscriber  subJointStateOdometry = NodeHandle.subscribe<sensor_msgs::JointState>(
                JOINT_STATES_ODOMETRY, 3, &JointStateHandller::callback,
                &JOINT_STATES_ODOMETRY_HANDLLER);
    ros::Publisher  pubJointState = NodeHandle.advertise<sensor_msgs::JointState>(
                JOINT_STATES_COMBINED, 3, true);
    // 
    while(ros::ok())
    {
        ros::spinOnce();
        if (JOINT_STATES_VEHICLE_HANDLLER.newMsg() && JOINT_STATES_ODOMETRY_HANDLLER.newMsg())
        {
            joint_state.current_vehicle_state = JOINT_STATES_VEHICLE_HANDLLER.fetchMsg();
            joint_state.current_odometry_state = JOINT_STATES_ODOMETRY_HANDLLER.fetchMsg();
            joint_state.last_vehicle_state = JOINT_STATES_VEHICLE_HANDLLER.fetchMsg();
            joint_state.last_odometry_state = JOINT_STATES_ODOMETRY_HANDLLER.fetchMsg();
            break;
        }
        ros::Duration(0.15).sleep();
        ROS_DEBUG("waiting for joint state update.");
    }

    // main loop start
    ROS_INFO( "main loop starts" );
    while(ros::ok())
    {
        double start = ros::Time::now().toSec();
        ros::spinOnce();

        // handle the condition that the communication is connected
        if (JOINT_STATES_VEHICLE_HANDLLER.newMsg() || JOINT_STATES_ODOMETRY_HANDLLER.newMsg())
        {
            if (JOINT_STATES_VEHICLE_HANDLLER.newMsg())
            {
                joint_state.current_vehicle_state = JOINT_STATES_VEHICLE_HANDLLER.fetchMsg();
                joint_state.last_vehicle_state = joint_state.current_vehicle_state;
            }
            else
            {
                joint_state.current_vehicle_state = joint_state.last_vehicle_state;
                ROS_DEBUG("can not get the joint_states of vehicle.");
            }
	    //
            if (JOINT_STATES_ODOMETRY_HANDLLER.newMsg())
            {
                joint_state.current_odometry_state = JOINT_STATES_ODOMETRY_HANDLLER.fetchMsg();
                sensor_msgs::JointState output_state;
                int resp = GenerateVelocity( joint_state.current_odometry_state,
                                             joint_state.last_odometry_state,
                                             output_state);
                if (resp == 1)
                {
                    joint_state.current_odometry_state = output_state;
                    joint_state.last_odometry_state = joint_state.current_odometry_state;
                }
                else
                {
                    joint_state.current_odometry_state = joint_state.last_odometry_state;
                }
            }
            else
            {
                joint_state.current_odometry_state = joint_state.last_odometry_state;
                ROS_DEBUG("can not get the joint_states of odometry.");
            }
            // 
            std::vector<sensor_msgs::JointState> input_state;
            input_state.push_back( joint_state.current_vehicle_state );
            input_state.push_back( joint_state.current_odometry_state );

            sensor_msgs::JointState output;
            int blend_num = BlendJointStates( input_state, output );
            if (blend_num <= 0)
                ROS_ERROR("can not get a normal blend response");
            else
                ROS_DEBUG("get a message with %d joint states", blend_num);

            pubJointState.publish( output );
        }

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
