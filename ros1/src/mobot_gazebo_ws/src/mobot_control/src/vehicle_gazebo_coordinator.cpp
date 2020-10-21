#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <log4cxx/logger.h>

#include "mobot_control/InterfaceJointVel.h"
#include "mobot_control/SwingControl.h"

#include <vector>
#include <list>
#include <iostream>
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>


typedef     const char*     CStrPtr;

//--------------------------------------------------------------------------------
// 
#define     NAVIGATION_COMMAND              "/navigation_command"          
#define     SWINGARM_COMMAND                "/swingarm_command"          
#define     INTERFACE_STATES_VEHICLE        "/interface_states_vehicle"    

#define     INTERFACE_JOINT_STATES          "/mobot_vehicle/joint_states"   
#define     ODOMETRY_JOINT_STATES           "/mobot_wheel/joint_states"   

#define     VEHICLE_STATES                  "/vehicle_states"              
#define     VEHICLE_ODOMETRY                "/vehicle_odometry"            
#define     INTERFACE_COMMAND_VEHICLE       "/interface_command_vehicle"   
#define     INTERFACE_VELOCITY_VEHICLE      "/interface_velocity_vehicle"  
//
#define     INTERFACE_STATE_SPARATER        "\t"
#define     INTERFACE_STATE_BASE_MOVING     "BASE_MOVING"
#define     INTERFACE_STATE_BASE_STABLE     "BASE_STABLE"
#define     INTERFACE_STATE_SWING_MOVING    "SWING_MOVING"
#define     INTERFACE_STATE_SWING_STABLE    "SWING_STABLE"
#define     INTERFACE_STATE_VELOCITY        "VELOCITY"
#define     INTERFACE_STATE_HOME            "HOME"
#define     INTERFACE_STATE_RESET           "RESET"
//
#define     LINK_LEFT_SWINGARM              "swingarm_left"
#define     LINK_RIGHT_SWINGARM             "swingarm_right"
#define     LINK_LEFT_FRONT_WHEEL           "wheel_lf"
#define     LINK_RIGHT_FRONT_WHEEL          "wheel_rf"
#define     LINK_LEFT_BACK_WHEEL            "wheel_lb"
#define     LINK_RIGHT_BACK_WHEEL           "wheel_rb"

#define     JOINT_VEHICLE_NUM               6
#define     JOINT_SWINGARM_NUM              2
#define     JOINT_WHEEL_NUM                 4

#define     JOINT_LEFT_SWINGARM             "joint_swingarm_left"
#define     JOINT_RIGHT_SWINGARM            "joint_swingarm_right"
#define     JOINT_LEFT_FRONT_WHEEL          "joint_wheel_lf"
#define     JOINT_RIGHT_FRONT_WHEEL         "joint_wheel_rf"
#define     JOINT_LEFT_BACK_WHEEL           "joint_wheel_lb"
#define     JOINT_RIGHT_BACK_WHEEL          "joint_wheel_rb"

#define     VEHICLE_BASE_LINK               "base_link"
#define     VEHICLE_FIXED_FRAME             "vehicle_odom"
#define     VEHICLE_ODOMETRY_FRAME          "base_footprint"

std::string VEHICLE_JOINT[] = { JOINT_LEFT_SWINGARM,
                                JOINT_RIGHT_SWINGARM,
                                JOINT_LEFT_FRONT_WHEEL,
                                JOINT_RIGHT_FRONT_WHEEL,
                                JOINT_LEFT_BACK_WHEEL,
                                JOINT_RIGHT_BACK_WHEEL };
// 
#define     SWING_CMD_MOVE                  "SWINGMOVE"
#define     SWING_CMD_RESET                 "SWINGRESET"
// 
#define     LABEL_STOP              0
#define     LABEL_SLOW              2
#define     LABEL_MIDDLE            3
#define     LABEL_FAST              4

#define     VELOCITY_STOP           0.00
#define     VELOCITY_SLOW           0.20
#define     VELOCITY_MIDDLE         0.35
#define     VELOCITY_FAST           0.50
// 
#define     VEHICLE_COORD_LOOP      0.020
#define     VEHICLE_GMAS_LOOP       0.020
#define     ROBOT_WIDTH             0.8
#define     ROBOT_RADIUS            0.175

#define     _in     const &
#define     _inc    const
#define     _out          &
#define     _ret    const
#define     _wt           &
#define     _rd     const &

//--------------------------------------------------------------------------------
//#######################################################
//###  1. define the structure - Parameter and State  ###
//#######################################################
struct Parameter
{
    double                      loop_time;
    double                      gmas_time;
    std::vector<double>         max_vel;
    Eigen::VectorXd             max_joint_disp;
    std::vector<std::string>    joint_names;
    std::string                 base;
};

struct State
{
    sensor_msgs::JointState         odomJointState;
    mobot_control::SwingControl     swingControl;
    std::string                     interfaceState;

    double                      left_wheel_rotation;
    double                      right_wheel_rotation;

    std::string                 vehicleMode;
    std::string                 swingState;
    bool                        newCommand;
    bool                        newJointVel;

    ros::Subscriber             subSwingCmd;        
    ros::Subscriber             subNavigationCmd;     
    ros::Subscriber             subSwingJointStates;  
    ros::Subscriber             subOdomJointStates;   
    ros::Subscriber             subInterfaceStates;    
};

struct Actor
{
    double                      ref_left_rotation; 
    double                      ref_right_rotation; 

    double                      ref_x;
    double                      ref_y;
    double                      ref_theta;

    ros::Time                           current_time;
    ros::Time                           last_time;
    std_msgs::String                    interface_command;
    mobot_control::InterfaceJointVel    interface_joint_vel;

    ros::Publisher              pInterfaceCommand; 
    ros::Publisher              pInterfaceVelocity; 
    ros::Publisher              pVehicleStates;     
    ros::Publisher              pVehicleOdometry;  
};

//################################################
//###  2. define the class StringHandllerOnce  ###
//################################################
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

//################################################
//###  3. define the class JointStateHandller  ###
//################################################
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

//##################################################
//###  4. define the class SwingCommandHandller  ###
//##################################################
class SwingCommandHandller
{
public:
    void callback(const mobot_control::SwingControl::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    SwingCommandHandller()
        : mCrtMsg()
        , mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const mobot_control::SwingControl & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    mobot_control::SwingControl mCrtMsg;
    bool mNewMsg;
};

//#################################################
//###  5. define the class NavOdometryHandller  ###
//#################################################
class NavCommandHandller
{
public:
    void callback(const geometry_msgs::Twist::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    NavCommandHandller()
        : mCrtMsg()
        , mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const geometry_msgs::Twist & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    geometry_msgs::Twist mCrtMsg;
    bool mNewMsg;
};

//#############################################
//###  6. define the StringAppend Function  ###
//#############################################
std::string _wt StringAppend( std::string _out output, CStrPtr _inc format, ... )
{
    const std::string::size_type write_point = output.size();
    while(true)
    {
        std::string::size_type remaining = output.capacity() - write_point;
        output.resize ( output.capacity() );
        va_list ap;
        va_start ( ap, format );
        int bytes_used = vsnprintf ( &output[write_point], remaining, format, ap );
        va_end ( ap );
        if ( bytes_used < 0 )
        {
            if ( output.size() + 128 > 1024 )
            {
                throw std::runtime_error (
                std::string ( "vsnprint retry did not manage to work " ) );
            }
            output.resize ( output.size() + 128 );
        }
        else if ( bytes_used < remaining )
        {
            output.resize ( write_point + bytes_used );
            break;
        }
        else
        {
            if ( write_point + bytes_used + 1 > 1024 )
            {
                throw std::runtime_error (
                            std::string ( "vsnprint retry did not manage to work " ) );
            }
            output.resize ( write_point + bytes_used + 1 );
        }
    }
    return output;
}


//##########################
//###  7. main function  ###
//##########################
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_coordinator");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle         Node_Handle;

    // 1>>1. init the topic-subscribed handllers
    StringHandllerOnce      INTERFACE_STATE_HANDLLER;
    JointStateHandller      SWING_JOINT_STATE_HANDLLER;
    JointStateHandller      ODOMETRY_JOINT_STATE_HANDLLER;
    SwingCommandHandller    SWING_COMMAND_HANDLLER;

    // 1>>2. init the Parameter struct
    ROS_INFO ( "waiting for data input." );
    Parameter   params;
    params.loop_time = VEHICLE_COORD_LOOP;
    params.gmas_time = VEHICLE_GMAS_LOOP;
    params.base = VEHICLE_BASE_LINK;
    std::cout << "vehicle base link: " << params.base << "\t";

    params.joint_names.resize( JOINT_VEHICLE_NUM );
    for (size_t i = 0; i < JOINT_VEHICLE_NUM; ++i)
    {
        params.joint_names[i] = VEHICLE_JOINT[i];
        std::cout << "joint_" << i+1 << ": " << params.joint_names[i] << "\t";
    }
    std::cout << std::endl;

    params.max_vel.resize( params.joint_names.size() );
    for (size_t i = 0; i < params.joint_names.size(); ++i)
    {
        params.max_vel[i] = 45.0 / 180.0 * 3.1415926;
        ROS_INFO ( "vehicle max velocity of joint [%d] = %f deg/s", i+1, params.max_vel[i]/3.1415926*180.0 );
    }

    params.max_joint_disp.resize( params.joint_names.size() );
    for (size_t i = 0; i < params.joint_names.size(); ++i)
    {
        params.max_joint_disp(i) = params.max_vel[i] * params.gmas_time;
        ROS_INFO( "vehicle max displacement of joint [%d] = %f deg", i+1, params.max_joint_disp(i)/3.1415926*180.0 );
    }

    // 1>>3. init the State struct
    State       state;
    state.swingState  = "";
    state.vehicleMode = "";
    state.newCommand  = false;
    state.newJointVel = false;

    state.subInterfaceStates =
            Node_Handle.subscribe<std_msgs::String>(
                INTERFACE_STATES_VEHICLE, 3, &StringHandllerOnce::callback,
                &INTERFACE_STATE_HANDLLER);
    state.subSwingCmd =
            Node_Handle.subscribe<mobot_control::SwingControl>(
                SWINGARM_COMMAND, 3, &SwingCommandHandller::callback,
                &SWING_COMMAND_HANDLLER);
    state.subSwingJointStates =
            Node_Handle.subscribe<sensor_msgs::JointState>(
                INTERFACE_JOINT_STATES, 3, &JointStateHandller::callback,
                &SWING_JOINT_STATE_HANDLLER);
    state.subOdomJointStates =
            Node_Handle.subscribe<sensor_msgs::JointState>(
                ODOMETRY_JOINT_STATES, 3, &JointStateHandller::callback,
                &ODOMETRY_JOINT_STATE_HANDLLER);

    // 1>>4. init the Actor struct
    Actor actor;
    actor.ref_x     = 0.0;
    actor.ref_y     = 0.0;
    actor.ref_theta = 0.0;
    actor.current_time = ros::Time::now();
    actor.last_time = ros::Time::now();

    actor.interface_command.data = "";
    actor.interface_joint_vel.joint_names.resize( JOINT_SWINGARM_NUM );
    actor.interface_joint_vel.command.resize( JOINT_SWINGARM_NUM );
    for (size_t i = 0; i < JOINT_SWINGARM_NUM; ++i)
        actor.interface_joint_vel.joint_names[i] = VEHICLE_JOINT[i];

    actor.pInterfaceCommand =
            Node_Handle.advertise<std_msgs::String>(
                INTERFACE_COMMAND_VEHICLE, 3, false);
    actor.pInterfaceVelocity =
            Node_Handle.advertise<mobot_control::InterfaceJointVel>(
                INTERFACE_VELOCITY_VEHICLE, 3, false);
    actor.pVehicleOdometry =
            Node_Handle.advertise<nav_msgs::Odometry>(
                VEHICLE_ODOMETRY, 1024, false);
    actor.pVehicleStates =
            Node_Handle.advertise<std_msgs::String>(
                VEHICLE_STATES, 1, false);

    // 1>>5. define the tf broadcaster, broadcasting the transform between "/odom" and "/base_footprint"
    tf::TransformBroadcaster    broadcaster;

    // 1>>6. init the jointState and wheel rotations in struct state and actor
    while(ros::ok())
    {
        ros::spinOnce();
        if ( ODOMETRY_JOINT_STATE_HANDLLER.newMsg() && SWING_JOINT_STATE_HANDLLER.newMsg() )
        {
            state.odomJointState = ODOMETRY_JOINT_STATE_HANDLLER.fetchMsg();
            for (size_t i = 0; i < state.odomJointState.name.size(); ++i)
            {
                if ( state.odomJointState.name[i] == JOINT_LEFT_FRONT_WHEEL )
                {
                    state.left_wheel_rotation = state.odomJointState.position[i];
                    actor.ref_left_rotation = state.odomJointState.position[i];
                }
                else if ( state.odomJointState.name[i] == JOINT_RIGHT_FRONT_WHEEL )
                {
                    state.right_wheel_rotation = state.odomJointState.position[i];
                    actor.ref_right_rotation = state.odomJointState.position[i];
                }
            }
            break;
        }
        ros::Duration(5.0).sleep();
        ROS_INFO("waiting for joint state update.");
    }

    // 2. start the main loop
    ROS_INFO("main loop start");

    while(ros::ok())
    {
        try
        {
            double start = ros::Time::now().toSec();
            ros::spinOnce();
	    //
            double start_1 = ros::Time::now().toSec();
            bool interface_velocity = false;
            bool interface_stable   = false;
            if ( INTERFACE_STATE_HANDLLER.newMsg() )
            {
                ROS_DEBUG("[part_1] get the interface states.");
                std_msgs::String  input = INTERFACE_STATE_HANDLLER.fetchMsg();
                state.interfaceState = input.data;

                std::vector<std::string> result;
                boost::split( result, state.interfaceState, boost::is_any_of("\t"),
                              boost::algorithm::token_compress_on );
                if (result.size() != 3)
                    ROS_ERROR("split the interface states message failed, split_size[%d].", result.size());

                state.vehicleMode = result[0];
                state.swingState  = result[1];

                if (state.swingState == INTERFACE_STATE_SWING_STABLE)
                    interface_stable = true;
                if (state.vehicleMode == INTERFACE_STATE_VELOCITY)
                    interface_velocity = true;
            }
            double cost_1 = ros::Time::now().toSec() - start_1;
            ROS_DEBUG("1. INTERFACE_STATE_HANDLLER end. cost_time: %lf ms.", cost_1*1000);

            // 
            double start_2 = ros::Time::now().toSec();
            actor.interface_joint_vel.command.clear();
            actor.interface_joint_vel.command.resize( JOINT_SWINGARM_NUM );
            if ( SWING_COMMAND_HANDLLER.newMsg() )
            {
                ROS_DEBUG("[part_2] get the control commands.");
                state.newCommand   = false;
                state.newJointVel  = false;
                state.swingControl = SWING_COMMAND_HANDLLER.fetchMsg();
                std::string  input_command = state.swingControl.commandID;
                //
                if (interface_velocity && interface_stable)
                {
                    if (input_command == SWING_CMD_RESET) // "SWINGRESET"
                    {
                        actor.interface_command.data = SWING_CMD_RESET;
                        actor.pInterfaceCommand.publish( actor.interface_command );
                        state.newCommand = true;
                    }
                }
                // 
                if (interface_velocity)
                {
                    bool motion_flag = true;
                    if (input_command == SWING_CMD_MOVE) 
                    {
                        for (size_t i = 0; i < JOINT_SWINGARM_NUM; ++i)
                        {
                            switch( (int)fabs(state.swingControl.data[i]) )
                            {
                                case LABEL_STOP:
                                {
                                    actor.interface_joint_vel.command[i] = VELOCITY_STOP;
                                    break;
                                }
                                case LABEL_SLOW:
                                {
                                    actor.interface_joint_vel.command[i] = VELOCITY_SLOW;
                                    break;
                                }
                                case LABEL_MIDDLE:
                                {
                                    actor.interface_joint_vel.command[i] = VELOCITY_MIDDLE;
                                    break;
                                }
                                case LABEL_FAST:
                                {
                                    actor.interface_joint_vel.command[i] = VELOCITY_FAST;
                                    break;
                                }
                                default:
                                {
                                    ROS_ERROR("can not explain the SwingControl 'MOVE' command.");
                                    motion_flag = false;
                                    break;
                                }
                            }
                            if ( state.swingControl.data[i] >= 0.0 )
                                actor.interface_joint_vel.command[i] *= 1.0;
                            else
                                actor.interface_joint_vel.command[i] *= -1.0;
                        }
                    }
                    if (motion_flag)
                    {
                        state.newJointVel = true;
                        actor.pInterfaceVelocity.publish( actor.interface_joint_vel );
                    }
                }
            }
            double cost_2 = ros::Time::now().toSec() - start_2;
            ROS_DEBUG("2. SWING_COMMAND_HANDLLER and NAV_COMMAND_HANDLLER end. cost_time: %lf ms.", cost_2*1000);

            double start_3 = ros::Time::now().toSec();
            actor.current_time = ros::Time::now();
            if ( ODOMETRY_JOINT_STATE_HANDLLER.newMsg() )
            {
                ROS_DEBUG("[part_3] get the odometry joint states.");
                state.odomJointState = ODOMETRY_JOINT_STATE_HANDLLER.fetchMsg();
                for (size_t i = 0; i < state.odomJointState.name.size(); ++i)
                {
                    if ( state.odomJointState.name[i] == JOINT_LEFT_FRONT_WHEEL )
                        state.left_wheel_rotation = state.odomJointState.position[i];
                    else if ( state.odomJointState.name[i] == JOINT_RIGHT_FRONT_WHEEL )
                        state.right_wheel_rotation = state.odomJointState.position[i];
                }

                double  delta_left_wheel = (state.left_wheel_rotation - actor.ref_left_rotation) * ROBOT_RADIUS;
                double  delta_right_wheel = (state.right_wheel_rotation - actor.ref_right_rotation) * ROBOT_RADIUS;
                double  delta_x = (delta_right_wheel + delta_left_wheel) / 2.0;
                double  delta_th = (delta_right_wheel - delta_left_wheel) / ROBOT_WIDTH;
                double  vx = delta_x / (actor.current_time-actor.last_time).toSec();
                double  vth = delta_th / (actor.current_time-actor.last_time).toSec();

                actor.ref_left_rotation = state.left_wheel_rotation;
                actor.ref_right_rotation = state.right_wheel_rotation;

                actor.ref_x = actor.ref_x + delta_x * cos(actor.ref_theta);
                actor.ref_y = actor.ref_y + delta_x * sin(actor.ref_theta);
                actor.ref_theta = actor.ref_theta + delta_th;
                geometry_msgs::Quaternion odom_quat;
                odom_quat = tf::createQuaternionMsgFromYaw( actor.ref_theta );
                // 
                geometry_msgs::TransformStamped odom_transform;
                odom_transform.header.stamp = actor.current_time;
                odom_transform.header.frame_id = VEHICLE_FIXED_FRAME;
                odom_transform.child_frame_id = VEHICLE_ODOMETRY_FRAME;

                odom_transform.transform.translation.x = actor.ref_x;
                odom_transform.transform.translation.y = actor.ref_y;
                odom_transform.transform.translation.z = 0.0;
                odom_transform.transform.rotation = odom_quat;
                //
                nav_msgs::Odometry  odom_navigation;
                odom_navigation.header.stamp = actor.current_time;
                odom_navigation.header.frame_id = VEHICLE_FIXED_FRAME;
                odom_navigation.child_frame_id = VEHICLE_ODOMETRY_FRAME;

                odom_navigation.pose.pose.position.x  = actor.ref_x;
                odom_navigation.pose.pose.position.y  = actor.ref_y;
                odom_navigation.pose.pose.position.z  = 0.0;
                odom_navigation.pose.pose.orientation = odom_quat;

                odom_navigation.twist.twist.linear.x  = vx;
                odom_navigation.twist.twist.linear.y  = 0.0;
                odom_navigation.twist.twist.linear.z  = 0.0;
                odom_navigation.twist.twist.angular.x = 0.0;
                odom_navigation.twist.twist.angular.y = 0.0;
                odom_navigation.twist.twist.angular.y = vth;
		//
                actor.pVehicleOdometry.publish( odom_navigation );
                actor.last_time = actor.current_time;

            }
            double cost_3 = ros::Time::now().toSec() - start_3;
            ROS_DEBUG("3. JOINT_STATE_HANDLLER end. cost_time: %lf ms.", cost_3*1000);

            // 
            double start_4 = ros::Time::now().toSec();
            ROS_DEBUG("[part_4] publish coordinator states.");

            std_msgs::String    stateoutput;
            std::string         stateout;
            if ( state.newCommand )
            {
                StringAppend(stateout, "new_command: ");
                StringAppend(stateout, "%s ", actor.interface_command.data.c_str());
            }
            else if (state.newJointVel)
            {
                StringAppend( stateout, "new_joint_velocity: " );
                StringAppend( stateout, "%f ", actor.interface_joint_vel.command[0] );
                StringAppend( stateout, "%f ", actor.interface_joint_vel.command[1] );
            }
            else
            {
                StringAppend( stateout, "%s ", state.vehicleMode.c_str() );
                StringAppend( stateout, "%s ", state.swingState.c_str() );
            }
            ROS_DEBUG( "%s", stateout.c_str() );
            stateoutput.data = stateout;
            actor.pVehicleStates.publish( stateoutput );

            double cost_4 = ros::Time::now().toSec() - start_4;
            ROS_DEBUG("4.  vehicle state_output end. cost_time: %lf ms.", cost_4*1000);


            // time control
            double sleep_time = params.loop_time - ( ros::Time::now().toSec() - start );
            if ( sleep_time > 0 )
            {
                ros::Duration ( sleep_time ).sleep();
                ROS_DEBUG( "total cost time: %lf ms", (params.loop_time-sleep_time)*1000 );
            }
            else
                ROS_WARN( "control loop over time: %f ms", -sleep_time*1000 );
        }
        catch (...)
        {
            ROS_ERROR("Maybe something is wrong!");
        }
    }
    ros::waitForShutdown();
    return 0;
}


