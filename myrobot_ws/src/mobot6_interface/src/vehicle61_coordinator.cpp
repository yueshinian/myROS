#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <log4cxx/logger.h>


#include "mobot6_interface/InterfaceJointVel.h"
#include "mobot6_interface/SwingControl.h"

#include <vector>
#include <list>
#include <iostream>
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>


typedef     const char*     CStrPtr;

//--------------------------------------------------------------------------------

// 1. define the corresponding topics
#define     NAVIGATION_COMMAND              "/navigation_command"           // keyboard pub, coord sub
#define     SWINGARM_COMMAND                "/swingarm_command"             // keyboard pub, coord sub
#define     INTERFACE_JOINT_STATES_VEHICLE  "/joint_states_vehicle"         // inter pub, coord sub
#define     INTERFACE_STATES_VEHICLE        "/interface_states_vehicle"     // inter pub, coord sub
#define     IMU_STATES_VEHICLE              "/imu_data"                     // imu pub, coord sub

#define     VEHICLE_STATES                  "/vehicle_states"               // coord pub
#define     VEHICLE_ODOMETRY                "/odom"                         // coord pub  "/vehicle_odometry"
#define     INTERFACE_COMMAND_VEHICLE       "/interface_command_vehicle"    // coord pub, inter sub
#define     INTERFACE_VELOCITY_VEHICLE      "/interface_velocity_vehicle"   // coord pub, inter sub

// 2. define the states of interface node
#define     INTERFACE_STATE_SPARATER        "\t"
#define     INTERFACE_STATE_BASE_MOVING     "BASE_MOVING"
#define     INTERFACE_STATE_BASE_STABLE     "BASE_STABLE"
#define     INTERFACE_STATE_SWING_MOVING    "SWING_MOVING"
#define     INTERFACE_STATE_SWING_STABLE    "SWING_STABLE"
#define     INTERFACE_STATE_VELOCITY        "VELOCITY"
#define     INTERFACE_STATE_HOME            "HOME"
#define     INTERFACE_STATE_RESET           "RESET"

// 3. define the active links and joints of vehicle
#define     LINK_LEFT_SWINGARM              "swingarm_left"
#define     LINK_RIGHT_SWINGARM             "swingarm_right"
#define     LINK_LEFT_WHEEL                 "fake_left_wheel"
#define     LINK_RIGHT_WHEEL                "fake_right_wheel"

#define     JOINT_VEHICLE_NUM               4
#define     JOINT_LEFT_SWINGARM             "joint_swingarm_left"
#define     JOINT_RIGHT_SWINGARM            "joint_swingarm_right"
#define     JOINT_LEFT_WHEEL                "base_l_wheel_joint"
#define     JOINT_RIGHT_WHEEL               "base_r_wheel_joint"

#define     VEHICLE_BASE_LINK               "base_link"
#define     VEHICLE_FIXED_FRAME             "odom"
#define     VEHICLE_ODOMETRY_FRAME          "base_footprint"

std::string VEHICLE_JOINT[] = {JOINT_LEFT_SWINGARM, JOINT_RIGHT_SWINGARM, JOINT_LEFT_WHEEL, JOINT_RIGHT_WHEEL};

// 4. define the input commands of preprocess node
//#define     SWING_CMD_STOP                  "SWINGSTOP"
#define     SWING_CMD_MOVE                  "SWINGMOVE"
#define     SWING_CMD_HOME                  "SWINGHOME"
#define     SWING_CMD_HOMESTOP              "SWINGHOMESTOP"
#define     SWING_CMD_RESET                 "SWINGRESET"

// 5. define the commands of interface node
#define     SWINGARM_DEATH          5.0    // degree
#define     SWINGARM_INTERVAL       10.0    // degree
#define     POSITIVE_DIRECTION      1
#define     NEGATIVE_DIRECTION      -1
#define     BASE_DEATH              1.0    // degree
#define     BASE_INTERVAL           1.0    // degree

// 6. define the other macros
#define     VEHICLE_COORD_LOOP      0.050
#define     VEHICLE_GMAS_LOOP       0.025
#define     ROBOT_WIDTH             0.8
#define     ROBOT_RADIUS            0.175

#define     _in     const &
#define     _inc    const
#define     _out          &
#define     _ret    const
#define     _wt           &
#define     _rd     const &

//--------------------------------------------------------------------------------
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
    sensor_msgs::JointState         jointState;
    geometry_msgs::Twist            navControl;
    mobot6_interface::SwingControl   swingControl;
    std::string                     interfaceState;

    double                      left_wheel_rotation;
    double                      right_wheel_rotation;

    std::string                 vehicleMode;
    std::string                 swingState;
    std::string                 baseState;
    bool                        newCommand;
    bool                        newJointVel;

    sensor_msgs::Imu            imuState;
    bool                        newImuState;

    ros::Subscriber             subSwingCmd;        // "/swingarm_command"
    ros::Subscriber             subNavigationCmd;   // "/navigation_command"
    ros::Subscriber             subJointStates;     // "/joint_states_vehicle"
    ros::Subscriber             subInterfaceStates; // "/interface_states_vehicle"
    ros::Subscriber             subImuStates;       // "/imu_data"
};

struct Actor
{
    double                      ref_left_rotation;  // last ratation of left wheel
    double                      ref_right_rotation; // last rotation of right wheel

    double                      ref_x;
    double                      ref_y;
    double                      ref_theta;

    ros::Time                           current_time;
    ros::Time                           last_time;
    std_msgs::String                    interface_command;
    mobot6_interface::InterfaceJointVel  interface_joint_vel;

    ros::Publisher              pInterfaceCommand;  // "/interface_command_vehicle"
    ros::Publisher              pInterfaceVelocity; // "/interface_velocity_vehicle"
    ros::Publisher              pVehicleStates;     // "/vehicle_states"
    ros::Publisher              pVehicleOdometry;   // /vehicle_odometry"
};

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

class ImuStateHandller  // New Add
{
public:
    void callback(const sensor_msgs::Imu::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    ImuStateHandller()
        : mCrtMsg()
        , mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const sensor_msgs::Imu & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    sensor_msgs::Imu mCrtMsg;
    bool mNewMsg;
};

class SwingCommandHandller
{
public:
    void callback(const mobot6_interface::SwingControl::ConstPtr & message)
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

    const mobot6_interface::SwingControl & fetchMsg()
    {
        mNewMsg = false;
        //ROS_WARN("swingarm command - fetchMsg successfully???");
        return mCrtMsg;
    }

protected:
    mobot6_interface::SwingControl mCrtMsg;
    bool mNewMsg;
};

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
        //ROS_WARN("navigation command - fetchMsg successfully???");
        return mCrtMsg;
    }

protected:
    geometry_msgs::Twist mCrtMsg;
    bool mNewMsg;
};

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

double quat2yaw(const geometry_msgs::Quaternion msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    return yaw;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_coordinator");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle         Node_Handle;

    // 
    StringHandllerOnce      INTERFACE_STATE_HANDLLER;
    JointStateHandller      JOINT_STATE_HANDLLER;
    SwingCommandHandller    SWING_COMMAND_HANDLLER;
    NavCommandHandller      NAV_COMMAND_HANDLLER;
    ImuStateHandller        IMU_STATE_HANDLLER;     // New Add

    // 
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

    // 
    State       state;
    state.baseState   = "";
    state.swingState  = "";
    state.vehicleMode = "";
    state.newCommand  = false;
    state.newJointVel = false;
    state.newImuState = false; // New Add

    state.subInterfaceStates =
            Node_Handle.subscribe<std_msgs::String>(
                INTERFACE_STATES_VEHICLE, 3, &StringHandllerOnce::callback,
                &INTERFACE_STATE_HANDLLER);
    state.subSwingCmd =
            Node_Handle.subscribe<mobot6_interface::SwingControl>(
                SWINGARM_COMMAND, 3, &SwingCommandHandller::callback,
                &SWING_COMMAND_HANDLLER);
    state.subNavigationCmd =
            Node_Handle.subscribe<geometry_msgs::Twist>(
                NAVIGATION_COMMAND, 3, &NavCommandHandller::callback,
                &NAV_COMMAND_HANDLLER);
    state.subJointStates =
            Node_Handle.subscribe<sensor_msgs::JointState>(
                INTERFACE_JOINT_STATES_VEHICLE, 3, &JointStateHandller::callback,
                &JOINT_STATE_HANDLLER);
// ------------------------------- New Add ------------------------------- //
    state.subImuStates =
            Node_Handle.subscribe<sensor_msgs::Imu>(
                IMU_STATES_VEHICLE, 3, &ImuStateHandller::callback,
                &IMU_STATE_HANDLLER);
// ------------------------------- New Add ------------------------------- //

    // 
    Actor actor;
    actor.ref_x     = 0.0;
    actor.ref_y     = 0.0;
    actor.ref_theta = 0.0;
    actor.current_time = ros::Time::now();
    actor.last_time = ros::Time::now();

    actor.interface_command.data = "";
    actor.interface_joint_vel.joint_names.resize( JOINT_VEHICLE_NUM );
    actor.interface_joint_vel.command_id.resize( JOINT_VEHICLE_NUM );
    actor.interface_joint_vel.direction.resize( JOINT_VEHICLE_NUM );
    for (size_t i = 0; i < JOINT_VEHICLE_NUM; ++i)
        actor.interface_joint_vel.joint_names[i] = VEHICLE_JOINT[i];

    actor.pInterfaceCommand =
            Node_Handle.advertise<std_msgs::String>(
                INTERFACE_COMMAND_VEHICLE, 3, false);
    actor.pInterfaceVelocity =
            Node_Handle.advertise<mobot6_interface::InterfaceJointVel>(
                INTERFACE_VELOCITY_VEHICLE, 3, false);
    actor.pVehicleOdometry =
            Node_Handle.advertise<nav_msgs::Odometry>(
                VEHICLE_ODOMETRY, 1024, false);
    actor.pVehicleStates =
            Node_Handle.advertise<std_msgs::String>(
                VEHICLE_STATES, 1, false);

    // 
    tf::TransformBroadcaster    broadcaster;

    //  init the jointState and wheel rotations in struct state and actor
    while(ros::ok())
    {
        ros::spinOnce();
        if ( JOINT_STATE_HANDLLER.newMsg() && IMU_STATE_HANDLLER.newMsg() )
        {
            state.jointState = JOINT_STATE_HANDLLER.fetchMsg();
            for (size_t i = 0; i < state.jointState.name.size(); ++i)
            {
                if ( state.jointState.name[i] == JOINT_LEFT_WHEEL )
                {
                    state.left_wheel_rotation = state.jointState.position[i];
                    actor.ref_left_rotation = state.jointState.position[i];
                }
                else if ( state.jointState.name[i] == JOINT_RIGHT_WHEEL )
                {
                    state.right_wheel_rotation = state.jointState.position[i];
                    actor.ref_right_rotation = state.jointState.position[i];
                }
            }
            state.imuState = IMU_STATE_HANDLLER.fetchMsg(); // New Add
            break;
        }
        else if ( !JOINT_STATE_HANDLLER.newMsg() && IMU_STATE_HANDLLER.newMsg() )
            ROS_INFO("can not get the joint_state, waiting for joint_state update.");
        else if ( JOINT_STATE_HANDLLER.newMsg() && !IMU_STATE_HANDLLER.newMsg() )
            ROS_INFO("can not get the imu_state, waiting for imu_state update.");
        else
            ROS_INFO("waiting for joint_state and imu_state update.");
        ros::Duration(0.5).sleep();
    }

    ROS_INFO("main loop start");
    while(ros::ok())
    {
        try
        {
            double start = ros::Time::now().toSec();
            ros::spinOnce();

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
                state.baseState   = result[2];

                if ((state.baseState == INTERFACE_STATE_BASE_STABLE) && (state.swingState == INTERFACE_STATE_SWING_STABLE))
                    interface_stable = true;
                if (state.vehicleMode == INTERFACE_STATE_VELOCITY)
                    interface_velocity = true;

            }
            double cost_1 = ros::Time::now().toSec() - start_1;
            ROS_DEBUG("1. INTERFACE_STATE_HANDLLER end. cost_time: %lf ms.", cost_1*1000);


            double start_2 = ros::Time::now().toSec();
            actor.interface_joint_vel.command_id.clear();
            actor.interface_joint_vel.command_id.resize( JOINT_VEHICLE_NUM );
            actor.interface_joint_vel.direction.clear();
            actor.interface_joint_vel.direction.resize( JOINT_VEHICLE_NUM );

            if ( SWING_COMMAND_HANDLLER.newMsg() && NAV_COMMAND_HANDLLER.newMsg() )
            {
                ROS_DEBUG("[part_2] get the control commands.");
                state.newCommand   = false;
                state.newJointVel  = false;
                state.swingControl = SWING_COMMAND_HANDLLER.fetchMsg();
                state.navControl   = NAV_COMMAND_HANDLLER.fetchMsg();
                std::string  input_command = state.swingControl.commandID;

                if (interface_velocity && interface_stable)
                {
                    if (input_command == SWING_CMD_HOME) // "SWINGHOME"
                    {
                        actor.interface_command.data = SWING_CMD_HOME;
                        actor.pInterfaceCommand.publish( actor.interface_command );
                        state.newCommand = true;
                    }
                    else if (input_command == SWING_CMD_RESET) // "SWINGRESET"
                    {
                        actor.interface_command.data = SWING_CMD_RESET;
                        actor.pInterfaceCommand.publish( actor.interface_command );
                        state.newCommand = true;
                    }
                }

                if (interface_velocity)
                {
                    bool motion_flag = true;
                    // fill interface command with the swingarm command part
                    if (input_command == SWING_CMD_MOVE) // "SWINGMOVE"
                    {
                        for (size_t i = 0; i < 2; ++i)
                        {
                            double temp_control = state.swingControl.data[i] * 180.0 / 3.1415926;
                            double temp_floor;
                            if ( fabs(temp_control) <= SWINGARM_DEATH )
                            {
                                actor.interface_joint_vel.command_id[i] = 0.0;
                            }
                            else if ( fabs(temp_control) <= SWINGARM_INTERVAL )
                            {
                                actor.interface_joint_vel.command_id[i] = SWINGARM_INTERVAL;
                            }
                            else
                            {
                                temp_floor = floor(fabs(temp_control)/SWINGARM_INTERVAL)*SWINGARM_INTERVAL;
                                actor.interface_joint_vel.command_id[i] = temp_floor;
                            }
                            // determin the control directions
                            if ( temp_control >= 0.0 )
                                actor.interface_joint_vel.direction[i] = POSITIVE_DIRECTION;
                            else
                                actor.interface_joint_vel.direction[i] = NEGATIVE_DIRECTION;
                        }
                    }
                    double  s_left_linear  = state.navControl.linear.x - state.navControl.angular.z*ROBOT_WIDTH/2.0;
                    double  s_right_linear = state.navControl.linear.x + state.navControl.angular.z*ROBOT_WIDTH/2.0;
                    double  s_left_rot  = s_left_linear/ROBOT_RADIUS*180.0/3.1415926;
                    double  s_right_rot = s_right_linear/ROBOT_RADIUS*180.0/3.1415926;
                    double  s_wheel_rot[] = {s_left_rot, s_right_rot};
                    //
                    for (int i = 2; i < 4; ++i)
                    {
                        double temp_floor_rot;
                        if ( fabs(s_wheel_rot[i-2]) <= BASE_DEATH )
                        {
                            actor.interface_joint_vel.command_id[i] = 0.0;
                        }
                        else if ( fabs(s_wheel_rot[i-2]) <= 90)//BASE_INTERVAL )
                        {
                            actor.interface_joint_vel.command_id[i] = fabs(s_wheel_rot[i-2]);//BASE_INTERVAL;
                        }
                        else
                        {
                            temp_floor_rot = floor(fabs(s_wheel_rot[i-2])/BASE_INTERVAL)*BASE_INTERVAL;
                            actor.interface_joint_vel.command_id[i] = temp_floor_rot;
                        }
                        if ( s_wheel_rot[i-2] >= 0.0 )
                            actor.interface_joint_vel.direction[i] = POSITIVE_DIRECTION;
                        else
                            actor.interface_joint_vel.direction[i] = NEGATIVE_DIRECTION;
                    }
                    //
                    if (motion_flag)
                    {
                        state.newJointVel = true;
                        actor.pInterfaceVelocity.publish( actor.interface_joint_vel );
                    }
                }

                else if (state.vehicleMode == INTERFACE_STATE_HOME)
                {
                    if (input_command == SWING_CMD_HOMESTOP) // "SWINGHOMESTOP"
                    {
                        actor.interface_command.data = SWING_CMD_HOMESTOP;
                        actor.pInterfaceCommand.publish( actor.interface_command );
                        state.newCommand = true;
                    }
                }
            }
            double cost_2 = ros::Time::now().toSec() - start_2;
            ROS_DEBUG("2. SWING_COMMAND_HANDLLER and NAV_COMMAND_HANDLLER end. cost_time: %lf ms.", cost_2*1000);

            state.newImuState = false;
            if ( IMU_STATE_HANDLLER.newMsg() )
            {
                ROS_DEBUG("[part_3] get the imu states.");
                state.imuState = IMU_STATE_HANDLLER.fetchMsg();
                state.newImuState = true;
            }

            double start_4 = ros::Time::now().toSec();
            actor.current_time = ros::Time::now();
            if ( JOINT_STATE_HANDLLER.newMsg() )
            {
                ROS_DEBUG("[part_4] get the joint states.");
                state.jointState = JOINT_STATE_HANDLLER.fetchMsg();
                for (size_t i = 0; i < state.jointState.name.size(); ++i)
                {
                    if ( state.jointState.name[i] == JOINT_LEFT_WHEEL )
                        state.left_wheel_rotation = state.jointState.position[i];
                    else if ( state.jointState.name[i] == JOINT_RIGHT_WHEEL )
                        state.right_wheel_rotation = state.jointState.position[i];
                }

                double  delta_left_wheel = (state.left_wheel_rotation - actor.ref_left_rotation) * ROBOT_RADIUS;
                double  delta_right_wheel = (state.right_wheel_rotation - actor.ref_right_rotation) * ROBOT_RADIUS;
                double  delta_x = (delta_right_wheel + delta_left_wheel) / 2.0;
                double  delta_th = (delta_right_wheel - delta_left_wheel) / ROBOT_WIDTH;
                double  vx = delta_x / (actor.current_time-actor.last_time).toSec();
                double  vth = delta_th / (actor.current_time-actor.last_time).toSec();

                actor.ref_left_rotation = state.left_wheel_rotation;
                actor.ref_right_rotation = state.right_wheel_rotation;

                //
                geometry_msgs::Quaternion imu_quat;
                imu_quat = state.imuState.orientation;
                if (!state.newImuState)
                    ROS_ERROR("[2>>4] did not obtain the imu message in one loop");
                else
                    state.newImuState = false;
		//
		actor.ref_theta = quat2yaw(imu_quat);
                actor.ref_x = actor.ref_x + delta_x * cos(actor.ref_theta);
                actor.ref_y = actor.ref_y + delta_x * sin(actor.ref_theta);
                actor.ref_theta = actor.ref_theta + delta_th;
                geometry_msgs::TransformStamped odom_transform;
                odom_transform.header.stamp = actor.current_time;
                odom_transform.header.frame_id = VEHICLE_FIXED_FRAME;
                odom_transform.child_frame_id = VEHICLE_ODOMETRY_FRAME;

                odom_transform.transform.translation.x = actor.ref_x;
                odom_transform.transform.translation.y = actor.ref_y;
                odom_transform.transform.translation.z = 0.0;
                odom_transform.transform.rotation = imu_quat;
                //
                // update the navigation message - odom_navigation
                nav_msgs::Odometry  odom_navigation;
                odom_navigation.header.stamp = actor.current_time;
                odom_navigation.header.frame_id = VEHICLE_FIXED_FRAME;
                odom_navigation.child_frame_id = VEHICLE_ODOMETRY_FRAME;

                odom_navigation.pose.pose.position.x  = actor.ref_x;
                odom_navigation.pose.pose.position.y  = actor.ref_y;
                odom_navigation.pose.pose.position.z  = 0.0;
                odom_navigation.pose.pose.orientation = imu_quat;

                odom_navigation.twist.twist.linear.x  = vx;
                odom_navigation.twist.twist.linear.y  = 0.0;
                odom_navigation.twist.twist.linear.z  = 0.0;
                odom_navigation.twist.twist.angular.x = 0.0;
                odom_navigation.twist.twist.angular.y = 0.0;
                odom_navigation.twist.twist.angular.y = vth;
                //
                // publish the odometry and tf, then refresh the last_time
                broadcaster.sendTransform( odom_transform );
                actor.pVehicleOdometry.publish( odom_navigation );
                actor.last_time = actor.current_time;
            }
            double cost_4 = ros::Time::now().toSec() - start_4;
            ROS_DEBUG("4. JOINT_STATE_HANDLLER end. cost_time: %lf ms.", cost_4*1000);

            double start_5 = ros::Time::now().toSec();
            ROS_DEBUG("[part_5] publish coordinator states.");

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
                StringAppend( stateout, "%d ", actor.interface_joint_vel.command_id[0] );
                StringAppend( stateout, "%d ", actor.interface_joint_vel.command_id[1] );
                StringAppend( stateout, "%d ", actor.interface_joint_vel.command_id[2] );
                StringAppend( stateout, "%d ", actor.interface_joint_vel.command_id[3] );
            }
            else
            {
                StringAppend( stateout, "%s ", state.vehicleMode.c_str() );
                StringAppend( stateout, "%s ", state.swingState.c_str() );
                StringAppend( stateout, "%s ", state.baseState.c_str() );
            }
            ROS_DEBUG( "%s", stateout.c_str() );
            stateoutput.data = stateout;
            actor.pVehicleStates.publish( stateoutput );

            double cost_5 = ros::Time::now().toSec() - start_5;
            ROS_DEBUG("5.  vehicle state_output end. cost_time: %lf ms.", cost_5*1000);

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


