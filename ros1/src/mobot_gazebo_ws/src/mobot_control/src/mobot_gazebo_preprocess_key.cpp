#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "mobot_control/Keyboard.h"
#include "mobot_control/ArmControl.h"
#include "mobot_control/SwingControl.h"
#include "mobot_control/GripControl.h"
#include <log4cxx/logger.h>

#include <sstream>
#include <iostream>
#include <stdexcept>
#include <string>

#define		INTERFACE_COMMAND		"/keyboard_input"
#define		INTERFACE_JOINT_STATES		"/joint_states"
#define		MANIPULATOR_USER_FRAME		"/arm_user_frame"
#define		MANIPULATOR_COMMAND		"/arm_command"
#define		NAVIGATION_COMMAND		"/navigation_command"   
#define		SWINGARM_COMMAND		"/swingarm_command"
#define      	GRIPPER_COMMAND                 "/gripper_command"
#define		DISPLAY_TRAJECTORY		"/move_group/display_planned_path"

#define		BASE_LINK_NAME			"base_link"
#define		END_EFFECTOR_NAME		"fake_arm_link"

#define		JOINT_ARM_NUMBER                8
#define		JOINT1_ARM_NAME                 "joint_turret_rotate"
#define		JOINT2_ARM_NAME                 "joint_bigarm_pitch"
#define		JOINT3_ARM_NAME                 "joint_bigarm_linear"
#define		JOINT4_ARM_NAME                 "joint_forearm_pitch"
#define		JOINT5_ARM_NAME                 "joint_forearm_rotate"
#define		JOINT6_ARM_NAME                 "joint_wrist_pitch"
#define		JOINT7_ARM_NAME                 "joint_wrist_rotate"
#define		JOINT8_ARM_NAME                 "fake_arm_joint"
//
#define		LINK_ARM_NUMBER			9
#define		LINK_NAME_0                     "11_arm_base_link"
#define		LINK_NAME_1                     "12_arm_turret_rotate"
#define		LINK_NAME_2                     "13_arm_bigarm_pitch"
#define         LINK_NAME_3                     "14_arm_bigarm_linear"
#define		LINK_NAME_4                     "15_arm_forearm_pitch"
#define		LINK_NAME_5                     "16_arm_forearm_rotate"
#define		LINK_NAME_6                     "17_arm_wrist_pitch"
#define		LINK_NAME_7                     "18_arm_wrist_rotate"
#define		LINK_NAME_8                     "fake_arm_link"

#define		JOINT_SWINGARM_LEFT		"joint_swingarm_left"
#define		JOINT_SWINGARM_RIGHT		"joint_swingarm_right"
#define		JOINT_WHEEL_LEFT		"base_l_wheel_joint"
#define		JOINT_WHEEL_RIGHT		"base_r_wheel_joint"

#define		COMMAND_ID_0			"STOP"
#define		COMMAND_ID_1			"MOVE"
#define		COMMAND_ID_2			"ARMJ"
#define		COMMAND_ID_3			"ARMC"
#define		COMMAND_ID_4			"BASE"
#define		COMMAND_ID_5			"SWING"
#define		COMMAND_ID_6			"GRIP"
#define		COMMAND_ID_7			"COORD"
#define		COMMAND_ID_8			"OTHERS"
#define		COMMAND_ID_9			"HOME"
#define		COMMAND_ID_10			"HOMESTOP"
#define         COMMAND_ID_11                   "NOLIMIT"

#define         SWING_CMD_MOVE                 "SWINGMOVE"
#define         SWING_CMD_HOME                 "SWINGHOME"
#define         SWING_CMD_HOMESTOP             "SWINGHOMESTOP"
#define         SWING_CMD_RESET                "SWINGRESET"

#define		JOINT_DOF			7
#define		CARTESIAN_DOF			6
#define		SWING_DOF			2

#define         LOOP_TIME                       0.050

class KeyboardInputHandller
{
public:
    void callback(const mobot_control::Keyboard::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    KeyboardInputHandller()
        : mCrtMsg()
        , mNewMsg ( false )
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const mobot_control::Keyboard& fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    mobot_control::Keyboard mCrtMsg;
    bool mNewMsg;
};


struct InputState
{
    int                             command_id;
    std::string                     commandID;
    std::string                     userFrame;
    mobot_control::SwingControl   swingControl;
    mobot_control::ArmControl     jointControl;
    mobot_control::ArmControl     cartesianControl;
    mobot_control::GripControl    gripperControl;
    
    geometry_msgs::Twist        navControl;
    ros::Subscriber             subKeyboardInput;
    ros::Publisher              pubNavCmd;
    ros::Publisher              pubArmCmd;
    ros::Publisher              pubSwingCmd;
    ros::Publisher              pubUserFrame;
    ros::Publisher              pubGripCmd;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Message_Pre_Process");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
                ros::console::g_level_lookup[ros::console::levels::Debug]);
    //
    ros::NodeHandle             Node_Handle;
    KeyboardInputHandller       COMMAND_HANDLLER;

    std::string LINK_NAME[] = { LINK_NAME_0, LINK_NAME_1, LINK_NAME_2, LINK_NAME_3, LINK_NAME_4,
                                LINK_NAME_5, LINK_NAME_6, LINK_NAME_7, LINK_NAME_8 };
    std::string JOINT_NAME[] = { JOINT1_ARM_NAME, JOINT2_ARM_NAME, JOINT3_ARM_NAME, JOINT4_ARM_NAME,
                                 JOINT5_ARM_NAME, JOINT6_ARM_NAME, JOINT7_ARM_NAME, JOINT8_ARM_NAME };
    std::string COMMAND_ID[] = { COMMAND_ID_0, COMMAND_ID_1, COMMAND_ID_2, COMMAND_ID_3, COMMAND_ID_4,
                                 COMMAND_ID_5, COMMAND_ID_6, COMMAND_ID_7, COMMAND_ID_8, COMMAND_ID_9,
                                 COMMAND_ID_10, COMMAND_ID_11 };
    //
    mobot_control::Keyboard     data_input;
    InputState                  state_input;
    std_msgs::String            msg;
    //
    ROS_INFO( "waiting for joint state update" );

    state_input.command_id  = 8;
    state_input.commandID   = COMMAND_ID[8];
    state_input.userFrame   = LINK_NAME[8];
    // 
    state_input.navControl.linear.x = 0.0;
    state_input.navControl.linear.y = 0.0;
    state_input.navControl.linear.z = 0.0;
    state_input.navControl.angular.x = 0.0;
    state_input.navControl.angular.y = 0.0;
    state_input.navControl.angular.z = 0.0;
    //
    state_input.jointControl.data.resize(JOINT_DOF);
    for (size_t i=0; i<JOINT_DOF; ++i)
    {
        state_input.jointControl.data[i] = 0.0;
    }
    state_input.jointControl.commandID = state_input.commandID;
    state_input.jointControl.userFrame = state_input.userFrame;
    // 
    state_input.cartesianControl.data.resize(CARTESIAN_DOF);
    for (size_t i=0;i<CARTESIAN_DOF;++i)
    {
        state_input.cartesianControl.data[i] = 0.0;
    }
    state_input.cartesianControl.commandID = state_input.commandID;
    state_input.cartesianControl.userFrame = state_input.userFrame;
    //
    state_input.swingControl.data.resize(SWING_DOF);
    for (size_t i=0; i<SWING_DOF; ++i)
    {
        state_input.swingControl.data[i] = 0.0;
    }
    state_input.swingControl.commandID = SWING_CMD_MOVE;
    // 
    state_input.gripperControl.data = 0.0;
    state_input.gripperControl.commandID = state_input.commandID;
    // 
    state_input.subKeyboardInput =
            Node_Handle.subscribe<mobot_control::Keyboard>(
                INTERFACE_COMMAND, 3, &KeyboardInputHandller::callback,
                &COMMAND_HANDLLER );

    state_input.pubUserFrame =
            Node_Handle.advertise<std_msgs::String>(
                MANIPULATOR_USER_FRAME, 1, true);

    state_input.pubNavCmd =
            Node_Handle.advertise<geometry_msgs::Twist>(
                NAVIGATION_COMMAND, 2, false);

    state_input.pubSwingCmd =
            Node_Handle.advertise<mobot_control::SwingControl>(
                SWINGARM_COMMAND, 2, false);

    state_input.pubArmCmd =
            Node_Handle.advertise<mobot_control::ArmControl>(
                MANIPULATOR_COMMAND, 2, false);

    state_input.pubGripCmd=
            Node_Handle.advertise<mobot_control::GripControl>(
                GRIPPER_COMMAND, 2, false);
    // 
    ros::Duration(0.5).sleep();
    ROS_INFO( "main loop started" );

    // main loop
    while (ros::ok())
    {
        try
        {
            double start = ros::Time::now().toSec();

            ros::spinOnce();
            if (COMMAND_HANDLLER.newMsg())
            {
                data_input = COMMAND_HANDLLER.fetchMsg();
				
                state_input.command_id = data_input.commandID;
                state_input.commandID = COMMAND_ID[data_input.commandID];
                state_input.userFrame = LINK_NAME[data_input.framesID];
				
                state_input.jointControl.userFrame = state_input.userFrame;
                state_input.cartesianControl.userFrame = state_input.userFrame;

				
                for (int i = 0; i < JOINT_DOF; i++)
                {
                    state_input.jointControl.data[i] = data_input.joints[i];
                }
                for (int i = 0; i < CARTESIAN_DOF; i++)
                {
                    state_input.cartesianControl.data[i] = data_input.cartesians[i];
                }
                for (int i = 0; i < SWING_DOF; i++)
                {
                    state_input.swingControl.data[i] = 0.0;
                }
                state_input.gripperControl.data = data_input.grippers;
				
                switch (state_input.command_id)
                {
                    case 0:
                    case 1:
                    case 2:
                    case 9:
                    case 10:
                    case 11:
                        state_input.jointControl.commandID = COMMAND_ID[data_input.commandID];
                        state_input.cartesianControl.commandID = COMMAND_ID[8];
                        state_input.swingControl.commandID = SWING_CMD_MOVE;
                        state_input.gripperControl.commandID = COMMAND_ID[8];
                        break;
                    case 3:
                        state_input.cartesianControl.commandID = COMMAND_ID[data_input.commandID];
                        state_input.jointControl.commandID = COMMAND_ID[8];
                        state_input.swingControl.commandID = SWING_CMD_MOVE;
                        state_input.gripperControl.commandID = COMMAND_ID[8];
                        break;
                    case 5:
                    {
                        if ((fabs(data_input.swings[0])==2.0)||(fabs(data_input.swings[1])==2.0))
                        {
                            state_input.swingControl.commandID = SWING_CMD_MOVE;
                            for (int i = 0; i < SWING_DOF; i++)
                                state_input.swingControl.data[i] = data_input.swings[i];
                        }
                        else if ((data_input.swings[0]==6.0)&&(data_input.swings[1]==6.0))
                        {
                            state_input.swingControl.commandID = SWING_CMD_HOME;
                            for (int i = 0; i < SWING_DOF; i++)
                                state_input.swingControl.data[i] = 0.0;
                        }
                        else if ((data_input.swings[0]==7.0)&&(data_input.swings[1]==7.0))
                        {
                            state_input.swingControl.commandID = SWING_CMD_HOMESTOP;
                            for (int i = 0; i < SWING_DOF; i++)
                                state_input.swingControl.data[i] = 0.0;
                        }
                        else if ((data_input.swings[0]==8.0)&&(data_input.swings[1]==8.0))
                        {
                            state_input.swingControl.commandID = SWING_CMD_RESET;
                            for (int i = 0; i < SWING_DOF; i++)
                                state_input.swingControl.data[i] = 0.0;
                        }
                        else
                        {
                            state_input.swingControl.commandID = SWING_CMD_MOVE;
                        }
                        state_input.jointControl.commandID = COMMAND_ID[8];
                        state_input.cartesianControl.commandID = COMMAND_ID[8];
                        state_input.gripperControl.commandID = COMMAND_ID[8];
                        break;
                    }
                    case 6:
                    {
                        state_input.gripperControl.commandID = COMMAND_ID[data_input.commandID];
                        state_input.jointControl.commandID = COMMAND_ID[8];
                        state_input.cartesianControl.commandID = COMMAND_ID[8];
                        state_input.swingControl.commandID = SWING_CMD_MOVE;
                        break;
                    }
                    case 4:
                    case 7:
                    case 8:
                        state_input.jointControl.commandID = COMMAND_ID[8];
                        state_input.cartesianControl.commandID = COMMAND_ID[8];
                        state_input.swingControl.commandID = SWING_CMD_MOVE;
                        state_input.gripperControl.commandID = COMMAND_ID[8];
                        break;
                    default:
                        ROS_ERROR("Something is wrong in command_id operation.");
                }

                state_input.navControl.linear.x = data_input.moves[0];
                state_input.navControl.linear.y = data_input.moves[1];
                state_input.navControl.linear.z = data_input.moves[2];
                state_input.navControl.angular.x = 0.0;
                state_input.navControl.angular.y = 0.0;
                state_input.navControl.angular.z = data_input.moves[3];
            }
            else
            {
                state_input.command_id = 8;
                state_input.commandID = COMMAND_ID[8];

                state_input.jointControl.data.resize(JOINT_DOF);
                for (size_t i=0;i<JOINT_DOF;++i)
                {
                    state_input.jointControl.data[i] = 0.0;
                }
                state_input.jointControl.commandID = state_input.commandID;
                state_input.jointControl.userFrame = state_input.userFrame;

                state_input.cartesianControl.data.resize(CARTESIAN_DOF);
                for (size_t i=0;i<CARTESIAN_DOF;++i)
                {
                    state_input.cartesianControl.data[i] = 0.0;
                }
                state_input.cartesianControl.commandID = state_input.commandID;
                state_input.cartesianControl.userFrame = state_input.userFrame;

                state_input.swingControl.data.resize(SWING_DOF);
                for (size_t i=0;i<SWING_DOF;++i)
                {
                    state_input.swingControl.data[i] = 0.0;
                }
                state_input.swingControl.commandID = SWING_CMD_MOVE;

                state_input.gripperControl.data = 0.0;
                state_input.gripperControl.commandID = state_input.commandID;
				
                state_input.navControl.linear.x = 0.0;
                state_input.navControl.linear.y = 0.0;
                state_input.navControl.linear.z = 0.0;
                state_input.navControl.angular.x = 0.0;
                state_input.navControl.angular.y = 0.0;
                state_input.navControl.angular.z = 0.0;
            }

	    ROS_DEBUG("jointControl.commandID: %s, userFrame: %s.\n",
                        state_input.jointControl.commandID.c_str(),
                        state_input.jointControl.userFrame.c_str());
            ROS_DEBUG("cartesianControl.commandID: %s, userFrame: %s.\n",
                        state_input.cartesianControl.commandID.c_str(),
                        state_input.cartesianControl.userFrame.c_str());
            ROS_DEBUG("swingControl.commandID: %s.\n", state_input.swingControl.commandID.c_str());
            ROS_DEBUG("gripperControl.commandID: %s.\n\n\n", state_input.gripperControl.commandID.c_str());

            state_input.pubNavCmd.publish(state_input.navControl);
            state_input.pubSwingCmd.publish(state_input.swingControl);
            state_input.pubGripCmd.publish(state_input.gripperControl);
            msg.data = state_input.userFrame;
            state_input.pubUserFrame.publish(msg);
            if (state_input.command_id==0 || state_input.command_id==1 || state_input.command_id==2 ||
                    state_input.command_id==9 || state_input.command_id==10 || state_input.command_id==11)
                state_input.pubArmCmd.publish(state_input.jointControl);
            else if (state_input.command_id == 3)
                state_input.pubArmCmd.publish(state_input.cartesianControl);

            // time control
            double sleep_time = LOOP_TIME - ( ros::Time::now().toSec() - start );
            if ( sleep_time > 0 )
            {
                ros::Duration ( sleep_time ).sleep();
                ROS_INFO("Cost Time: %lf ms", (LOOP_TIME-sleep_time)*1000);
            }
            else
                ROS_WARN ( "control loop over time: %f ms", -sleep_time*1000 );
        }
        catch (...)
        {
            ROS_ERROR ( "Maybe something is wrong!" );
        }
    }

    ros::waitForShutdown();
    return 0;
}
