#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "mobot6_interface/command_list.h"
#include "mobot6_interface/ArmControl.h"
#include "mobot6_interface/SwingControl.h"
#include "mobot6_interface/NuclearControl.h"  // new_added
#include "mobot6_interface/GripControl.h"
#include <log4cxx/logger.h>


#include <sstream>
#include <iostream>
#include <stdexcept>
#include <string>

#define		INTERFACE_COMMAND		"/command_input"
#define		INTERFACE_JOINT_STATES		"/joint_states"
#define		MANIPULATOR_USER_FRAME		"/arm_user_frame"	// just for debuging
#define		MANIPULATOR_COMMAND		"/arm_command"
#define		NAVIGATION_COMMAND		"/navigation_command"   // "/cmd_vel"
#define		SWINGARM_COMMAND		"/swingarm_command"
#define   	NUCLEAR_COMMAND               "/nuclear_command"      // new_added
#define      	GRIPPER_COMMAND                 "/gripper_command"
#define		DISPLAY_TRAJECTORY		"/move_group/display_planned_path"

#define		BASE_LINK_NAME			"base_link"
#define		END_EFFECTOR_NAME		"fake_arm_link"

#define		JOINT_ARM_NUMBER                7
#define		JOINT1_ARM_NAME                 "joint_turret_rotate"
#define		JOINT2_ARM_NAME                 "joint_bigarm_pitch"
#define		JOINT3_ARM_NAME                 "joint_forearm_pitch"
#define		JOINT4_ARM_NAME                 "joint_forearm_rotate"
#define		JOINT5_ARM_NAME                 "joint_wrist_pitch"
#define		JOINT6_ARM_NAME                 "joint_wrist_rotate"
#define		JOINT7_ARM_NAME                 "fake_arm_joint"

// should modify the following link name
#define		LINK_ARM_NUMBER			8
#define		LINK_NAME_0                     "11_arm_base_link"
#define		LINK_NAME_1                     "12_arm_turret_rotate"
#define		LINK_NAME_2                     "13_arm_bigarm_pitch"
#define		LINK_NAME_3                     "15_arm_forearm_pitch"
#define		LINK_NAME_4                     "16_arm_forearm_rotate"
#define		LINK_NAME_5                     "17_arm_wrist_pitch"
#define		LINK_NAME_6                     "18_arm_wrist_rotate"
#define		LINK_NAME_7                     "fake_arm_link"

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
#define         COMMAND_ID_12                   "SPLINE"
#define         COMMAND_ID_13                   "MOVJ_LSPB"
#define         COMMAND_ID_14                   "MCHI_LSPB"
#define         COMMAND_ID_15                   "LASER"

//-----------------------------------------------------------------------new_added

#define   	COMMAND_ID_16                   "NUCLEAR"

#define         STP_CMD_MOVE                    "STEPMOVE"
#define         STP_CMD_HOME                    "STEPHOME"
#define         STP_CMD_STOP                    "STEPSTOP"
#define         STP_CMD_RESET                   "STEPRESET"
#define         STP_CMD_AUTO                    "STEPAUTO"
#define         STP_CMD_AUTOSTOP                "STEPAUTOSTOP"
#define         STP_CMD_DETECT                  "STEPDETECT"
#define         STP_CMD_SAVEFILE                "STEPSAVE"
#define         STP_CMD_REQ                     "STEPREQ"       // inertial command
#define         STP_CMD_OTHER                   "STEPOTHER"     // inertial command
//--------------------------------------------------------------------------------

#define         SWING_CMD_MOVE                 "SWINGMOVE"
#define         SWING_CMD_HOME                 "SWINGHOME"
#define         SWING_CMD_HOMESTOP             "SWINGHOMESTOP"
#define         SWING_CMD_RESET                "SWINGRESET"

#define		JOINT_DOF			6
#define		CARTESIAN_DOF			6
#define		SWING_DOF			2
#define         NUCLEAR_DOF                     2   // new_added

#define         LOOP_TIME                       0.050

class KeyboardInputHandller
{
public:
    void callback(const mobot6_interface::command_list::ConstPtr & message)
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

    const mobot6_interface::command_list& fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    mobot6_interface::command_list mCrtMsg;
    bool mNewMsg;
};


struct InputState
{
    int                             command_id;
    std::string                     commandID;
    std::string                     userFrame;
    mobot6_interface::SwingControl  swingControl;
    mobot6_interface::ArmControl    jointControl;
    mobot6_interface::ArmControl    cartesianControl;
    mobot6_interface::GripControl   gripperControl;
    mobot6_interface::ArmControl    lspbControl;
    mobot6_interface::NuclearControl    nuclearControl; // new_added
    
    geometry_msgs::Twist        navControl;
    ros::Subscriber             subKeyboardInput;
    ros::Publisher              pubNavCmd;
    ros::Publisher              pubArmCmd;
    ros::Publisher              pubSwingCmd;
    ros::Publisher              pubUserFrame;
    ros::Publisher              pubGripCmd;
    ros::Publisher              pubNuclearCmd;  // new_added
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Message_Pre_Process");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
                ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle             Node_Handle;
    KeyboardInputHandller       COMMAND_HANDLLER;

    // create the array of joint name and command id
    std::string LINK_NAME[] = { LINK_NAME_0, LINK_NAME_1, LINK_NAME_2, LINK_NAME_3, LINK_NAME_4,
                                LINK_NAME_5, LINK_NAME_6, LINK_NAME_7 };
    std::string JOINT_NAME[] = { JOINT1_ARM_NAME, JOINT2_ARM_NAME, JOINT3_ARM_NAME, JOINT4_ARM_NAME,
                                 JOINT5_ARM_NAME, JOINT6_ARM_NAME, JOINT7_ARM_NAME };
    std::string COMMAND_ID[] = { COMMAND_ID_0, COMMAND_ID_1, COMMAND_ID_2, COMMAND_ID_3, COMMAND_ID_4,
                                 COMMAND_ID_5, COMMAND_ID_6, COMMAND_ID_7, COMMAND_ID_8, COMMAND_ID_9,
                                 COMMAND_ID_10, COMMAND_ID_11, COMMAND_ID_12, COMMAND_ID_13,COMMAND_ID_14,
				 COMMAND_ID_15,COMMAND_ID_16};// new_added

    // create the input data and input state
    mobot6_interface::command_list  data_input;
    InputState                  state_input;
    std_msgs::String            msg;

    // initialize the state_input
    ROS_INFO( "waiting for joint state update" );

    state_input.command_id  = 8;
    state_input.commandID   = COMMAND_ID[8];
    state_input.userFrame   = LINK_NAME[0];
	
    // Initialize the state_input.navControl
    state_input.navControl.linear.x = 0.0;
    state_input.navControl.linear.y = 0.0;
    state_input.navControl.linear.z = 0.0;
    state_input.navControl.angular.x = 0.0;
    state_input.navControl.angular.y = 0.0;
    state_input.navControl.angular.z = 0.0;

    // Initialize the state_input.jointControl
    state_input.jointControl.data.resize(JOINT_DOF);
    for (size_t i=0; i<JOINT_DOF; ++i)
    {
        state_input.jointControl.data[i] = 0.0;
    }
    state_input.jointControl.commandID = state_input.commandID;
    state_input.jointControl.userFrame = state_input.userFrame;
	
    // Initialize the state_input.cartesianControl
    state_input.cartesianControl.data.resize(CARTESIAN_DOF);
    for (size_t i=0;i<CARTESIAN_DOF;++i)
    {
        state_input.cartesianControl.data[i] = 0.0;
    }
    state_input.cartesianControl.commandID = state_input.commandID;
    state_input.cartesianControl.userFrame = state_input.userFrame;
	
    // Initialize the state_input.swingControl
    state_input.swingControl.data.resize(SWING_DOF);
    for (size_t i=0; i<SWING_DOF; ++i)
    {
        state_input.swingControl.data[i] = 0.0;
    }
    state_input.swingControl.commandID = SWING_CMD_MOVE;

    // Initialize the state_input.nuclearContro------------------------------new_added
    state_input.nuclearControl.data.resize(NUCLEAR_DOF);
    for (size_t i=0; i<NUCLEAR_DOF; ++i)
    {
        state_input.nuclearControl.data[i] = 0.0;
    }
    state_input.nuclearControl.commandID = STP_CMD_OTHER;
    //---------   -----------------------------------------------------------------------
    // Initialize the state_input.gripperControl
    state_input.gripperControl.data = 0.0;
    state_input.gripperControl.commandID = state_input.commandID;

    // Initialize the state_input.lspbControl
    state_input.lspbControl.data.resize(20);
    for (size_t i=0;i<20;++i)
    {
        state_input.lspbControl.data[i] = 0.0;
    }
    state_input.lspbControl.commandID = state_input.commandID;
    state_input.lspbControl.userFrame = state_input.userFrame;

    // Initialize the subscriber and publisher of state_input
    state_input.subKeyboardInput =
            Node_Handle.subscribe<mobot6_interface::command_list>(
                INTERFACE_COMMAND, 3, &KeyboardInputHandller::callback,
                &COMMAND_HANDLLER );

    state_input.pubUserFrame =
            Node_Handle.advertise<std_msgs::String>(
                MANIPULATOR_USER_FRAME, 1, true);

    state_input.pubNavCmd =
            Node_Handle.advertise<geometry_msgs::Twist>(
                NAVIGATION_COMMAND, 2, false);

    state_input.pubSwingCmd =
            Node_Handle.advertise<mobot6_interface::SwingControl>(
                SWINGARM_COMMAND, 2, false);

    state_input.pubArmCmd =
            Node_Handle.advertise<mobot6_interface::ArmControl>(
                MANIPULATOR_COMMAND, 2, false);

    state_input.pubGripCmd=
            Node_Handle.advertise<mobot6_interface::GripControl>(
                GRIPPER_COMMAND, 2, false);
    state_input.pubNuclearCmd=
            Node_Handle.advertise<mobot6_interface::NuclearControl>(
                NUCLEAR_COMMAND, 2, false);
    // Inform the user
    ros::Duration(0.5).sleep();
    ROS_INFO( "main loop started" );

    // main loop,
    //ros::Rate loop_rate(5);
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
		state_input.lspbControl.userFrame = LINK_NAME[7];
				
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
                //-----------------------------------------------------------------------new_added
                for (int i = 0; i < NUCLEAR_DOF; i++)
                {
                    state_input.nuclearControl.data[i] = 0.0;
                }
                //--------------------------------------------------------------------------------
                state_input.gripperControl.data = data_input.grippers;
		state_input.lspbControl.data.resize(data_input.lspb.size(),0.0);
		for (int i = 0; i < data_input.lspb.size(); i++)
                {
                    state_input.lspbControl.data[i] = data_input.lspb[i];
                }		
                switch (state_input.command_id)
                {
                    case 0:
                    case 1:
                    case 2:
                    case 9:
                    case 10:
                    case 11:
	            case 12:
                        state_input.jointControl.commandID = COMMAND_ID[data_input.commandID];
                        state_input.cartesianControl.commandID = COMMAND_ID[8];
                        state_input.swingControl.commandID = SWING_CMD_MOVE;
			state_input.nuclearControl.commandID = STP_CMD_OTHER;   // new_added
                        state_input.gripperControl.commandID = COMMAND_ID[8];
                        break;
                    case 3:
                        state_input.cartesianControl.commandID = COMMAND_ID[data_input.commandID];
                        state_input.jointControl.commandID = COMMAND_ID[8];
                        state_input.swingControl.commandID = SWING_CMD_MOVE;
                        state_input.nuclearControl.commandID = STP_CMD_OTHER;   // new_added
                        state_input.gripperControl.commandID = COMMAND_ID[8];
                        break;
                    case 5:
                    {
                        if ((data_input.swings[0]==6.0)&&(data_input.swings[1]==6.0))
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
                        else if ((fabs(data_input.swings[0])!=0.0)||(fabs(data_input.swings[1])!=0.0))
                        {
                            state_input.swingControl.commandID = SWING_CMD_MOVE;
                            for (int i = 0; i < SWING_DOF; i++)
                                state_input.swingControl.data[i] = data_input.swings[i];
                        }
                        else
                        {
                            state_input.swingControl.commandID = SWING_CMD_MOVE;
                        }
                        state_input.jointControl.commandID = COMMAND_ID[8];
                        state_input.cartesianControl.commandID = COMMAND_ID[8];
                        state_input.nuclearControl.commandID = STP_CMD_OTHER;   // new_added
                        state_input.gripperControl.commandID = COMMAND_ID[8];
                        break;
                    }
//-----------------------------------------------------------------------new_added
                case 16:
                {
                    if (data_input.nuclear[0] == -1.0)
                    {
                        state_input.nuclearControl.commandID = STP_CMD_STOP;
                        for (int i = 0; i < NUCLEAR_DOF; i++)
                            state_input.nuclearControl.data[i] = 0.0;
                    }
                    else if (data_input.nuclear[0] == -2.0)
                    {
                        state_input.nuclearControl.commandID = STP_CMD_RESET;
                        for (int i = 0; i < NUCLEAR_DOF; i++)
                            state_input.nuclearControl.data[i] = 0.0;
                    }
                    else if (data_input.nuclear[0] == -3.0)
                    {
                        state_input.nuclearControl.commandID = STP_CMD_HOME;
                        for (int i = 0; i < NUCLEAR_DOF; i++)
                            state_input.nuclearControl.data[i] = 0.0;
                    }
                    else if (data_input.nuclear[0] == -4.0)
                    {
                        state_input.nuclearControl.commandID = STP_CMD_DETECT;
                        for (int i = 0; i < NUCLEAR_DOF; i++)
                            state_input.nuclearControl.data[i] = data_input.nuclear[i+1];
                    }
                    else if (data_input.nuclear[0] == -5.0)
                    {
                        state_input.nuclearControl.commandID = STP_CMD_AUTO;
                        for (int i = 0; i < NUCLEAR_DOF; i++)
                            state_input.nuclearControl.data[i] = data_input.nuclear[1];
                    }
                    else if (data_input.nuclear[0] == -6.0)
                    {
                        state_input.nuclearControl.commandID = STP_CMD_AUTOSTOP;
                        for (int i = 0; i < NUCLEAR_DOF; i++)
                            state_input.nuclearControl.data[i] = 0.0;
                    }
                    else if (data_input.nuclear[0] == -7.0)
                    {
                        state_input.nuclearControl.commandID = STP_CMD_SAVEFILE;
                        for (int i = 0; i < NUCLEAR_DOF; i++)
                            state_input.nuclearControl.data[i] = 0.0;
                    }
                    else if (data_input.nuclear[0] == 1.0)
                    {
                        state_input.nuclearControl.commandID = STP_CMD_MOVE;
                        for (int i = 0; i < NUCLEAR_DOF; i++)
                            state_input.nuclearControl.data[i] = data_input.nuclear[i+1];
                    }
                    else
                    {
                        state_input.nuclearControl.commandID = STP_CMD_OTHER;
                    }
                    state_input.jointControl.commandID = COMMAND_ID[8];
                    state_input.cartesianControl.commandID = COMMAND_ID[8];
                    state_input.swingControl.commandID = SWING_CMD_MOVE;
                    state_input.gripperControl.commandID = COMMAND_ID[8];
                    break;
                }
                //--------------------------------------------------------------------------------
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
                        state_input.nuclearControl.commandID = STP_CMD_OTHER;   // new_added
                        state_input.gripperControl.commandID = COMMAND_ID[8];
                        break;
		    case 13:
		    case 14:
			state_input.lspbControl.commandID = COMMAND_ID[data_input.commandID];
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
            // if the COMMAND_HANDLLER is null
            else
            {
                state_input.command_id = 8;
                state_input.commandID = COMMAND_ID[8];
                //state_input.userFrame = LINK_NAME[0];

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
		// ----------------------------------------------------------------------new_added
                state_input.nuclearControl.data.resize(NUCLEAR_DOF);
                for (size_t i=0; i<NUCLEAR_DOF; ++i)
                {
                    state_input.nuclearControl.data[i] = 0.0;
                }
                state_input.nuclearControl.commandID = STP_CMD_OTHER;
                //--------------------------------------------------------------------------------
                state_input.gripperControl.data = 0.0;
                state_input.gripperControl.commandID = state_input.commandID;
				
		state_input.lspbControl.data.resize(20);
                for (size_t i=0;i<20;++i)
                {
                    state_input.lspbControl.data[i] = 0.0;
                }
    		state_input.lspbControl.commandID = state_input.commandID;
   		state_input.lspbControl.userFrame = state_input.userFrame;

                state_input.navControl.linear.x = 0.0;
                state_input.navControl.linear.y = 0.0;
                state_input.navControl.linear.z = 0.0;
                state_input.navControl.angular.x = 0.0;
                state_input.navControl.angular.y = 0.0;
                state_input.navControl.angular.z = 0.0;
            }

/*          ROS_INFO("navControl: %f, %f, %f, %f, %f, %f",
                        state_input.navControl.linear.x, state_input.navControl.linear.y,
                        state_input.navControl.linear.z, state_input.navControl.angular.x,
                        state_input.navControl.angular.y, state_input.navControl.angular.z);

*/          ROS_DEBUG("jointControl.commandID: %s, userFrame: %s.\n",
                        state_input.jointControl.commandID.c_str(),
                        state_input.jointControl.userFrame.c_str());
/*          for(size_t i=0;i<JOINT_DOF;++i)
            {
                ROS_INFO("jointControl.data[%ld]: %f", i+1, state_input.jointControl.data[i]);
            }
*/
            ROS_DEBUG("cartesianControl.commandID: %s, userFrame: %s.\n",
                        state_input.cartesianControl.commandID.c_str(),
                        state_input.cartesianControl.userFrame.c_str());
/*          for(size_t i=0;i<CARTESIAN_DOF;++i)
            {
                ROS_INFO("cartesianControl.data[%ld]: %f", i+1, state_input.cartesianControl.data[i]);
            }
*/
            ROS_DEBUG("swingControl.commandID: %s.\n", state_input.swingControl.commandID.c_str());
/*          for(size_t i=0;i<SWING_DOF;++i)
            {
                ROS_INFO("swingControl.data[%ld]: %f", i+1, state_input.swingControl.data[i]);
            }
*/
            ROS_DEBUG("gripperControl.commandID: %s.\n\n\n", state_input.gripperControl.commandID.c_str());
            ROS_DEBUG("lspbControl.commandID: %s.\n\n\n", state_input.lspbControl.commandID.c_str());

            state_input.pubNavCmd.publish(state_input.navControl);
            state_input.pubSwingCmd.publish(state_input.swingControl);
            state_input.pubNuclearCmd.publish(state_input.nuclearControl);  // new_added
            state_input.pubGripCmd.publish(state_input.gripperControl);
            msg.data = state_input.userFrame;
            state_input.pubUserFrame.publish(msg);
            if (state_input.command_id==0 || state_input.command_id==1 || state_input.command_id==2 ||
                    state_input.command_id==9 || state_input.command_id==10 ||
		    state_input.command_id==11 || state_input.command_id==12)
                state_input.pubArmCmd.publish(state_input.jointControl);
            else if (state_input.command_id == 3)
                state_input.pubArmCmd.publish(state_input.cartesianControl);
            else if (state_input.command_id == 13 || state_input.command_id == 14)
                state_input.pubArmCmd.publish(state_input.lspbControl);
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
