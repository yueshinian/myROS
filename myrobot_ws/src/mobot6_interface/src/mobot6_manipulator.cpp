#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include "mobot6_interface/ArmControl.h"
#include "mobot6_interface/GripControl.h"
#include "monotonecubicinterpolation.h"
#include <log4cxx/logger.h>


#include <vector>
#include <list>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

typedef     const char*     CStrPtr;


#define     MANIPULATOR_USER_FRAME          "/arm_user_frame"      
#define     MANIPULATOR_COMMAND             "/arm_command"        
#define     MANIPULATOR_STATES              "/arm_states"        
#define     GRIPPER_COMMAND                 "/gripper_command" 

#define     INTERFACE_JOINT_STATES          "/joint_states_arm"     
#define     INTERFACE_STATES                "/interface_states"      
#define     INTERFACE_JOINT_PATH_COMMAND    "/joint_path_command"   
#define     INTERFACE_COMMAND               "/interface_command"    
#define     INTERFACE_COMMAND_GRIP          "/interface_command_grip"  

#define     INTERFACE_STATE_SPARATER    "\t"
#define     INTERFACE_STATE_MOVE        "MOVE"
#define     INTERFACE_STATE_SPLINE      "SPLINE"
#define     INTERFACE_STATE_INIT        "INIT"
#define     INTERFACE_STATE_STOP        "STOP"
#define     INTERFACE_STATE_HOME        "HOME"

#define     INTERFACE_STATE_MOVING      "MOVING"
#define     INTERFACE_STATE_STABLE      "STABLE"

#define     INTERFACE_STATE_DELAYED     "DELAYED"
#define     INTERFACE_STATE_NORMAL      "NORMAL"

#define     INTERFACE_CMD_MOVE          "MOVE"
#define     INTERFACE_CMD_SPLINE        "SPLINE"
#define     INTERFACE_CMD_CLEAR         "CLEAR" // dummy parameter??
#define     INTERFACE_CMD_HALT          "HALT"  // dummy parameter??
#define     INTERFACE_CMD_STOP          "STOP"
#define     INTERFACE_CMD_HOME          "HOME"
#define     INTERFACE_CMD_HOMESTOP      "HOMESTOP"

#define     GROUP_MANIPULATOR           "manipulator"

#define     MANIPULATOR_BASE_LINK       "11_arm_base_link"
#define     MANIPULATOR_END_EFFECTOR    "fake_arm_link"

#define		JOINT_ARM_NUMBER        7
#define		JOINT1_ARM_NAME         "joint_turret_rotate"
#define		JOINT2_ARM_NAME         "joint_bigarm_pitch"
#define		JOINT3_ARM_NAME         "joint_forearm_pitch"
#define		JOINT4_ARM_NAME         "joint_forearm_rotate"
#define		JOINT5_ARM_NAME         "joint_wrist_pitch"
#define		JOINT6_ARM_NAME         "joint_wrist_rotate"
#define		JOINT7_ARM_NAME         "fake_arm_joint"

#define		LINK_ARM_NUMBER         8
#define		LINK_NAME_0             "11_arm_base_link"
#define		LINK_NAME_1             "12_arm_turret_rotate"
#define		LINK_NAME_2             "13_arm_bigarm_pitch"
#define		LINK_NAME_3             "15_arm_forearm_pitch"
#define		LINK_NAME_4             "16_arm_forearm_rotate"
#define		LINK_NAME_5             "17_arm_wrist_pitch"
#define		LINK_NAME_6             "18_arm_wrist_rotate"
#define		LINK_NAME_7             "fake_arm_link"

#define		COMMAND_ID_0            "STOP"
#define		COMMAND_ID_1            "MOVE"
#define		COMMAND_ID_2            "ARMJ"
#define		COMMAND_ID_3            "ARMC"
#define		COMMAND_ID_4            "BASE"
#define		COMMAND_ID_5            "SWING"
#define		COMMAND_ID_6            "GRIP"
#define		COMMAND_ID_7            "COORD"
#define		COMMAND_ID_8            "OTHERS"
#define		COMMAND_ID_9            "HOME"
#define		COMMAND_ID_10           "HOMESTOP"
#define         COMMAND_ID_11           "NOLIMIT"

#define         GRIP_FORWARD            "GripForward"
#define         GRIP_REVERSE            "GripReverse"
#define         GRIP_STOP               "GripStop"

#define         HOME_AXIS_ALL           "AXISALL"
#define         HOME_AXIS_1             "AXIS1"
#define         HOME_AXIS_2             "AXIS2"
#define         HOME_AXIS_3             "AXIS3"
#define         HOME_AXIS_4             "AXIS4"
#define         HOME_AXIS_5             "AXIS5"
#define         HOME_AXIS_6             "AXIS6"
#define         HOME_AXIS_7             "AXIS7"

#define		JOINT_DOF               6  
#define		CARTESIAN_DOF           6
#define		LINK_NUM                8

#define		LOOP_TIME               0.050
#define		GMAS_TIME               0.060 // 0.050

#define		LIMIT_REVOL             45.0

#define         _in                     const &
#define         _inc                    const
#define         _out                          &
#define         _ret                    const
#define         _wt                           &
#define         _rd                     const &

std::string     LINK_NAME[] = { LINK_NAME_0, LINK_NAME_1, LINK_NAME_2, LINK_NAME_3, LINK_NAME_4,
                                LINK_NAME_5, LINK_NAME_6, LINK_NAME_7 };
std::string     JOINT_NAME[] = { JOINT1_ARM_NAME, JOINT2_ARM_NAME, JOINT3_ARM_NAME, JOINT4_ARM_NAME,
                                 JOINT5_ARM_NAME, JOINT6_ARM_NAME, JOINT7_ARM_NAME };
std::string     COMMAND_ID[] = { COMMAND_ID_0, COMMAND_ID_1, COMMAND_ID_2, COMMAND_ID_3, COMMAND_ID_4,
                                 COMMAND_ID_5, COMMAND_ID_6, COMMAND_ID_7, COMMAND_ID_8, COMMAND_ID_9,
                                 COMMAND_ID_10, COMMAND_ID_11 };
std::string     HOME_AXIS[] = { HOME_AXIS_ALL, HOME_AXIS_1, HOME_AXIS_2, HOME_AXIS_3, HOME_AXIS_4,
                                HOME_AXIS_5, HOME_AXIS_6, HOME_AXIS_7 };

#define         LIMIT_FORWARD_AXIS1     5    //2.0943
#define         LIMIT_REVERSE_AXIS1     -3.3   //-2.0943

#define         LIMIT_FORWARD_AXIS2     5      //0.7854
#define         LIMIT_REVERSE_AXIS2     -3.14       //-1.5708

#define         LIMIT_FORWARD_AXIS3     1.0472
#define         LIMIT_REVERSE_AXIS3     -3.4907

#define         LIMIT_FORWARD_AXIS4     3.1415
#define         LIMIT_REVERSE_AXIS4     -3.1415

#define         LIMIT_FORWARD_AXIS5     1.5708 // 1.3963(80.0deg)
#define         LIMIT_REVERSE_AXIS5     -2.0 // -1.5708 //-1.7453

#define         LIMIT_FORWARD_AXIS6     3.1415
#define         LIMIT_REVERSE_AXIS6     -3.1415

#define         LIMIT_FORWARD_GRIP      1.00
#define         LIMIT_REVERSE_GRIP      -2.00

double  LIMIT_MANIPULATOR[] = { LIMIT_FORWARD_AXIS1, LIMIT_REVERSE_AXIS1,
                                LIMIT_FORWARD_AXIS2, LIMIT_REVERSE_AXIS2,
                                LIMIT_FORWARD_AXIS3, LIMIT_REVERSE_AXIS3,
                                LIMIT_FORWARD_AXIS4, LIMIT_REVERSE_AXIS4,
                                LIMIT_FORWARD_AXIS5, LIMIT_REVERSE_AXIS5,
                                LIMIT_FORWARD_AXIS6, LIMIT_REVERSE_AXIS6 };
double  LIMIT_GRIPPER[]     = { LIMIT_FORWARD_GRIP, LIMIT_REVERSE_GRIP };


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


//--------------------------------------------------------------------------------
class MoveitGroup
{
public:
    MoveitGroup(const std::string&  group_name)
        : group(group_name)
    {
    }
public:
    moveit::planning_interface::MoveGroup       group;
    robot_state::JointModelGroup*               joint_group;
    std::vector<std::string>                    joint_names;
    std::vector<double>                         max_vel;
    Eigen::VectorXd                             max_joint_disp;
    std::string                                 eef;
    std::string                                 base;
};

class Parameter
{
public:
    Parameter()
        : manipulator(GROUP_MANIPULATOR)
    {
    }
public:
    double                          loop_time;
    double                          gmas_time;
    robot_model::RobotModelPtr      robot_model;
    tf::Transform                   userFrameTF;
    tf::StampedTransform            T2B_transform;
    tf::StampedTransform            T2U_transform;
    MoveitGroup                     manipulator;
};

struct State
{
    int                             command_id;
    std::string                     commandID;
    std::string                     userFrame;
    std::string                     interfaceState;
    mobot6_interface::ArmControl    armControl;
    mobot6_interface::GripControl   gripControl;

    sensor_msgs::JointState         jointState;  // current (real) joint state
    robot_state::RobotStatePtr      robotState;  // current (real) state
    ros::Subscriber                 subUserFrame;       // "/arm_user_frame"
    ros::Subscriber                 subArmCmd;          // "/arm_command"
    ros::Subscriber                 subGripCmd;         // "/gripper_command"
    ros::Subscriber                 subJointStates;     // "/joint_states"
    ros::Subscriber                 subInterfaceStates; // "//interface_states"

    bool                            newFrame;
    bool                            newArmCommand;
    std::string                     ExecuteResult;
    bool                            no_joint_limitation; // wether delete soft position limitation
};

struct Actor
{
    sensor_msgs::JointState     ref_joint;
    robot_state::RobotStatePtr  ref_state;

    ros::Publisher              pInterfaceJointPath;    // "/joint_path_command"
    ros::Publisher              pCoordinateStates;      // "/coordinator_states"
    ros::Publisher              pInterfaceCommand;      // "/interface_command"
    ros::Publisher              pInterfaceCommandGrip;  // "/interface_command_grip"
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

class ArmCommandHandller
{
public:
    void callback(const mobot6_interface::ArmControl::ConstPtr & message)
    {
        mCrtMsg.push_back(*message);
    }

    ArmCommandHandller()
        : mCrtMsg()
    {
    }

    const bool newMsg()
    {
        // ROS_WARN("arm command - newMsg [%d]", mCrtMsg.size()>0);
        return mCrtMsg.size()>0;
    }

    const mobot6_interface::ArmControl fetchMsg()
    {
        mobot6_interface::ArmControl ret = mCrtMsg.front();
        mCrtMsg.pop_front();
        ROS_WARN("arm command - fetchMsg successfully");
        return ret;
    }

    void clear()
    {
        mCrtMsg.clear();
    }

    size_t size() const
    {
        return mCrtMsg.size();
    }

protected:
    std::list<mobot6_interface::ArmControl> mCrtMsg;
};

class GripCommandHandller
{
public:
    void callback(const mobot6_interface::GripControl::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    GripCommandHandller()
        : mCrtMsg()
        , mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const mobot6_interface::GripControl & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    mobot6_interface::GripControl mCrtMsg;
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

class CommandProcessor
{
public:
    CommandProcessor(std::string x) : regex(x) {  }
    //CommandProcessor(CommandProcessor & x) : regex(x.regex) {  }
    virtual ~CommandProcessor() {  }

    virtual std::string _ret Process ( std::string  command,
                                  Parameter & params,
                                  State & state,
                                  Actor & actor ) { };

    virtual bool CanProcess ( std::string & command )
    {
        bool result = boost::regex_match ( command, regex );
        return result;
    }

public:
    boost::regex regex;
};

class STOP_Processor : public CommandProcessor
{
public:
    STOP_Processor()
        : CommandProcessor("STOP")
    {
    }

    std::string _ret Process ( std::string command,
                               Parameter & params,
                               State & state,
                               Actor & actor )
    {
        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues(actor.ref_joint);

        std_msgs::String output;
        output.data = INTERFACE_CMD_STOP;
        actor.pInterfaceCommand.publish(output);
        ROS_INFO("send command [%s] succeed", output.data.c_str());

        return "succeed";
    }

};

class MOVE_Processor : public CommandProcessor
{
public:
    MOVE_Processor()
        : CommandProcessor("MOVE")
    {
    }

    std::string _ret Process ( std::string command,
                               Parameter & params,
                               State & state,
                               Actor & actor )
    {
        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues(actor.ref_joint);

        std_msgs::String output;
        output.data = INTERFACE_CMD_MOVE;
        actor.pInterfaceCommand.publish(output);
        ROS_INFO("send command [%s] succeed", output.data.c_str());

        return "succeed";
    }

};

class SPLINE_Processor : public CommandProcessor
{
public:
    SPLINE_Processor()
        : CommandProcessor("SPLINE")
    {
    }

    std::string _ret Process ( std::string command,
                               Parameter & params,
                               State & state,
                               Actor & actor )
    {
        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues(actor.ref_joint);

        std_msgs::String output;
        output.data = INTERFACE_CMD_SPLINE;
        actor.pInterfaceCommand.publish( output );
        ROS_INFO("send command [%s] succeed", output.data.c_str());

        return "succeed";
    }

};


class HOME_Processor : public CommandProcessor
{
public:
    HOME_Processor()
        : CommandProcessor("HOME")
    {
    }

    std::string _ret Process ( std::string command,
                               Parameter & params,
                               State & state,
                               Actor & actor )
    {
        state.no_joint_limitation = false;
        ROS_ERROR("[TEST] current NO_LIMIT flag [%d]", state.no_joint_limitation);

        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues(actor.ref_joint);

        std::vector<std::string>  join_input;
        join_input.clear();
        join_input.push_back(INTERFACE_CMD_HOME);

        bool pub_flag = true;
        if ( state.armControl.data[0] == 1.0 || state.armControl.data[0] == 2.0 || state.armControl.data[0] == 3.0 ||
             state.armControl.data[0] == 4.0 || state.armControl.data[0] == 5.0 || state.armControl.data[0] == 6.0 ||
             state.armControl.data[0] == 7.0 )
            join_input.push_back( HOME_AXIS[(int)state.armControl.data[0]] );
        else if ( state.armControl.data[0] == 9.0 )
            join_input.push_back( HOME_AXIS[0] );
        else
            pub_flag = false;

        if (pub_flag)
        {
            std_msgs::String output;
            output.data = boost::algorithm::join( join_input, " ");
            actor.pInterfaceCommand.publish(output);
            ROS_INFO("send command [%s] succeed", output.data.c_str());
        }
        else
        {
            ROS_INFO("invalid command [%s], abort it.", join_input[0].c_str());
        }

        return "succeed";
    }

};

class HOMESTOP_Processor : public CommandProcessor
{
public:
    HOMESTOP_Processor()
        : CommandProcessor("HOMESTOP")
    {
    }

    std::string _ret Process ( std::string command,
                               Parameter & params,
                               State & state,
                               Actor & actor )
    {
        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues(actor.ref_joint);

        std_msgs::String output;
        output.data = INTERFACE_CMD_HOMESTOP;
        actor.pInterfaceCommand.publish(output);
        ROS_INFO("send command [%s] succeed", output.data.c_str());

        return "succeed";
    }
};

class NOLIMIT_Processor : public CommandProcessor
{
public:
    NOLIMIT_Processor()
        : CommandProcessor("NOLIMIT")
    {
    }

    std::string _ret Process ( std::string command,
                               Parameter & params,
                               State & state,
                               Actor & actor )
    {
        state.no_joint_limitation = true;
        ROS_ERROR("[TEST] current NO_LIMIT flag [%d]", state.no_joint_limitation);
        return "succeed";
    }
};

class OTHERS_Processor : public CommandProcessor
{
public:
    OTHERS_Processor()
        : CommandProcessor("OTHERS")
    {
    }

    std::string _ret Process ( std::string command,
                               Parameter & params,
                               State & state,
                               Actor & actor )
    {
        return "succeed";
    }

};

class ARMJ_Processor : public CommandProcessor
{
public:
    ARMJ_Processor()
        : CommandProcessor("ARMJ")
    {
    }

    std::string _ret Process ( std::string  command,
                               Parameter & params,
                               State & state,
                               Actor & actor )
    {
        std::vector<double> VALUES;
        bool  ZERO = true;
        for (int i = 0; i < state.armControl.data.size(); ++i)
        {
            VALUES.push_back(state.armControl.data[i]);
        }
        for (int i = 0; i < VALUES.size(); ++i)
        {
            if (fabs(VALUES[i]) > 0.0001)
                ZERO = false;
        }

        if (ZERO)
        {
            actor.ref_joint = state.jointState;
            actor.ref_state->setVariableValues(actor.ref_joint);
            ROS_INFO("ARMJ: zero input");
            return "zero input";
        }

        if ((VALUES.size()) % JOINT_DOF != 0)
        {
            ROS_INFO("ARMJ: wrong number of points");
            return "wrong number of points";
        }

        std::vector<double> values(VALUES.size());

        for (size_t i = 0; i < values.size(); ++i)
        {
            // all of the joints are revolute
            values[i] = VALUES[i]/180.0*3.1415926 + actor.ref_joint.position[i];
        }
        std::vector<double> joint_values = values;

        std::cout << "current joints: " << std::endl;
        for (size_t i = 0; i < state.jointState.position.size(); ++i)
        {
            // all of the joints are revolute
            std::cout << state.jointState.position[i]*180.0/3.1415926 << std::endl;
        }

        std::vector<double> last_joint_values;
        actor.ref_state->copyJointGroupPositions ( params.manipulator.joint_group, last_joint_values );
        std::cout << "planned joints:" << std::endl;
        for (int i = 0; i <  joint_values.size(); ++i)
        {
            if (fabs((joint_values[i] - last_joint_values[i]) / params.gmas_time) > params.manipulator.max_vel[i])
            {
                ROS_INFO("ARMJ: over speed [%f]", fabs((joint_values[i] - last_joint_values[i]) / params.gmas_time));
                return "over speed";
            }
            std::cout << joint_values[i] << std::endl;
        }

        if( !state.no_joint_limitation )
        {
            for (int i = 0; i <  joint_values.size(); ++i)
            {
                if( joint_values[i] >= LIMIT_MANIPULATOR[2*i] )
                {
                    ROS_WARN("ARMJ: Joint%02d over forward limitation [%f]", i, joint_values[i]);
                    return "over position limitation";
                }
                else if( joint_values[i] <= LIMIT_MANIPULATOR[2*i+1] )
                {
                    ROS_INFO("ARMJ: Joint%02d over reverse limitation [%f]", i, joint_values[i]);
                    return "over position limitation";
                }
            }
        }

        trajectory_msgs::JointTrajectory output_command;
        output_command.header.stamp = ros::Time::now();
        output_command.joint_names = params.manipulator.joint_names;
        output_command.points.resize(1);
        output_command.points[0].time_from_start = ros::Duration(params.gmas_time);
        output_command.points[0].positions = joint_values;
        actor.pInterfaceJointPath.publish(output_command);

        actor.ref_joint.header.stamp = ros::Time::now();
        actor.ref_joint.name = params.manipulator.joint_names;
        actor.ref_joint.position = joint_values;
        actor.ref_state->setVariableValues(actor.ref_joint);

        ROS_INFO("ARMJ: succeed");
        return "succeed";
    }
};

class ARMC_Processor : public CommandProcessor
{
public:
    ARMC_Processor()
        : CommandProcessor("ARMC")
    {
    }

    std::string _ret Process ( std::string  command,
                          Parameter & params,
                          State & state,
                          Actor & actor )
    {
        std::vector<double> VALUES;
        bool  ZERO = true;
        for (size_t i = 0; i < state.armControl.data.size(); ++i)
        {
            VALUES.push_back(state.armControl.data[i]);
        }
        for (size_t i = 0; i < VALUES.size(); ++i)
        {
            if (fabs(VALUES[i]) > 0.0001)
                ZERO = false;
        }
        ROS_INFO("ARMC: explain command sucess");
        if (ZERO)
        {
            actor.ref_joint = state.jointState;
            actor.ref_state->setVariableValues(actor.ref_joint);
            ROS_INFO("ARMC: zero input");
            return "zero input";
        }

        if ((VALUES.size()) % CARTESIAN_DOF != 0)
        {
            ROS_INFO("ARMC: wrong number of points");
            return "wrong number of points";
        }
        //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        geometry_msgs::Twist in_twist;
        in_twist.linear.x = VALUES[0]/1000.0 ;
        in_twist.linear.y = VALUES[1]/1000.0 ;
        in_twist.linear.z = VALUES[2]/1000.0 ;
        in_twist.angular.x = VALUES[3]/180.0*3.1415926 ;
        in_twist.angular.y = VALUES[4]/180.0*3.1415926 ;
        in_twist.angular.z = VALUES[5]/180.0*3.1415926 ;

        tf::Vector3 twist_rot(in_twist.angular.x,
                              in_twist.angular.y,
                              in_twist.angular.z);
        tf::Vector3 twist_vel(in_twist.linear.x,
                              in_twist.linear.y,
                              in_twist.linear.z);
        // from the Base System to the Tip System
        tf::Vector3 out_rot1 = params.T2B_transform.getBasis() * twist_rot;
        tf::Vector3 out_vel1 = params.T2B_transform.getBasis() * twist_vel;
        // from the User System to the Tip System
        tf::Vector3 out_rot3 = params.T2U_transform.getBasis() * twist_rot;
        tf::Vector3 out_vel3 = params.T2U_transform.getBasis() * twist_vel;
        ROS_INFO("ARMC: command process transform success");

        //----------------------------------------------------New Add-----------------------------------------------------------------------------------------------------------
        geometry_msgs::Twist twist;
        int m_frame = 0;
        if ( state.userFrame == LINK_NAME_0 )
        {
            ROS_INFO("ARMC: select base frame");
            m_frame = 1;
        }
        else if ( state.userFrame == LINK_NAME_7 )
        {
            ROS_INFO("ARMC: select end effector frame");
            m_frame = 2;
        }
        else if ( state.userFrame == LINK_NAME_1 ||
                  state.userFrame == LINK_NAME_2 ||
                  state.userFrame == LINK_NAME_3 ||
                  state.userFrame == LINK_NAME_4 ||
                  state.userFrame == LINK_NAME_5 ||
                  state.userFrame == LINK_NAME_6 )
        {
            ROS_INFO("ARMC: select other user frame");
            m_frame = 3;
        }
        else
        {
            ROS_INFO("ARMC: select unkown frame");
            m_frame = 0;
        }

	m_frame = 2; // !!!!!!
        switch (m_frame)
        {
            case 1:
                twist.linear.x = out_vel1.x();
                twist.linear.y = out_vel1.y();
                twist.linear.z = out_vel1.z();
                twist.angular.x = out_rot1.x();
                twist.angular.y = out_rot1.y();
                twist.angular.z = out_rot1.z();
                break;
            case 2:
                twist = in_twist;
                break;
            case 3:
                if (state.newFrame)
                {
                    twist.linear.x = out_vel3.x();
                    twist.linear.y = out_vel3.y();
                    twist.linear.z = out_vel3.z();
                    twist.angular.x = out_rot3.x();
                    twist.angular.y = out_rot3.y();
                    twist.angular.z = out_rot3.z();
                }
                else
                {
                    ROS_WARN("No User Frame Yet! Using Tool Frame!");
                    twist = in_twist;
                }
                break;
            default:
                ROS_WARN("Unknown Frame! Using Tool Frame!");
                twist = in_twist;
                break;
        }
        //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        robot_state::RobotStatePtr target_state(new robot_state::RobotState(*actor.ref_state));
        bool found_ik = target_state->setFromDiffIK(params.manipulator.joint_group, twist, params.manipulator.eef, 1);
        if (!found_ik)
        {
            ROS_INFO("ARMC: did not find IK solution");
            return "did not find IK solution";
        }

        std::vector<double> joint_values;
        target_state->copyJointGroupPositions(params.manipulator.joint_group, joint_values);

        std::cout << "current joints: " << std::endl;
        for (size_t i = 0; i < state.jointState.position.size(); ++i)
        {
            // all the joints are revolute
            std::cout << "current joints: " << state.jointState.position[i]*180.0/3.1415926 << " ";
        }
        std::cout << std::endl;
        /*
        for (size_t i = 0; i < joint_values.size(); ++i)
        {
            std::cout << "planned joints: " << joint_values[i] << " ";
        }
        std::cout << std::endl;
        */

        std::vector<double> last_joint_values;
        actor.ref_state->copyJointGroupPositions(params.manipulator.joint_group, last_joint_values);
        for (int i = 0; i < joint_values.size(); ++i)
        {
            double v = fabs(joint_values[i] - last_joint_values[i]) / params.gmas_time;
            if  ( v > params.manipulator.max_vel[i] )
            {
                ROS_WARN("ARMC over speed: v[%d] = %f deg/s", i, v / 3.1415926 * 180.0);
                return "over speed";
            }
        }

        if( !state.no_joint_limitation )
        {
            for (int i = 0; i <  joint_values.size(); ++i)
            {
                if( joint_values[i] >= LIMIT_MANIPULATOR[2*i] )
                {
                    ROS_INFO("ARMJ: Joint%02d over forward limitation [%f]", i, joint_values[i]);
                    return "over position limitation";
                }
                else if( joint_values[i] <= LIMIT_MANIPULATOR[2*i+1] )
                {
                    ROS_INFO("ARMJ: Joint%02d over reverse limitation [%f]", i, joint_values[i]);
                    return "over position limitation";
                }
            }
        }

        trajectory_msgs::JointTrajectory output_command;
        output_command.joint_names = params.manipulator.joint_names;
        output_command.header.stamp = ros::Time::now();
        output_command.points.resize(1);
        output_command.points[0].time_from_start = ros::Duration(params.gmas_time);
        output_command.points[0].positions = joint_values;
        actor.pInterfaceJointPath.publish(output_command);

        actor.ref_joint.header.stamp = ros::Time::now();
        actor.ref_joint.name = params.manipulator.joint_names;
        actor.ref_joint.position = joint_values;
        actor.ref_state->setVariableValues(actor.ref_joint);

        ROS_INFO("ARMC: succeed");
        return "succeed";
    }
};

class MOVJ_LSPB_Processor : public CommandProcessor
{
public:
    MOVJ_LSPB_Processor()
        : CommandProcessor("MOVJ_LSPB")
    {
    }

    std::string _ret Process ( std::string  command,
                               Parameter & params,
                               State & state,
                               Actor & actor )
    {
        std::vector<double> VALUES;
        for (int i = 0; i < state.armControl.data.size()-1; ++i)
        {
            VALUES.push_back(state.armControl.data[i]);
        }
        double  TIME = state.armControl.data[ state.armControl.data.size()-1 ];

        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues(actor.ref_joint);
        ROS_INFO("MOVJ_LSPB: update state");

        if ((VALUES.size()) != JOINT_DOF)
        {
            ROS_INFO("MOVJ_LSPB: wrong number of points");
            return "wrong number of points";
        }
        if (TIME <= 0.0)
            return "time error";
        //
        std::vector<double> values( VALUES.size() );
        for (int i = 0; i < VALUES.size(); ++i)
            values[i] = VALUES[i]/180.0*3.1415926;
        //
        Eigen::Matrix<double, JOINT_DOF, 2>  joints_in;
        for (int i = 0; i < values.size(); ++i)
        {
            joints_in(i, 0) = actor.ref_joint.position[i];
            joints_in(i, 1) = values[i];
        }
        //
        colvecX  indexes = lspb(0.0, 1.0, TIME, params.gmas_time);
        Eigen::MatrixXd  joints_out = mcspline(joints_in, indexes);
        std::cout << "MOVJ_LSPB" << std::endl;

        trajectory_msgs::JointTrajectory  output_command;
        output_command.header.stamp = ros::Time::now();
        output_command.joint_names = params.manipulator.joint_names;
        output_command.points.resize( joints_out.cols() );

        std::vector<double>  last_joint_values = actor.ref_joint.position;
        std::vector<double>  joint_values( JOINT_DOF, 0.0 );
        for (int i = 0; i < joints_out.cols(); ++i)
        {
            joint_values.resize( joints_out.rows() );
            for (int j = 0; j < joints_out.rows(); ++j)
            {
                joint_values[j] = joints_out(j, i);
            }

            for (int j = 0; j < joint_values.size(); ++j)
            {
                double v = fabs(joint_values[j]-last_joint_values[j])/params.gmas_time;
                if (v > params.manipulator.max_vel[j])
                {
                    ROS_WARN ( "over speed: v[%ld] = %f (%f)", j+1, v / 3.1415926 * 180.0,
                               params.manipulator.max_vel[j] / 3.1415926 * 180.0);
                    return "over speed";
                }
            }

            if( !state.no_joint_limitation )
            {
                for (int j = 0; j <  joint_values.size(); ++j)
                {
                    if( joint_values[j] >= LIMIT_MANIPULATOR[2*j] )
                    {
                        ROS_INFO("MOVJ_LSPB: Joint%02d over forward limitation [%f]", j+1, joint_values[j]);
                        return "over position limitation";
                    }
                    else if( joint_values[j] <= LIMIT_MANIPULATOR[2*j+1] )
                    {
                        ROS_INFO("MOVJ_LSPB: Joint%02d over reverse limitation [%f]", j+1, joint_values[j]);
                        return "over position limitation";
                    }
                }
            }

            // fill the joint_values message
            last_joint_values = joint_values;
            output_command.points[i].time_from_start = ros::Duration( (i+1)*params.gmas_time );
            output_command.points[i].positions = joint_values;

            // just for test: print out the planned trajectory
            for(int k = 0 ; k < state.jointState.position.size(); ++k)
            {
                std::cout << std::setprecision(12) << state.jointState.position[k] << " ";
            }
            std::cout << ",";
            for(int k = 0 ; k < joint_values.size(); ++k)
            {
                std::cout << joint_values[k] << " ";
            }
            std::cout << std::endl;
        }

        actor.pInterfaceJointPath.publish( output_command );

        actor.ref_joint.header.stamp = ros::Time::now();
        actor.ref_joint.name = params.manipulator.joint_names;
        actor.ref_joint.position = joint_values;
        actor.ref_state->setVariableValues ( actor.ref_joint );

        ROS_INFO("MOVJ_LSPB: succeed");
        return "succeed";
    }

};


class MOVJ_MCHI_LSPB_Processor : public CommandProcessor
{
public:
    MOVJ_MCHI_LSPB_Processor()
        : CommandProcessor("MCHI_LSPB")
    {
    }

    std::string _ret Process ( std::string  command,
                               Parameter & params,
                               State & state,
                               Actor & actor )
    {
        std::vector<double> VALUES;
        for (int i = 0; i < state.armControl.data.size()-1; ++i)
        {
            VALUES.push_back(state.armControl.data[i]);
        }
        double  TIME = state.armControl.data[ state.armControl.data.size()-1 ];

        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues(actor.ref_joint);
        ROS_INFO("MOVJ_MCHI: update state");
        // just for test: print out the planned trajectory
        /*for(int k = 0 ; k < state.jointState.position.size(); ++k)
        {
            std::cout << state.jointState.position[k] << " ";
        }
        std::cout << std::endl;*/

        if ((VALUES.size()) % JOINT_DOF != 0)
        {
            ROS_INFO("MOVJ_MCHI: wrong number of points");
            return "wrong number of points";
        }
        if (TIME <= 0.0)
            return "time error";
	ROS_INFO("MOVJ_MCHI: all-ok");
        Eigen::Matrix<double, JOINT_DOF, Eigen::Dynamic>  joints_in;
        joints_in.resize(JOINT_DOF, VALUES.size()/JOINT_DOF);
        for (int i = 0; i < VALUES.size()/JOINT_DOF; ++i)
        {
            for (int j = 0; j < JOINT_DOF; ++j)
            {
                joints_in(j, i) = VALUES[j+i*JOINT_DOF]/180.0*3.1415926;
            }
        }
	ROS_INFO("MOVJ_MCHI: all-ok");
        colvecX  indexes = lspb(0.0, 1.0, TIME, params.gmas_time);
        Eigen::MatrixXd  joints_out = mcspline(joints_in, indexes);
        std::cout << "MOVJ_MCHI" << std::endl;

        // fill the messages into <trajectory_msgs::JointTrajectory> class
        trajectory_msgs::JointTrajectory  output_command;
        output_command.header.stamp = ros::Time::now();
        output_command.joint_names = params.manipulator.joint_names;
        output_command.points.resize( joints_out.cols() );

        std::vector<double>  last_joint_values = actor.ref_joint.position;
        std::vector<double>  joint_values( JOINT_DOF, 0.0 );
        for (int i = 0; i < joints_out.cols(); ++i)
        {
            joint_values.resize( joints_out.rows() );
            for (int j = 0; j < joints_out.rows(); ++j)
            {
                joint_values[j] = joints_out(j, i);
            }

            for (int j = 0; j < joint_values.size(); ++j)
            {
                double  v = fabs(joint_values[j]-last_joint_values[j])/params.gmas_time;
                if (v > params.manipulator.max_vel[j])
                {
                    ROS_WARN ( "over speed: v[%ld] = %f (%f)", j, v / 3.1415926 * 180.0,
                               params.manipulator.max_vel[j] / 3.1415926 * 180.0);
                    return "over speed";
                }
            }

            if( !state.no_joint_limitation )
            {
                for (int i = 0; i <  joint_values.size(); ++i)
                {
                    if( joint_values[i] >= LIMIT_MANIPULATOR[2*i] )
                    {
                        ROS_INFO("MOVJ_MCHI: Joint%02d over forward limitation [%f]", i, joint_values[i]);
                        return "over position limitation";
                    }
                    else if( joint_values[i] <= LIMIT_MANIPULATOR[2*i+1] )
                    {
                        ROS_INFO("MOVJ_MCHI: Joint%02d over reverse limitation [%f]", i, joint_values[i]);
                        return "over position limitation";
                    }
                }
            }

            // fill the joint_values message
            last_joint_values = joint_values;
            output_command.points[i].time_from_start = ros::Duration( (i+1)*params.gmas_time );
            output_command.points[i].positions = joint_values;

            // just for test: print out the planned trajectory
            for(int k = 0 ; k < state.jointState.position.size(); ++k)
            {
                std::cout << std::setprecision(12) << state.jointState.position[k] << " ";
            }
            std::cout << ",";
            for(int k = 0 ; k < joint_values.size(); ++k)
            {
                std::cout << joint_values[k] << " ";
            }
            std::cout << std::endl;
        }

        actor.pInterfaceJointPath.publish( output_command );

        actor.ref_joint.header.stamp = ros::Time::now();
        actor.ref_joint.name = params.manipulator.joint_names;
        actor.ref_joint.position = joint_values;
        actor.ref_state->setVariableValues ( actor.ref_joint );

        ROS_INFO("MOVJ_MCHI: succeed");
        return "succeed";
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm6_coordinator");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel( ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle			Node_Handle;
    robot_model_loader::RobotModelLoader ROBOT_MODEL_LOADER("robot_description");
    //moveit::planning_interface::PlanningSceneInterface  planning_scene_interface;
	
    StringHandllerOnce                  USER_FRAME_HANDLLER;
    StringHandllerOnce                  INTERFACE_STATE_HANDLLER;
    ArmCommandHandller                  ARM_COMMAND_HANDLLER;
    GripCommandHandller                 GRIP_COMMAND_HANDLLER;
    JointStateHandller                  JOINT_STATE_HANDLLER;
	
    ROS_INFO ( "waiting for data input." );

    Parameter   params;

    params.loop_time	= LOOP_TIME;
    params.gmas_time	= GMAS_TIME;
    params.robot_model  = ROBOT_MODEL_LOADER.getModel();

    params.userFrameTF.setIdentity();
    params.T2B_transform.setIdentity();
    params.T2U_transform.setIdentity();

    // initialize the manipulator part
    params.manipulator.joint_group = params.robot_model->getJointModelGroup(GROUP_MANIPULATOR);
    params.manipulator.joint_names = params.manipulator.group.getActiveJoints();
    params.manipulator.group.setPoseReferenceFrame(MANIPULATOR_BASE_LINK);
    params.manipulator.base = params.manipulator.group.getPoseReferenceFrame();
    params.manipulator.eef = params.manipulator.group.getEndEffectorLink();

    //-------------------- test-1 --------------------//
    for (int i=0; i<params.manipulator.joint_names.size(); ++i)
    {
        ROS_WARN ("joint_names[%d]: %s", i+1, params.manipulator.joint_names[i].c_str());
    }
    ROS_WARN ("Base: %s, End: %s.", params.manipulator.base.c_str(), params.manipulator.eef.c_str());
    //-------------------- test-1 --------------------//

    params.manipulator.max_vel.resize(params.manipulator.joint_names.size());
    for (int i=0; i<params.manipulator.joint_names.size(); ++i)
    {
        params.manipulator.max_vel[i] = LIMIT_REVOL / 180.0 * 3.1415926;
        ROS_INFO ( "manipulator max velocity of joint [%d] = %f deg/s",
                   i + 1, params.manipulator.max_vel[i] / 3.1415926 * 180.0 );
    }
    params.manipulator.max_joint_disp.resize(params.manipulator.joint_names.size());
    for (int i=0; i<params.manipulator.joint_names.size(); ++i)
    {
        params.manipulator.max_joint_disp(i) = params.manipulator.max_vel[i] * params.gmas_time;
        ROS_INFO("manipulator max joint displacement of joint [%d] = %f deg",
                 i + 1, params.manipulator.max_joint_disp(i) / 3.1415926 * 180.0);
    }

    ROS_INFO("manipulator end effector frame: %s, manipulator base link frame: %s",
             params.manipulator.eef.c_str(), params.manipulator.base.c_str());


    State	state;

    state.command_id = 8;
    state.commandID = COMMAND_ID[8];
    state.userFrame = LINK_NAME[7];
    state.interfaceState = "not_available";
    state.ExecuteResult = "";
    state.robotState.reset(new robot_state::RobotState ( params.robot_model ));

    state.jointState.name.resize(JOINT_DOF);
    state.jointState.position.resize(JOINT_DOF);
    state.jointState.velocity.resize(JOINT_DOF);
    state.jointState.effort.resize(JOINT_DOF);

    state.newFrame = false;// flag for new frame
    state.newArmCommand = false;// flag for new command
    state.no_joint_limitation = false;// flag for no_soft_limitation

    state.armControl.data.resize(JOINT_DOF);
    for (int i=0; i<JOINT_DOF; ++i)
    {
        state.armControl.data[i] = 0.0;
    }
    state.armControl.commandID = state.commandID;
    state.armControl.userFrame = state.userFrame;

    state.subUserFrame =
            Node_Handle.subscribe<std_msgs::String>(
                MANIPULATOR_USER_FRAME, 3, &StringHandllerOnce::callback,
                &USER_FRAME_HANDLLER);
    state.subArmCmd =
            Node_Handle.subscribe<mobot6_interface::ArmControl>(
                MANIPULATOR_COMMAND, 3, &ArmCommandHandller::callback,
                &ARM_COMMAND_HANDLLER);
    state.subGripCmd =
            Node_Handle.subscribe<mobot6_interface::GripControl>(
                GRIPPER_COMMAND, 3, &GripCommandHandller::callback,
                &GRIP_COMMAND_HANDLLER);
    state.subJointStates =
            Node_Handle.subscribe<sensor_msgs::JointState>(
                INTERFACE_JOINT_STATES, 3, &JointStateHandller::callback,
                &JOINT_STATE_HANDLLER);
    state.subInterfaceStates =
            Node_Handle.subscribe<std_msgs::String>(
                INTERFACE_STATES, 3, &StringHandllerOnce::callback,
                &INTERFACE_STATE_HANDLLER);

    Actor	actor;
    actor.ref_state.reset(new robot_state::RobotState ( params.robot_model ));

    actor.pInterfaceJointPath =
            Node_Handle.advertise<trajectory_msgs::JointTrajectory>(
                INTERFACE_JOINT_PATH_COMMAND, 1024, false);
    actor.pInterfaceCommand =
            Node_Handle.advertise<std_msgs::String>(
                INTERFACE_COMMAND, 1, false);
    actor.pInterfaceCommandGrip =
            Node_Handle.advertise<std_msgs::String>(
                INTERFACE_COMMAND_GRIP, 1, false);
    actor.pCoordinateStates =
            Node_Handle.advertise<std_msgs::String>(
                MANIPULATOR_STATES, 1, false);

    // create the tf broadcaster and listener
    //tf::TransformBroadcaster    tf_broadcaster;
    tf::TransformListener       tf_listener;

    // create the command processor
    STOP_Processor      processor_STOP;
    MOVE_Processor      processor_MOVE;
    SPLINE_Processor    processor_SPLINE;
    HOME_Processor      processor_HOME;
    HOMESTOP_Processor  processor_HOMESTOP;
    NOLIMIT_Processor   processor_NOLIMIT;
    OTHERS_Processor    processor_OTHERS;
    ARMJ_Processor      processor_ARMJ;
    ARMC_Processor      processor_ARMC;

    MOVJ_LSPB_Processor         processor_LSPB;
    MOVJ_MCHI_LSPB_Processor    processor_MCHI;

    // init the jointState and robotState of struct state and actor
    while ( ros::ok() )
    {
        ros::spinOnce();
        if ( JOINT_STATE_HANDLLER.newMsg() )
        {
            sensor_msgs::JointState temp_joint_state = JOINT_STATE_HANDLLER.fetchMsg();
            for (size_t i = 0; i < JOINT_DOF; ++i)
            {
                state.jointState.name[i] = temp_joint_state.name[i];
                state.jointState.position[i] = temp_joint_state.position[i];
                state.jointState.velocity[i] = temp_joint_state.velocity[i];
                //state.jointState.effort[i] = temp_joint_state.effort[i];
            }

            state.robotState->setVariableValues(state.jointState);
            actor.ref_joint = state.jointState;
            actor.ref_state->setVariableValues(actor.ref_joint);
            break;
        }
        ros::Duration(0.5).sleep();
        ROS_INFO("waiting for joint state update.");
    }


    ROS_INFO("main loop start");
    bool auto_ref = true;

    while (ros::ok())
    {
        try
        {
            double start = ros::Time::now().toSec();

            state.newFrame = false;
            state.newArmCommand = false;
            ros::spinOnce();

            double start_1 = ros::Time::now().toSec();
            if ( JOINT_STATE_HANDLLER.newMsg() )
            {
                // ROS_INFO("[part_1] get joint states feedback.");
                sensor_msgs::JointState temp_joint_state = JOINT_STATE_HANDLLER.fetchMsg();
                for (size_t i = 0; i < JOINT_DOF; ++i)
                {
                    state.jointState.name[i] = temp_joint_state.name[i];
                    state.jointState.position[i] = temp_joint_state.position[i];
                    state.jointState.velocity[i] = temp_joint_state.velocity[i];
                    //state.jointState.effort[i] = temp_joint_state.effort[i];
                }
                state.robotState->setVariableValues(state.jointState);
            }
            double cost_1 = ros::Time::now().toSec() - start_1;
            ROS_DEBUG("1. JOINT_STATE_HANDLLER end. cost_time: %lf ms.", cost_1*1000);

            double start_2 = ros::Time::now().toSec();
            bool interface_stable = false;
            bool interface_move   = false;
            bool interface_delay  = false;
            if ( INTERFACE_STATE_HANDLLER.newMsg() )
            {
                // ROS_INFO("[part_2] get interface states.");
                std_msgs::String    input = INTERFACE_STATE_HANDLLER.fetchMsg();
                state.interfaceState = input.data;
                std::vector<std::string> result;
                boost::split( result, state.interfaceState, boost::is_any_of("\t") );
                if ((result[0]==INTERFACE_STATE_MOVE) || (result[0]==INTERFACE_STATE_SPLINE))
                    interface_move = true;
                if (result[1] == INTERFACE_STATE_STABLE)
                    interface_stable = true;
                if (result[2] == INTERFACE_STATE_DELAYED)
                    interface_delay = true;
                // test: show out the split_number,
                // ROS_INFO("interface split_number[%d], result_0[%s], result_1[%s], interface_stable[%d], interface_move[%d].",
                         // result.size(), result[0].c_str(), result[1].c_str(), interface_stable, interface_move);
            }
            double cost_2 = ros::Time::now().toSec() - start_2;
            ROS_DEBUG("2. INTERFACE_STATE_HANDLLER end. cost_time: %lf ms.", cost_2*1000);


            double start_3 = ros::Time::now().toSec();
            if ( USER_FRAME_HANDLLER.newMsg() )
            {
                // ROS_INFO("[part_3] get user frame command.");
                state.newFrame = true;
                std_msgs::String user_frame_info = USER_FRAME_HANDLLER.fetchMsg();
                // ROS_INFO("show the user frame: %s", user_frame_info.data.c_str());
            }
            double cost_3 = ros::Time::now().toSec() - start_3;
            ROS_DEBUG("3. USER_FRAME_HANDLLER end. cost_time: %lf ms.", cost_3*1000);


            double start_4 = ros::Time::now().toSec();
            if ( ARM_COMMAND_HANDLLER.newMsg() )
            {
                ROS_INFO("[part_4] get arm command.");
                state.newArmCommand = true;
                state.armControl = ARM_COMMAND_HANDLLER.fetchMsg();
                state.commandID = state.armControl.commandID;
                state.userFrame = state.armControl.userFrame;

                params.manipulator.group.setPoseReferenceFrame (state.userFrame);
                // params.manipulator.group.setStartState(*state.robotState);

                // get the tansformation
                tf::StampedTransform T2B_transform, T2U_transform;
                try
                {
                    tf_listener.lookupTransform(params.manipulator.eef, params.manipulator.base,
                                                ros::Time(0), T2B_transform);
                    params.T2B_transform = T2B_transform;
                    ROS_INFO("eef:%s, base:%s, main loop: get the T2B transform success",
                             params.manipulator.eef.c_str(), params.manipulator.base.c_str());
                }
                catch (tf::TransformException ex)
                {
                    ROS_ERROR("%s", ex.what());
                }

                if (state.newArmCommand)
                {
                    try
                    {
                        tf_listener.lookupTransform(params.manipulator.eef, state.userFrame,
                                                    ros::Time(0), T2U_transform);
                        params.T2U_transform = T2U_transform;
                        ROS_INFO("eef:%s, user:%s, main loop: get the T2U transform success",
                                 params.manipulator.eef.c_str(), state.userFrame.c_str());
                    }
                    catch (tf::TransformException ex)
                    {
                        ROS_ERROR("%s", ex.what());
                    }
                }

                // process the command
                bool processed = false;

                // update reference, just for STOP command
                if ( auto_ref )
                {
                    actor.ref_joint = state.jointState;
                    actor.ref_state->setVariableValues(actor.ref_joint);
                }
                auto_ref = true;

                // handle the "OTHERS" command
                if (processor_OTHERS.CanProcess(state.commandID))
                {
                    ROS_DEBUG("***mainloop_OTHERS***");
                    processed = true;
                    state.ExecuteResult = processor_OTHERS.Process(
                                state.commandID, params, state, actor);
                }
                // handle the "STOP" command
                else if (processor_STOP.CanProcess(state.commandID))
                {
                    ROS_DEBUG("***mainloop_STOP***");
                    processed = true;
                    state.ExecuteResult = processor_STOP.Process(
                                state.commandID, params, state, actor);
                }
                // handle the "MOVE" command
                else if (processor_MOVE.CanProcess(state.commandID))
                {
                    ROS_INFO("***mainloop_MOVE***");
                    processed = true;
                    state.ExecuteResult = processor_MOVE.Process(
                                state.commandID, params, state, actor);
                }
                // handle the "SPLINE" command
                else if (processor_SPLINE.CanProcess(state.commandID))
                {
                    ROS_INFO("***mainloop_SPLINE***");
                    processed = true;
                    state.ExecuteResult = processor_SPLINE.Process(
                                state.commandID, params, state, actor);
                }
                // handle the "HOME" command
                else if (processor_HOME.CanProcess(state.commandID))
                {
                    ROS_DEBUG("***mainloop_HOME***");
                    processed = true;
                    state.ExecuteResult = processor_HOME.Process(
                                state.commandID, params, state, actor);
                }
                // handle the "HOMESTOP" command
                else if (processor_HOMESTOP.CanProcess(state.commandID))
                {
                    ROS_DEBUG("***mainloop_HOMESTOP***");
                    processed = true;
                    state.ExecuteResult = processor_HOMESTOP.Process(
                                state.commandID, params, state, actor);
                }
                // handle the "NOLIMIT" command
                else if (processor_NOLIMIT.CanProcess(state.commandID))
                {
                    ROS_DEBUG("***mainloop_NOLIMIT***");
                    processed = true;
                    state.ExecuteResult = processor_NOLIMIT.Process(
                                state.commandID, params, state, actor);
                }

                if (interface_move && !interface_delay)
                {
                    // handle the "ARMJ" command
                    if (processor_ARMJ.CanProcess(state.commandID))
                    {
                        ROS_DEBUG("***mainloop_ARMJ***");
                        processed = true;
                        state.ExecuteResult = processor_ARMJ.Process(
                                    state.commandID, params, state, actor);
                        auto_ref = false;
                    }
                    // handle the "ARMC" command
                    else if (processor_ARMC.CanProcess(state.commandID))
                    {
                        ROS_DEBUG("***mainloop_ARMC***");
                        processed = true;
                        state.ExecuteResult = processor_ARMC.Process(
                                    state.commandID, params, state, actor);
                        auto_ref = false;
                    }
                }
                else if (interface_move && interface_delay)
                {
                    ROS_ERROR("***mainloop_DELAY***");
                    processed = true;
                }

                // handle the LSPB && MCHI condition
                if (interface_move && interface_stable)
                {
                    // handle the "MCHI_LSPB" command
                    if (processor_MCHI.CanProcess(state.commandID))
                    {
                        ROS_DEBUG("***mainloop_MCHI_LSPB***");
                        processed = true;
                        state.ExecuteResult = processor_MCHI.Process(
                                    state.commandID, params, state, actor);
                        auto_ref = false;
                    }
                    // handle the "LSPB" command
                    else if (processor_LSPB.CanProcess(state.commandID))
                    {
                        ROS_DEBUG("***mainloop_LSPB***");
                        processed = true;
                        state.ExecuteResult = processor_LSPB.Process(
                                    state.commandID, params, state, actor);
                        auto_ref = false;
                    }
                }

                // handle unknown commands
                if ( !processed )
                {
                    ROS_WARN ( "unsupported command: %s", state.commandID.c_str() );
                    state.ExecuteResult = "No valid command!";
                    auto_ref = false;
                }
            }
            else
            {
                if (interface_move && interface_delay)
                {
                    ROS_ERROR("***mainloop_DELAY***");
                    actor.ref_joint = state.jointState;
                    actor.ref_state->setVariableValues(actor.ref_joint);
                }
            }

            double cost_4 = ros::Time::now().toSec() - start_4;
            ROS_DEBUG("4. ARM_COMMAND_HANDLLER end. cost_time: %lf ms.", cost_4*1000);


            double start_5 = ros::Time::now().toSec();
            if ( GRIP_COMMAND_HANDLLER.newMsg() )
            {
                // ROS_INFO("[part_5] get gripper command.");
                //
                state.gripControl = GRIP_COMMAND_HANDLLER.fetchMsg();
                std::string grip_command = state.gripControl.commandID;
                //
                if ((interface_move == true) && (interface_stable == true))
                {
                    if (grip_command == "OTHERS")  // it means stopping the gripper
                    {
                        std_msgs::String output;
                        output.data = GRIP_STOP;
                        actor.pInterfaceCommandGrip.publish(output);
                    }
                    else if (grip_command == "GRIP")
                    {
                        if (state.gripControl.data > 0.0)  // it means runing forward the gripper
                        {
                            std_msgs::String output;
                            output.data = GRIP_FORWARD;
                            actor.pInterfaceCommandGrip.publish(output);
                        }
                        else if (state.gripControl.data < 0.0)  // it means runing backward the gripper
                        {
                            std_msgs::String output;
                            output.data = GRIP_REVERSE;
                            actor.pInterfaceCommandGrip.publish(output);
                        }
                    }
                }
            }

            double cost_5 = ros::Time::now().toSec() - start_5;
            ROS_DEBUG("5. GRIP_COMMAND_HANDLLER end. cost_time: %lf ms.", cost_5*1000);

            double start_6 = ros::Time::now().toSec();
            // ROS_INFO("[part_6] publish coordinator states.");

            std_msgs::String    stateoutput;
            std::string         stateout;
            if (state.newArmCommand)
            {
                StringAppend(stateout, "new_command ");
            }
            else
            {
                if (interface_move)
                {
                    if (interface_stable)
                        StringAppend(stateout, "stable ");
                    else
                        StringAppend(stateout, "moving ");
                }
                else
                {
                    StringAppend(stateout, "stop_state/init_state ");
                }
            }

            if (state.no_joint_limitation)
                StringAppend(stateout, "no_pos_limit");
            else
                StringAppend(stateout, "pos_limit");

            Eigen::MatrixXd jcob = state.robotState->getJacobian ( params.manipulator.joint_group );
            Eigen::MatrixXd manipulability = jcob * jcob.transpose();
            StringAppend ( stateout, "%s ", state.commandID.c_str() );
            StringAppend ( stateout, "%s ", state.ExecuteResult.c_str() );
            StringAppend ( stateout, "%f ", manipulability.norm() );
            StringAppend ( stateout, "%d ", ARM_COMMAND_HANDLLER.size() );
            ROS_DEBUG ( "%s", stateout.c_str () );

            stateoutput.data = stateout;
            actor.pCoordinateStates.publish ( stateoutput );

            double cost_6 = ros::Time::now().toSec() - start_6;
            ROS_DEBUG("6. state_output end. cost_time: %lf ms.", cost_6*1000);

            double sleep_time = LOOP_TIME - ( ros::Time::now().toSec() - start );
            if ( sleep_time > 0 )
            {
                ros::Duration ( sleep_time ).sleep();
                ROS_DEBUG("Total cost Time: %lf ms", (LOOP_TIME-sleep_time)*1000);
            }
            else
                ROS_WARN ( "control loop over time: %f ms", -sleep_time*1000 );
        }
        catch (...)
        {
            ROS_ERROR("Maybe something is wrong!");
        }
    }
    ros::waitForShutdown();
    return 0;
}
