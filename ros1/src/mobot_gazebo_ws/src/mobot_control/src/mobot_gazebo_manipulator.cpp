#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include "mobot_control/ArmControl.h"
#include "mobot_control/GripControl.h"
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

#include <moveit/move_group_interface/move_group_interface.h>
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

//--------------------------------------------------------------------------------
#define     MANIPULATOR_USER_FRAME          "/arm_user_frame"    
#define     MANIPULATOR_COMMAND             "/arm_command"        
#define     MANIPULATOR_STATES              "/arm_states"          
#define     GRIPPER_COMMAND                 "/gripper_command"     

#define     INTERFACE_JOINT_STATES          "/mobot_vehicle/joint_states" 
#define     INTERFACE_STATES                "/interface_states"          
#define     INTERFACE_JOINT_PATH_COMMAND    "/joint_path_command"       
#define     INTERFACE_COMMAND               "/interface_command"           
#define     INTERFACE_COMMAND_GRIP          "/interface_command_grip"    

// define and enumerate the states of interface node
#define     INTERFACE_STATE_SPARATER    "\t"
#define     INTERFACE_STATE_MOVE        "MOVE"
#define     INTERFACE_STATE_INIT        "INIT"
#define     INTERFACE_STATE_STOP        "STOP"
#define     INTERFACE_STATE_HOME        "HOME"

#define     INTERFACE_STATE_MOVING      "MOVING"
#define     INTERFACE_STATE_STABLE      "STABLE"

#define     INTERFACE_STATE_DELAYED     "DELAYED"
#define     INTERFACE_STATE_NORMAL      "NORMAL"
// 
#define     INTERFACE_CMD_MOVE          "MOVE"
#define     INTERFACE_CMD_STOP          "STOP"

// 
#define     GROUP_MANIPULATOR           "manipulator"
// 
#define     MANIPULATOR_BASE_LINK       "11_arm_base_link"
#define     MANIPULATOR_END_EFFECTOR    "fake_arm_link"
// 
#define		JOINT_ARM_NUMBER        8
#define		JOINT1_ARM_NAME         "joint_turret_rotate"
#define		JOINT2_ARM_NAME         "joint_bigarm_pitch"
#define		JOINT3_ARM_NAME         "joint_bigarm_linear"
#define		JOINT4_ARM_NAME         "joint_forearm_pitch"
#define		JOINT5_ARM_NAME         "joint_forearm_rotate"
#define		JOINT6_ARM_NAME         "joint_wrist_pitch"
#define		JOINT7_ARM_NAME         "joint_wrist_rotate"
#define		JOINT8_ARM_NAME         "fake_arm_joint"
// 
#define		LINK_ARM_NUMBER         9
#define		LINK_NAME_0             "11_arm_base_link"
#define		LINK_NAME_1             "12_arm_turret_rotate"
#define		LINK_NAME_2             "13_arm_bigarm_pitch"
#define         LINK_NAME_3             "14_arm_bigarm_linear"
#define		LINK_NAME_4             "15_arm_forearm_pitch"
#define		LINK_NAME_5             "16_arm_forearm_rotate"
#define		LINK_NAME_6             "17_arm_wrist_pitch"
#define		LINK_NAME_7             "18_arm_wrist_rotate"
#define		LINK_NAME_8             "fake_arm_link"
// 
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
//
#define         GRIP_FORWARD            "GripForward"
#define         GRIP_REVERSE            "GripReverse"
#define         GRIP_STOP               "GripStop"
//
#define         HOME_AXIS_ALL           "AXISALL"
#define         HOME_AXIS_1             "AXIS1"
#define         HOME_AXIS_2             "AXIS2"
#define         HOME_AXIS_3             "AXIS3"
#define         HOME_AXIS_4             "AXIS4"
#define         HOME_AXIS_5             "AXIS5"
#define         HOME_AXIS_6             "AXIS6"
#define         HOME_AXIS_7             "AXIS7"
#define         HOME_AXIS_8             "AXIS8"

#define		JOINT_DOF               7  
#define		CARTESIAN_DOF           6
#define		LINK_NUM                9
#define         LINEAR_SCALE            5.0  

#define		LOOP_TIME               0.020
#define		GMAS_TIME               0.025

#define		LIMIT_REVOL             45.0
#define		LIMIT_PRIS              100.0

#define         _in                     const &
#define         _inc                    const
#define         _out                          &
#define         _ret                    const
#define         _wt                           &
#define         _rd                     const &

std::string     LINK_NAME[] = { LINK_NAME_0, LINK_NAME_1, LINK_NAME_2, LINK_NAME_3, LINK_NAME_4,
                                LINK_NAME_5, LINK_NAME_6, LINK_NAME_7, LINK_NAME_8 };
std::string     JOINT_NAME[] = { JOINT1_ARM_NAME, JOINT2_ARM_NAME, JOINT3_ARM_NAME, JOINT4_ARM_NAME,
                                 JOINT5_ARM_NAME, JOINT6_ARM_NAME, JOINT7_ARM_NAME, JOINT8_ARM_NAME };
std::string     COMMAND_ID[] = { COMMAND_ID_0, COMMAND_ID_1, COMMAND_ID_2, COMMAND_ID_3, COMMAND_ID_4,
                                 COMMAND_ID_5, COMMAND_ID_6, COMMAND_ID_7, COMMAND_ID_8, COMMAND_ID_9,
                                 COMMAND_ID_10, COMMAND_ID_11 };
std::string     HOME_AXIS[] = { HOME_AXIS_ALL, HOME_AXIS_1, HOME_AXIS_2, HOME_AXIS_3, HOME_AXIS_4,
                                HOME_AXIS_5, HOME_AXIS_6, HOME_AXIS_7, HOME_AXIS_8 };

// define the position limitation of each axis
#define         LIMIT_FORWARD_AXIS1     2.0943
#define         LIMIT_REVERSE_AXIS1     -2.0943

#define         LIMIT_FORWARD_AXIS2     0.7854
#define         LIMIT_REVERSE_AXIS2     -1.5708

#define         LIMIT_FORWARD_AXIS3     0.160
#define         LIMIT_REVERSE_AXIS3     -0.001 

#define         LIMIT_FORWARD_AXIS4     1.0472
#define         LIMIT_REVERSE_AXIS4     -3.4907

#define         LIMIT_FORWARD_AXIS5     3.1415
#define         LIMIT_REVERSE_AXIS5     -3.1415

#define         LIMIT_FORWARD_AXIS6     1.3963 
#define         LIMIT_REVERSE_AXIS6     -1.5708

#define         LIMIT_FORWARD_AXIS7     3.1415
#define         LIMIT_REVERSE_AXIS7     -3.1415

#define         LIMIT_FORWARD_GRIP      1.00
#define         LIMIT_REVERSE_GRIP      -1.00

double  LIMIT_MANIPULATOR[] = { LIMIT_FORWARD_AXIS1, LIMIT_REVERSE_AXIS1,
                                LIMIT_FORWARD_AXIS2, LIMIT_REVERSE_AXIS2,
                                LIMIT_FORWARD_AXIS3, LIMIT_REVERSE_AXIS3,
                                LIMIT_FORWARD_AXIS4, LIMIT_REVERSE_AXIS4,
                                LIMIT_FORWARD_AXIS5, LIMIT_REVERSE_AXIS5,
                                LIMIT_FORWARD_AXIS6, LIMIT_REVERSE_AXIS6,
                                LIMIT_FORWARD_AXIS7, LIMIT_REVERSE_AXIS7 };
double  LIMIT_GRIPPER[]     = { LIMIT_FORWARD_GRIP, LIMIT_REVERSE_GRIP };


//--------------------------------------------------------------------------------
//############################################
//###        Function  StringAppend        ###
//############################################
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

//################################################
//###    Function  ComputerVariableVelocity    ###
//################################################
void ComputeVariableVelocity(const robot_state::JointModelGroup* jmg,
                             Eigen::VectorXd& qdot,
                             const Eigen::VectorXd& twist,
                             const robot_state::LinkModel* tip,
                             const robot_state::RobotStatePtr& state,
                             const double lin_scale = 10.0)  
{
    int var_count = jmg->getVariableCount();
    Eigen::MatrixXd J(6, var_count);
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
    state->getJacobian(jmg, tip, reference_point, J, false);

    Eigen::Affine3d eMb = state->getGlobalLinkTransform(tip).inverse();
    Eigen::MatrixXd eWb = Eigen::ArrayXXd::Zero(6, 6);
    eWb.block(0, 0, 3, 3) = eMb.matrix().block(0, 0, 3, 3);
    eWb.block(3, 3, 3, 3) = eMb.matrix().block(0, 0, 3, 3);

    Eigen::VectorXd scale(var_count);
    for (int i=0; i<var_count; ++i)
    {
        if (i != 2)
            scale(i) = 1.0;
        else
            scale(i) = 1.0/lin_scale;
    }
    J = (eWb * J * scale.asDiagonal());

    Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd U = svdOfJ.matrixU();
    const Eigen::MatrixXd V = svdOfJ.matrixV();
    const Eigen::VectorXd S = svdOfJ.singularValues();

    Eigen::VectorXd Sinv = S;
    static const double pinvtoler = std::numeric_limits<float>::epsilon();
    double maxsv = 0.0;
    for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i)
    if (fabs(S(i)) > maxsv)
        maxsv = fabs(S(i));
    for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i)
    {
        if (fabs(S(i)) > maxsv * pinvtoler)
            Sinv(i) = 1.0 / S(i);
        else
            Sinv(i) = 0.0;
    }
    Eigen::MatrixXd Jinv = (V * Sinv.asDiagonal() * U.transpose());

    qdot = Jinv * twist;
    qdot(2) = qdot(2) / lin_scale;
    ROS_ERROR("[computer_jocbian] velocity:");
    for (int i=0; i<var_count; ++i)
        std::cout << qdot(i) << std::endl;
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
    moveit::planning_interface::MoveGroupInterface       group;
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
    mobot_control::ArmControl       armControl;
    mobot_control::GripControl      gripControl;

    sensor_msgs::JointState         jointState; 
    robot_state::RobotStatePtr      robotState;  
    ros::Subscriber                 subUserFrame;     
    ros::Subscriber                 subArmCmd;       
    ros::Subscriber                 subGripCmd;        
    ros::Subscriber                 subJointStates;     
    ros::Subscriber                 subInterfaceStates; 

    bool                            newFrame;
    bool                            newArmCommand;
    std::string                     ExecuteResult;
    bool                            no_joint_limitation; 
};

struct Actor
{
    sensor_msgs::JointState     ref_joint;
    robot_state::RobotStatePtr  ref_state;

    ros::Publisher              pInterfaceJointPath;    
    ros::Publisher              pCoordinateStates;    
    ros::Publisher              pInterfaceCommand;     
    ros::Publisher              pInterfaceCommandGrip; 
    ros::Publisher              pTest;                 
};


//#########################################
//###    define the command handller    ###
//#########################################
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
    void callback(const mobot_control::ArmControl::ConstPtr & message)
    {
        mCrtMsg.push_back(*message);
    }

    ArmCommandHandller()
        : mCrtMsg()
    {
    }

    const bool newMsg()
    {
        return mCrtMsg.size()>0;
    }

    const mobot_control::ArmControl fetchMsg()
    {
        mobot_control::ArmControl ret = mCrtMsg.front();
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
    std::list<mobot_control::ArmControl> mCrtMsg;
};

class GripCommandHandller
{
public:
    void callback(const mobot_control::GripControl::ConstPtr & message)
    {
        mCrtMsg.push_back(*message);
    }

    GripCommandHandller()
        : mCrtMsg()
    {
    }

    const bool newMsg()
    {
        return mCrtMsg.size()>0;
    }

    const mobot_control::GripControl fetchMsg()
    {
        mobot_control::GripControl ret = mCrtMsg.front();
        mCrtMsg.pop_front();
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
    std::list<mobot_control::GripControl> mCrtMsg;
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

//############################################
//###     define the command processor     ###
//############################################
class CommandProcessor
{
public:
    CommandProcessor(std::string x) : regex(x) {  }
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
        // 1. explain command

        // 3. update state.ref if required, else goto 4
        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues(actor.ref_joint);

        // 4. generate command

        // 6. publish
        std_msgs::String output;
        output.data = INTERFACE_CMD_STOP;
        actor.pInterfaceCommand.publish(output);
        ROS_INFO("send command [%s] succeed", output.data.c_str());

        // 7. update state of Actor class if required

        // 8. set actor.cur_error to success
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
        // 1. explain command

        // 3. update state.ref if required, else goto 4
        actor.ref_joint = state.jointState;
        actor.ref_state->setVariableValues(actor.ref_joint);

        // 4. generate command

        // 6. publish
        std_msgs::String output;
        output.data = INTERFACE_CMD_MOVE;
        actor.pInterfaceCommand.publish(output);
        ROS_INFO("send command [%s] succeed", output.data.c_str());

        // 7. update state of Actor class if required

        // 8. set actor.cur_error to success
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
        // 1. explain command

        // 3. update state.ref if required, else goto 4

        // 4. generate command

        // 6. publish

        // 7. update state of Actor class if required

        // 8. set actor.cur_error to success
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
        // 1. explain command
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

        // 3. update state.ref if required, else goto 4
        if (ZERO)
        {
            actor.ref_joint = state.jointState;
            actor.ref_state->setVariableValues(actor.ref_joint);
            ROS_INFO("ARMJ: zero input");
            return "zero input";
        }

        // 4. generate command
        if ((VALUES.size()) % JOINT_DOF != 0)
        {
            ROS_INFO("ARMJ: wrong number of points");
            return "wrong number of points";
        }

        std::vector<double> values(VALUES.size());

        for (size_t i = 0; i < values.size(); ++i)
        {
            if(i != 2) 
                values[i] = VALUES[i]/180.0*3.1415926 + actor.ref_joint.position[i];
            else      
                values[i] = VALUES[i]/1000.0 + actor.ref_joint.position[i];
        }
        std::vector<double> joint_values = values;

        std::cout << "current joints: " << std::endl;
        for (size_t i = 0; i < state.jointState.position.size(); ++i)
        {
            if (i != 2)
                std::cout << state.jointState.position[i]*180.0/3.1415926 << std::endl;
            else
                std::cout << state.jointState.position[i]*1000.0 << std::endl;
        }

        // 5. check over speed
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

        // 6. check position limitation
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

        // 7. publish to the "/joint_path_command" topic
        trajectory_msgs::JointTrajectory output_command;
        output_command.header.stamp = ros::Time::now();
        output_command.joint_names = params.manipulator.joint_names;
        output_command.points.resize(1);
        output_command.points[0].time_from_start = ros::Duration(params.gmas_time);
        output_command.points[0].positions = joint_values;
        actor.pInterfaceJointPath.publish(output_command);

        // 8. update state of Actor class if required
        actor.ref_joint.header.stamp = ros::Time::now();
        actor.ref_joint.name = params.manipulator.joint_names;
        actor.ref_joint.position = joint_values;
        actor.ref_state->setVariableValues(actor.ref_joint);

        // 9. set actor.cur_error to success
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
        // 1. explain command
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
        // 3. update state.ref if required, else goto 4
        if (ZERO)
        {
            actor.ref_joint = state.jointState;
            actor.ref_state->setVariableValues(actor.ref_joint);
            ROS_INFO("ARMC: zero input");
            return "zero input";
        }

        // 4. generate command
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

        tf::Vector3 out_rot1 = params.T2B_transform.getBasis() * twist_rot;
        tf::Vector3 out_vel1 = params.T2B_transform.getBasis() * twist_vel;

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
        else if ( state.userFrame == LINK_NAME_8 )
        {
            ROS_INFO("ARMC: select end effector frame");
            m_frame = 2;
        }
        else if ( state.userFrame == LINK_NAME_1 ||
                  state.userFrame == LINK_NAME_2 ||
                  state.userFrame == LINK_NAME_3 ||
                  state.userFrame == LINK_NAME_4 ||
                  state.userFrame == LINK_NAME_5 ||
                  state.userFrame == LINK_NAME_6 ||
                  state.userFrame == LINK_NAME_7 )
        {
            ROS_INFO("ARMC: select other user frame");
            m_frame = 3;
        }
        else
        {
            ROS_INFO("ARMC: select unkown frame");
            m_frame = 0;
        }

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
        Eigen::VectorXd qdot;
        Eigen::Matrix<double, 6, 1> twist_matrix;
        tf::twistMsgToEigen(twist, twist_matrix);

        ComputeVariableVelocity(params.manipulator.joint_group,
                                qdot,
                                twist_matrix,
                                target_state->getLinkModel(params.manipulator.eef),
                                target_state,
                                LINEAR_SCALE );
        bool found_ik = target_state->integrateVariableVelocity(params.manipulator.joint_group, qdot, 1);
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
            if (i != 2)
                std::cout << state.jointState.position[i]*180.0/3.1415926 << std::endl;
            else
                std::cout << state.jointState.position[i]*1000.0 << std::endl;
        }

        // 5. check over speed
        std::vector<double> last_joint_values;
        actor.ref_state->copyJointGroupPositions(params.manipulator.joint_group, last_joint_values);
        for (int i = 0; i < joint_values.size(); ++i)
        {
            double v = fabs(joint_values[i] - last_joint_values[i]) / params.gmas_time;
            if  ( v > params.manipulator.max_vel[i] )
            {
                if (i != 2)
                    ROS_WARN("ARMC over speed: v[%d] = %f deg/s", i, v / 3.1415926 * 180.0);
                else
                    ROS_WARN("ARMC over speed: v[%d] = %f mm/s", i, v * 1000.0);
                return "over speed";
            }
        }

        // 6. check position limitation
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

        // 7. publish to the "/joint_path_command" topic
        trajectory_msgs::JointTrajectory output_command;
        output_command.header.stamp = ros::Time::now();
        output_command.joint_names = params.manipulator.joint_names;
        output_command.points.resize(1);
        output_command.points[0].time_from_start = ros::Duration(params.gmas_time);
        output_command.points[0].positions = joint_values;
        actor.pInterfaceJointPath.publish(output_command);

        // 8. update state of Actor class if required
        actor.ref_joint.header.stamp = ros::Time::now();
        actor.ref_joint.name = params.manipulator.joint_names;
        actor.ref_joint.position = joint_values;
        actor.ref_state->setVariableValues(actor.ref_joint);

        // 9. set actor.cur_error to success
        ROS_INFO("ARMC: succeed");
        return "succeed";
    }
};


//################################
//###     Main   Programme     ###
//################################
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_coordinator");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle			Node_Handle;
    robot_model_loader::RobotModelLoader ROBOT_MODEL_LOADER("robot_description");
	
    StringHandllerOnce                  USER_FRAME_HANDLLER;
    StringHandllerOnce                  INTERFACE_STATE_HANDLLER;
    ArmCommandHandller                  ARM_COMMAND_HANDLLER;
    GripCommandHandller                 GRIP_COMMAND_HANDLLER;
    JointStateHandller                  JOINT_STATE_HANDLLER;
	
    //#############################################
    //###     initialize the Parameter Class    ###
    //#############################################
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

    params.manipulator.max_vel.resize(params.manipulator.joint_names.size());
    for (int i=0; i<params.manipulator.joint_names.size(); ++i)
    {
        if (i != 2) 
        {
            params.manipulator.max_vel[i] = LIMIT_REVOL / 180.0 * 3.1415926;
            ROS_INFO ( "manipulator max velocity of joint [%d] = %f deg/s",
                       i + 1, params.manipulator.max_vel[i] / 3.1415926 * 180.0 );
        }
        else 
        {
            params.manipulator.max_vel[i] = LIMIT_PRIS / 1000.0;
            ROS_INFO ( "manipulator max velocity of joint [%d] = %f mm/s",
                       i + 1, params.manipulator.max_vel[i] * 1000.0 );
        }
    }
    params.manipulator.max_joint_disp.resize(params.manipulator.joint_names.size());
    for (int i=0; i<params.manipulator.joint_names.size(); ++i)
    {
        params.manipulator.max_joint_disp(i) = params.manipulator.max_vel[i] * params.gmas_time;
        if (i != 2) 
        {
            ROS_INFO("manipulator max joint displacement of joint [%d] = %f deg",
                     i + 1, params.manipulator.max_joint_disp(i) / 3.1415926 * 180.0);
        }
        else  
        {
            ROS_INFO("manipulator max joint displacement of joint [%d] = %f mm",
                     i + 1, params.manipulator.max_joint_disp(i) * 1000.0);
        }
    }

    ROS_INFO("manipulator end effector frame: %s, manipulator base link frame: %s",
             params.manipulator.eef.c_str(), params.manipulator.base.c_str());

    //###############################################
    //###      initialize the State Structure     ###
    //###############################################
    State	state;

    state.command_id = 8;
    state.commandID = COMMAND_ID[8];
    state.userFrame = LINK_NAME[8];
    state.interfaceState = "not_available";
    state.ExecuteResult = "";
    state.robotState.reset(new robot_state::RobotState ( params.robot_model ));

    state.jointState.name.resize(JOINT_DOF);
    state.jointState.position.resize(JOINT_DOF);
    state.jointState.velocity.resize(JOINT_DOF);
    state.jointState.effort.resize(JOINT_DOF);

    state.newFrame = false;
    state.newArmCommand = false;
    state.no_joint_limitation = false;

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
            Node_Handle.subscribe<mobot_control::ArmControl>(
                MANIPULATOR_COMMAND, 3, &ArmCommandHandller::callback,
                &ARM_COMMAND_HANDLLER);
    state.subGripCmd =
            Node_Handle.subscribe<mobot_control::GripControl>(
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

    //################################################################
    //###      initialize the Actor Structure and other things     ###
    //################################################################
    Actor	actor;
    actor.ref_state.reset(new robot_state::RobotState ( params.robot_model ));
    actor.pInterfaceJointPath = Node_Handle.advertise<trajectory_msgs::JointTrajectory>(
                INTERFACE_JOINT_PATH_COMMAND, 1024, false);
    actor.pInterfaceCommand = Node_Handle.advertise<std_msgs::String>(
                INTERFACE_COMMAND, 1, false);
    actor.pInterfaceCommandGrip = Node_Handle.advertise<std_msgs::String>(
                INTERFACE_COMMAND_GRIP, 1, false);
    actor.pCoordinateStates = Node_Handle.advertise<std_msgs::String>(
                MANIPULATOR_STATES, 1, false);

    tf::TransformListener       tf_listener;

    STOP_Processor      processor_STOP;
    MOVE_Processor      processor_MOVE;
    OTHERS_Processor    processor_OTHERS;
    ARMJ_Processor      processor_ARMJ;
    ARMC_Processor      processor_ARMC;

    // init the jointState and robotState of struct state and actor
    while ( ros::ok() )
    {
        ros::spinOnce();
        if ( JOINT_STATE_HANDLLER.newMsg() )
        {
            sensor_msgs::JointState temp_joint_state = JOINT_STATE_HANDLLER.fetchMsg();
            for (size_t i = 0; i < temp_joint_state.name.size(); ++i)
            {
                for (size_t j = 0; j < JOINT_DOF; ++j)
                {
                    if( JOINT_NAME[j] == temp_joint_state.name[i] )
                    {
                        state.jointState.name[j] = temp_joint_state.name[i];
                        state.jointState.position[j] = temp_joint_state.position[i];
                        state.jointState.velocity[j] = temp_joint_state.velocity[i];
                    }
                }
            }
            state.robotState->setVariableValues(state.jointState);
            actor.ref_joint = state.jointState;
            actor.ref_state->setVariableValues(actor.ref_joint);
            break;
        }
        ros::Duration(5.0).sleep();
        ROS_INFO("waiting for joint state update.");
    }

    //#############################
    //###     MAIN     LOOP     ###
    //#############################

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

            // 1.
            double start_1 = ros::Time::now().toSec();
            if ( JOINT_STATE_HANDLLER.newMsg() )
            {
                sensor_msgs::JointState temp_joint_state = JOINT_STATE_HANDLLER.fetchMsg();
                for (size_t i = 0; i < temp_joint_state.name.size(); ++i)
                {
                    for (size_t j = 0; j < JOINT_DOF; ++j)
                    {
                        if( JOINT_NAME[j] == temp_joint_state.name[i] )
                        {
                            state.jointState.name[j] = temp_joint_state.name[i];
                            state.jointState.position[j] = temp_joint_state.position[i];
                            state.jointState.velocity[j] = temp_joint_state.velocity[i];
                        }
                    }
                }
                state.robotState->setVariableValues(state.jointState);
            }
            double cost_1 = ros::Time::now().toSec() - start_1;
            ROS_DEBUG("1. JOINT_STATE_HANDLLER end. cost_time: %lf ms.", cost_1*1000);

            // 2. 
            double start_2 = ros::Time::now().toSec();
            bool interface_stable = false;
            bool interface_move   = false;
            bool interface_delay  = false;
            if ( INTERFACE_STATE_HANDLLER.newMsg() )
            {
                std_msgs::String    input = INTERFACE_STATE_HANDLLER.fetchMsg();
                state.interfaceState = input.data;
                std::vector<std::string> result;
                boost::split( result, state.interfaceState, boost::is_any_of("\t") );
                if (result[0] == INTERFACE_STATE_MOVE)
                    interface_move = true;
                if (result[1] == INTERFACE_STATE_STABLE)
                    interface_stable = true;
                if (result[2] == INTERFACE_STATE_DELAYED)
                    interface_delay = true;
            }
            double cost_2 = ros::Time::now().toSec() - start_2;
            ROS_DEBUG("2. INTERFACE_STATE_HANDLLER end. cost_time: %lf ms.", cost_2*1000);

            // 3.
            double start_3 = ros::Time::now().toSec();
            if ( USER_FRAME_HANDLLER.newMsg() )
            {
                state.newFrame = true;
                std_msgs::String user_frame_info = USER_FRAME_HANDLLER.fetchMsg();
            }
            double cost_3 = ros::Time::now().toSec() - start_3;
            ROS_DEBUG("3. USER_FRAME_HANDLLER end. cost_time: %lf ms.", cost_3*1000);

            // 4.
            double start_4 = ros::Time::now().toSec();
            if ( ARM_COMMAND_HANDLLER.newMsg() )
            {
                ROS_INFO("[part_4] get arm command.");
                state.newArmCommand = true;
                state.armControl = ARM_COMMAND_HANDLLER.fetchMsg();
                state.commandID = state.armControl.commandID;
                state.userFrame = state.armControl.userFrame;

                params.manipulator.group.setPoseReferenceFrame (state.userFrame);

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

                bool processed = false;
		//
                if ( auto_ref )
                {
                    actor.ref_joint = state.jointState;
                    actor.ref_state->setVariableValues(actor.ref_joint);
                }
                auto_ref = true;
		//
                if (processor_OTHERS.CanProcess(state.commandID))
                {
                    ROS_DEBUG("***mainloop_OTHERS***");
                    processed = true;
                    state.ExecuteResult = processor_OTHERS.Process(
                                state.commandID, params, state, actor);
                }
                //
                else if (processor_STOP.CanProcess(state.commandID))
                {
                    ROS_DEBUG("***mainloop_STOP***");
                    processed = true;
                    state.ExecuteResult = processor_STOP.Process(
                                state.commandID, params, state, actor);
                }
                //
                else if (processor_MOVE.CanProcess(state.commandID))
                {
                    ROS_DEBUG("***mainloop_MOVE***");
                    processed = true;
                    state.ExecuteResult = processor_MOVE.Process(
                                state.commandID, params, state, actor);
                }
                //
                if (interface_move && !interface_delay)
                {
                    if (processor_ARMJ.CanProcess(state.commandID))
                    {
                        ROS_DEBUG("***mainloop_ARMJ***");
                        processed = true;
                        state.ExecuteResult = processor_ARMJ.Process(
                                    state.commandID, params, state, actor);
                        auto_ref = false;
                    }
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
                // 
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

            // 5. 
            double start_5 = ros::Time::now().toSec();
            if ( GRIP_COMMAND_HANDLLER.newMsg() )
            {
                state.gripControl = GRIP_COMMAND_HANDLLER.fetchMsg();
                std::string grip_command = state.gripControl.commandID;
                //
                if ((interface_move == true) && (interface_stable == true))
                {
                    if (grip_command == "OTHERS") 
                    {
                        std_msgs::String output;
                        output.data = GRIP_STOP;
                        actor.pInterfaceCommandGrip.publish(output);
                    }
                    else if (grip_command == "GRIP")
                    {
                        if (state.gripControl.data == 1.0) 
                        {
                            std_msgs::String output;
                            output.data = GRIP_FORWARD;
                            actor.pInterfaceCommandGrip.publish(output);
                        }
                        else if (state.gripControl.data == -1.0) 
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

            // 6. 
            double start_6 = ros::Time::now().toSec();

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

            // 7. 
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
