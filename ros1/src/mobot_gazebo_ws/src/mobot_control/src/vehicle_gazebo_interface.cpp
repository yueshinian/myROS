#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "mobot_control/InterfaceJointVel.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <boost/system/error_code.hpp>
#include <boost/algorithm/string.hpp>
#include <log4cxx/logger.h>

#include <vector>
#include <deque>
#include <cerrno>
#include <cstdarg>
#include <cstring>
#include <stdexcept>
#include <stdint.h>

typedef     const char*     CStrPtr;

//--------------------------------------------------------------------------------

// 
#define _in   const &   
#define _inc  const   
#define _out        &  
#define _ret  const     
#define _wt         &   
#define _rd   const &   
// 
#define     INTERFACE_COMMAND_VEHICLE       "/interface_command_vehicle"   
#define     INTERFACE_VELOCITY_VEHICLE      "/interface_velocity_vehicle"  
#define     INTERFACE_JOINT_STATES          "/mobot_vehicle/joint_states"  

#define     INTERFACE_STATES_VEHICLE        "/interface_states_vehicle"   
#define     INTERFACE_SWINGARM_COMMAND      "/mobot_vehicle/swingarm_velocity_controllers/command"
// 
#define     INTERFACE_STATE_SPARATER        "\t"
#define     INTERFACE_STATE_BASE_MOVING     "BASE_MOVING"
#define     INTERFACE_STATE_BASE_STABLE     "BASE_STABLE"
#define     INTERFACE_STATE_SWING_MOVING    "SWING_MOVING"
#define     INTERFACE_STATE_SWING_STABLE    "SWING_STABLE"
#define     INTERFACE_STATE_VELOCITY        "VELOCITY"
#define     INTERFACE_STATE_RESET           "RESET"
// 
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
#define     VEHICLE_FIXED_FRAME             "odom"
#define     VEHICLE_ODOMETRY_FRAME          "base_footprint"

std::string VEHICLE_JOINT[] = { JOINT_LEFT_SWINGARM,
                                JOINT_RIGHT_SWINGARM,
                                JOINT_LEFT_FRONT_WHEEL,
                                JOINT_RIGHT_FRONT_WHEEL,
                                JOINT_LEFT_BACK_WHEEL,
                                JOINT_RIGHT_BACK_WHEEL };
// 
#define     SWING_CMD_MOVE                 "SWINGMOVE"
#define     SWING_CMD_RESET                "SWINGRESET"
// 
#define     LIMIT_FORWARD_L_SWING   0.6545
#define     LIMIT_REVERSE_L_SWING   -0.7854
#define     LIMIT_FORWARD_R_SWING   0.6545
#define     LIMIT_REVERSE_R_SWING   -0.7854
double      LIMIT_SWINGARM[] = { LIMIT_FORWARD_L_SWING, LIMIT_REVERSE_L_SWING,
                                 LIMIT_FORWARD_R_SWING, LIMIT_REVERSE_R_SWING };
// 
const int ERROR_UNHANDLEABLE_INPUT = 1;
const int ERROR_INVALID_TRAJECTORY = 2;
const int ERROR_INVALID_FEEDBACK = 3;

const int   MODE_RESET    = 0;
const int   MODE_VELOCITY = 1;
// 
#define     VEHICLE_INTERFACE_LOOP  0.020
#define     VEHICLE_GMAS_LOOP       0.020


//--------------------------------------------------------------------------------
//########################################################
//###  1. define the class - InterfaceCommandHandller  ###
//########################################################
class InterfaceJointVelHandller
{
public:
    void callback(const mobot_control::InterfaceJointVel::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    InterfaceJointVelHandller()
        : mCrtMsg()
        , mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    void clear()
    {
        mNewMsg = false;
    }

    const mobot_control::InterfaceJointVel & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    mobot_control::InterfaceJointVel mCrtMsg;
    bool mNewMsg;
};


//##################################################
//###  2. define the class - StringHandllerOnce  ###
//##################################################
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


//##################################################
//###  3. define the class - JointStateHandller  ###
//##################################################
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

    void clear()
    {
        mNewMsg = false;
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


//###########################################################
//###  4. define the Result Function and exception class  ###
//###########################################################
class Result
{
public:
  typedef int ID;
  static const ID SUCCESS = 0;

  Result ( ID _in l, std::string _in s )
    : mCode ( l ),
      mCatalog ( s )
  {
  }

  inline CStrPtr _ret message () const
  {
    return mCatalog.c_str();
  }

  inline bool _ret is_failed() const
  {
    return SUCCESS != mCode;
  }

  inline bool _ret is_succeed() const
  {
    return SUCCESS == mCode;
  }

  inline ID _rd code() const
  {
    return mCode;
  }

  static inline Result Success()
  {
    return Result ( 0, "success" );
  }

protected:
  ID                 mCode;
  std::string        mCatalog;
};


class exception
{
public:
    exception() throw() {  }

    virtual ~exception() throw() {  }

    exception ( std::exception _in e )
        : stde()
    {
        stde = e;
    }

    virtual CStrPtr _ret Message() const throw()
    {
        return "exception";
    }

protected:
    std::exception stde;
};


//#############################################
//###  5. define the StringAppend Function  ###
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


//#####################################
//###  6. define the Main Function  ###
//#####################################

int main(int argc, char **argv)
{
    ros::init ( argc, argv, "vehicle_interface" );
    ros::NodeHandle Node_Handle;
    ROS_INFO ( "initializing ROS succeed" );
    ros::Duration(1.5).sleep();
    // 
    while ( ros::ok() )
    {
        ROS_INFO ( "mian loop started" );
        //
        const double            p_GMAS_TIME              = VEHICLE_GMAS_LOOP;
        const double            p_LOOP_TIME              = VEHICLE_INTERFACE_LOOP;
        ROS_INFO ( "setting parameters succeed" );
        // 
        StringHandllerOnce              INTERFACE_COMMAND_HANDLLER;
        InterfaceJointVelHandller       INTERFACE_VELOCITY_HANDLLER;
        JointStateHandller              INTERFACE_JOINT_STATES_HANDLLER;

        ros::Subscriber subInterfaceCommand =
                Node_Handle.subscribe<std_msgs::String>(
                    INTERFACE_COMMAND_VEHICLE, 3, &StringHandllerOnce::callback,
                    &INTERFACE_COMMAND_HANDLLER);
        ros::Subscriber subInterfaceVelocity =
                Node_Handle.subscribe<mobot_control::InterfaceJointVel>(
                    INTERFACE_VELOCITY_VEHICLE, 3, &InterfaceJointVelHandller::callback,
                    &INTERFACE_VELOCITY_HANDLLER);
        ros::Subscriber subInterfaceJointStates =
                Node_Handle.subscribe<sensor_msgs::JointState>(
                    INTERFACE_JOINT_STATES, 3, &JointStateHandller::callback,
                    &INTERFACE_JOINT_STATES_HANDLLER);

        ros::Publisher  pInterfaceStates =
                Node_Handle.advertise<std_msgs::String> (
                    INTERFACE_STATES_VEHICLE, 3, false );
        ros::Publisher  pInterfaceSwingCmd =
                Node_Handle.advertise<std_msgs::Float64MultiArray> (
                    INTERFACE_SWINGARM_COMMAND, 1024, false );

        ROS_INFO ( "initializing subscriber and publisher succeed" );
        //---excpetions NO.1 is thrown - begin ------------------------------------------
        try
        {
            int s_MODE = MODE_VELOCITY;
            bool                    s_SWING_MOVING              = false;
            int                     s_REFRESH_COUNT             = 0;
            std::vector<double>     s_SWING_LAST_FEEDBACK ( JOINT_SWINGARM_NUM, 0.0 );

            std::vector<double>     s_CURRENT_COMMAND ( JOINT_SWINGARM_NUM, 0.0 );
            std::vector<double>     s_LAST_COMMAND ( JOINT_SWINGARM_NUM, 0.0 );

            std::string     strActions;
            std::string     strStatus;
            strActions.reserve( 256 );
            strStatus.reserve( 256 );

            bool    f_swingarm_limitation[JOINT_SWINGARM_NUM] = { false, false };
            bool    r_swingarm_limitation[JOINT_SWINGARM_NUM] = { false, false };

            //--------------------------------------------------------------------------
            // 2. initialize the second_level loop
            ROS_INFO ( "control loop started" );
            while( ros::ok() && s_MODE != MODE_RESET )
            {
                strActions.clear();
                double start = ros::Time::now().toSec();
                //---excpetions NO.2 is thrown - begin ------------------------------------------
                try
                {
                    ros::spinOnce();
                    double start_1 = ros::Time::now().toSec();
                    if ( INTERFACE_COMMAND_HANDLLER.newMsg() )
                    {
                        ROS_INFO("[part 2-3] get the interface command from '/interface_command_vehicle'");
                        std_msgs::String  input_ = INTERFACE_COMMAND_HANDLLER.fetchMsg();
                        std::string  input = input_.data;
                        boost::to_upper ( input );
                        std::cout << "get input " <<input << std::endl;
                        StringAppend ( strActions, "1. %s\t", input.c_str () );

                        if ( s_MODE == MODE_VELOCITY )
                        {
                            if ( input == SWING_CMD_RESET )
                            {
                                s_MODE = MODE_RESET;
                                ROS_WARN("VELOCITY STATE: RESET COMMAND");
                            }
                        }
                    }
                    double cost_1 = ros::Time::now().toSec() - start_1;
                    ROS_DEBUG("1. INTERFACE_COMMAND_HANDLLER end. cost_time: %lf ms.", cost_1*1000);

                    double start_2 = ros::Time::now().toSec();
                    if ( INTERFACE_VELOCITY_HANDLLER.newMsg() && s_MODE==MODE_VELOCITY )
                    {
                        ROS_DEBUG("[part 2-4] get the interface velocity from '/interface_velocity_vehicle'");
                        mobot_control::InterfaceJointVel  input = INTERFACE_VELOCITY_HANDLLER.fetchMsg();

                        for (size_t i = 0; i < JOINT_SWINGARM_NUM; ++i)
                        {
                            if ((!f_swingarm_limitation[i]) && (!r_swingarm_limitation[i]))
                            {
                                ROS_DEBUG("swingarm is in the appropriate range.");
                            }
                            else if ((f_swingarm_limitation[i]) && (!r_swingarm_limitation[i])) 
                            {
                                if ( input.command[i] > 0.0 )
                                {
                                    input.command[i] = 0.0;
                                    if( i == 0 )
                                        ROS_WARN("The forward limitation of left swingarm.");
                                    else if( i == 1 )
                                        ROS_WARN("The forward limitation of right swingarm.");
                                }
                            }
                            else if ((!f_swingarm_limitation[i]) && (r_swingarm_limitation[i])) 
                            {
                                if ( input.command[i] < 0.0 )
                                {
                                    input.command[i] = 0.0;
                                    if( i == 0 )
                                        ROS_WARN("The reverse limitation of left swingarm.");
                                    else if( i == 1 )
                                        ROS_WARN("The reverse limitation of right swingarm.");
                                }
                            }
                        }
                        for (size_t i = 0; i < input.joint_names.size(); ++i)
                        {
                            s_CURRENT_COMMAND[i] = input.command[i];
                        }
                        ROS_DEBUG("get the interface_velocity command");

                        //
                        bool refresh_flag = false;
                        for (size_t i = 0; i < input.joint_names.size(); ++i)
                        {
                            if (s_CURRENT_COMMAND[i] != s_LAST_COMMAND[i])
                                refresh_flag = true;
                        }
                        if ( refresh_flag )
                            s_REFRESH_COUNT++;
                        else
                            s_REFRESH_COUNT = 0;

                        // 
                        if (refresh_flag && s_REFRESH_COUNT>=3)
                        {
                            for (size_t i = 0; i < input.joint_names.size(); ++i)
                                s_LAST_COMMAND[i] = s_CURRENT_COMMAND[i];

                            std::cout << "New command: ";
                            for (size_t i = 0; i < input.joint_names.size(); ++i)
                                std::cout << "Joint_" << i+1 << ": " << s_CURRENT_COMMAND[i] << "  ";
                            std::cout << std::endl;
                            StringAppend ( strActions, "2. velocity_command\t" );
                            //
                            std_msgs::Float64MultiArray vel_output;
                            vel_output.data.resize( JOINT_SWINGARM_NUM );
                            for (size_t i = 0; i < JOINT_SWINGARM_NUM; ++i)
                                vel_output.data[i] = s_CURRENT_COMMAND[i];
                            pInterfaceSwingCmd.publish( vel_output );
                            s_REFRESH_COUNT = 0;
                        }
                    }
                    double cost_2 = ros::Time::now().toSec() - start_2;
                    ROS_DEBUG("2. INTERFACE_VELOCITY_HANDLLER end. cost_time: %lf ms.", cost_2*1000);

                    //
                    double start_3 = ros::Time::now().toSec();
                    if ( s_MODE != MODE_RESET )
                    {
                        int temp_count = 0;
                        if ( INTERFACE_JOINT_STATES_HANDLLER.newMsg() )
                        {
                            sensor_msgs::JointState input = INTERFACE_JOINT_STATES_HANDLLER.fetchMsg();
                            for (size_t i = 0; i < input.name.size(); ++i)
                            {
                                for (size_t j = 0; j < JOINT_SWINGARM_NUM; ++j)
                                {
                                    if( input.name[i] == VEHICLE_JOINT[j] )
                                    {
                                        s_SWING_LAST_FEEDBACK[j] = input.position[i];
                                        temp_count++;
                                    }
                                }
                            }
                            if( temp_count != JOINT_SWINGARM_NUM )
                                ROS_ERROR("Cannot get the swingarm jointStates. temp_count[%d]", temp_count);
                        }

                        for (size_t i = 0; i < JOINT_SWINGARM_NUM; ++i)
                        {
                            if( s_SWING_LAST_FEEDBACK[i] >= LIMIT_SWINGARM[2*i] ) 
                            {
                                f_swingarm_limitation[i] = true;
                                r_swingarm_limitation[i] = false;
                            }
                            else if( s_SWING_LAST_FEEDBACK[i] <= LIMIT_SWINGARM[2*i+1] )
                            {
                                f_swingarm_limitation[i] = false;
                                r_swingarm_limitation[i] = true;
                            }
                            else if ((s_SWING_LAST_FEEDBACK[i]<LIMIT_SWINGARM[2*i])
                                     && (s_SWING_LAST_FEEDBACK[i]>LIMIT_SWINGARM[2*i+1]))
                            {
                                f_swingarm_limitation[i] = false;
                                r_swingarm_limitation[i] = false;
                            }
                        }
                    }
                    double cost_3 = ros::Time::now().toSec() - start_3;
                    ROS_DEBUG("3. publish joint_states_vehicle end. cost_time: %lf ms.", cost_3*1000);
                    //
                    double start_4 = ros::Time::now().toSec();
                    ROS_DEBUG("[part 2-6] send status to coordinator.");

                    for (size_t i = 0; i < JOINT_SWINGARM_NUM; ++i)
                    {
                        if ( s_LAST_COMMAND[i] != 0.0 )
                            s_SWING_MOVING = true;
                    }

                    strStatus.clear();
                    if ( s_MODE == MODE_VELOCITY )
                        StringAppend ( strStatus, INTERFACE_STATE_VELOCITY INTERFACE_STATE_SPARATER );
                    else if ( s_MODE == MODE_RESET )
                        StringAppend ( strStatus, INTERFACE_STATE_RESET INTERFACE_STATE_SPARATER );
                    if ( s_SWING_MOVING )
                        StringAppend ( strStatus, INTERFACE_STATE_SWING_MOVING INTERFACE_STATE_SPARATER );
                    else
                        StringAppend ( strStatus, INTERFACE_STATE_SWING_STABLE INTERFACE_STATE_SPARATER );

                    std_msgs::String output;
                    output.data = strStatus;
                    pInterfaceStates.publish( output );

                    double cost_4 = ros::Time::now().toSec() - start_4;
                    ROS_DEBUG("4. Send Status end. cost_time: %lf ms.", cost_4*1000);

                    // loop time control
                    double sleep_time = p_LOOP_TIME - ( ros::Time::now().toSec() - start );
                    if ( sleep_time > 0 )
                    {
                        ros::Duration( sleep_time ).sleep();
                        ROS_DEBUG("5. Cost Time: %lf ms", (p_LOOP_TIME-sleep_time)*1000);
                    }
                    else
                        ROS_WARN ( "control loop over time: %f s", -sleep_time );
                }
                //---excpetions NO.2 is thrown - end --------------------------------------------
                catch (...)
                {
                    ROS_ERROR ( "a unhandleable unkown exception ourcced in teh second loop. emergency stop.\n"
                                "------------------------------NOTE------------------------------\n"
                                "There is NO tested method to recover the robot from this except-\n"
                                "ion right now.\n " );
                }
            } 
            ROS_INFO("second-level control-loop ended");
        }
        //---excpetions NO.1 is thrown - end -------------------------------------------
        catch ( std::exception ec )
        {
            ROS_ERROR ( "a unhandleable std exception ourcced. emergency stop.\n"
                        "------------------------------NOTE------------------------------\n"
                        "There is NO tested method to recover the robot from this except-\n"
                        "ion right now.\n "
                        "std::exception %s",
                        ec.what() );
        }
        catch ( boost::system::error_code ec )
        {
            ROS_ERROR ( "a unhandleable boost exception ourcced. emergency stop.\n"
                        "------------------------------NOTE------------------------------\n"
                        "There is NO tested method to recover the robot from this except-\n"
                        "ion right now.\n "
                        "boost::system::error_code %s:%s",
                        ec.category().name(),
                        ec.message().c_str() );
        }
        catch ( ... )
        {
            ROS_ERROR ( "a unhandleable unkown exception ourcced. emergency stop.\n"
                        "------------------------------NOTE------------------------------\n"
                        "There is NO tested method to recover the robot from this except-\n"
                        "ion right now.\n " );
        }
        //
        subInterfaceCommand.shutdown();
        subInterfaceVelocity.shutdown();
        subInterfaceJointStates.shutdown();
        pInterfaceStates.shutdown();
        pInterfaceSwingCmd.shutdown();
        ROS_INFO( "shutting down subscriber and publisher succeed" );
        //
        if ( ros::ok() )
        {
            for ( size_t i = 3; i > 0; --i )
            {
              ROS_INFO ( "restart main-loop in %lu seconds.", i );
              sleep ( 1 );
            }
        }
        //
        ROS_INFO( "main-loop ends" );
    } // main loop end
    ros::waitForShutdown();
    return 0;
}





