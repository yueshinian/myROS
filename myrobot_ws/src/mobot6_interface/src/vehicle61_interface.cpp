#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "mobot6_interface/InterfaceJointVel.h"


#include "modbus/modbus.h"
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

#define _in   const &   
#define _inc  const     
#define _out        &   
#define _ret  const     
#define _wt         &   
#define _rd   const &   

#define     INTERFACE_JOINT_STATES_VEHICLE  "/joint_states_vehicle"         
#define     INTERFACE_STATES_VEHICLE        "/interface_states_vehicle"     
#define     INTERFACE_PRESSURE              "/interface_pressure"           
#define     INTERFACE_COMMAND_VEHICLE       "/interface_command_vehicle"    
#define     INTERFACE_VELOCITY_VEHICLE      "/interface_velocity_vehicle"   

#define     INTERFACE_STATE_SPARATER        "\t"
#define     INTERFACE_STATE_BASE_MOVING     "BASE_MOVING"
#define     INTERFACE_STATE_BASE_STABLE     "BASE_STABLE"
#define     INTERFACE_STATE_SWING_MOVING    "SWING_MOVING"
#define     INTERFACE_STATE_SWING_STABLE    "SWING_STABLE"
#define     INTERFACE_STATE_VELOCITY        "VELOCITY"
#define     INTERFACE_STATE_HOME            "HOME"
#define     INTERFACE_STATE_RESET           "RESET"

#define     JOINT_VEHICLE_NUM               4
#define     JOINT_LEFT_SWINGARM             "joint_swingarm_left"
#define     JOINT_RIGHT_SWINGARM            "joint_swingarm_right"
#define     JOINT_LEFT_WHEEL                "base_l_wheel_joint"
#define     JOINT_RIGHT_WHEEL               "base_r_wheel_joint"

#define     VEHICLE_BASE_LINK               "base_link"
#define     VEHICLE_FIXED_FRAME             "odom"
#define     VEHICLE_ODOMETRY_FRAME          "base_footprint"

std::string VEHICLE_JOINT[] = {JOINT_LEFT_SWINGARM, JOINT_RIGHT_SWINGARM,
                               JOINT_LEFT_WHEEL, JOINT_RIGHT_WHEEL};

#define     SWING_CMD_MOVE                 "SWINGMOVE"
#define     SWING_CMD_HOME                 "SWINGHOME"
#define     SWING_CMD_HOMESTOP             "SWINGHOMESTOP"
#define     SWING_CMD_RESET                "SWINGRESET"

#define     VELOCITY_CONTINUE       99.0

#define     POSITIVE_DIRECTION      1
#define     NEGATIVE_DIRECTION      -1

#define     LIMIT_FORWARD_L_SWING   0.6545
#define     LIMIT_REVERSE_L_SWING   -0.7854
#define     LIMIT_FORWARD_R_SWING   0.6545
#define     LIMIT_REVERSE_R_SWING   -0.7854
double      LIMIT_SWINGARM[] = { LIMIT_FORWARD_L_SWING, LIMIT_REVERSE_L_SWING,
                                 LIMIT_FORWARD_R_SWING, LIMIT_REVERSE_R_SWING };

#define     COMMAND_POS             0
#define     INPUT_START_INDEX       2
#define     STATE_START_INDEX       30

#define     FUNC_STATE              30
#define     MODBUS_RUNNING          31
#define     REENTRANCE              32
#define     ERROR_FLAG              33
#define     SENSED_PRESSURE         34

#define     OpMode_Axis1            51
#define     OpMode_Axis2            61
#define     OpMode_Axis3            71
#define     OpMode_Axis4            81

#define     Status_Axis1            52
#define     Status_Axis2            62
#define     Status_Axis3            72
#define     Status_Axis4            82

#define     VelStatus_Axis1         53
#define     VelStatus_Axis2         63
#define     VelStatus_Axis3         73
#define     VelStatus_Axis4         83

#define     Pos_Axis1               54
#define     Pos_Axis2               64
#define     Pos_Axis3               74
#define     Pos_Axis4               84

#define     Vel_Axis1               56
#define     Vel_Axis2               66
#define     Vel_Axis3               76
#define     Vel_Axis4               86

int Map_OpMode[]    = { OpMode_Axis1, OpMode_Axis2, OpMode_Axis3, OpMode_Axis4 };
int Map_Status[]    = { Status_Axis1, Status_Axis2, Status_Axis3, Status_Axis4 };
int Map_VelStatus[] = { VelStatus_Axis1, VelStatus_Axis2, VelStatus_Axis3, VelStatus_Axis4 };
int Map_Position[]  = { Pos_Axis1, Pos_Axis2, Pos_Axis3, Pos_Axis4 };
int Map_Velocity[]  = { Vel_Axis1, Vel_Axis2, Vel_Axis3, Vel_Axis4 };

#define     LEFT_SWING_VELOCITY     2
#define     RIGHT_SWING_VELOCITY    4
#define     LEFT_BASE_VELOCITY      6
#define     RIGHT_BASE_VELOCITY     8

#define     LEFT_SWING_DIRECTION    3
#define     RIGHT_SWING_DIRECTION   5
#define     LEFT_BASE_DIRECTION     7
#define     RIGHT_BASE_DIRECTION    9

int Map_VelCmdMode[] = { LEFT_SWING_VELOCITY, RIGHT_SWING_VELOCITY, LEFT_BASE_VELOCITY, RIGHT_BASE_VELOCITY };
int Map_VelDirection[] = { LEFT_SWING_DIRECTION, RIGHT_SWING_DIRECTION, LEFT_BASE_DIRECTION, RIGHT_BASE_DIRECTION };

const int ERROR_UNHANDLEABLE_INPUT = 1;
const int ERROR_INVALID_TRAJECTORY = 2;
const int ERROR_INVALID_FEEDBACK = 3;

const int   MODE_RESET    = 0;
const int   MODE_VELOCITY = 1;
const int   MODE_HOME     = 2;

#define     MAX_AXES                4
#define     SWINGARM_NUM            2
#define     MODBUS_RATIO            1000.0
#define     PRESSURE_RATIO          1000000

#define     VEHICLE_COORD_LOOP      0.050
#define     VEHICLE_INTERFACE_LOOP  0.050
#define     VEHICLE_STATE_LOOP      0.150
#define     VEHICLE_GMAS_LOOP       0.025

bool        reset_flag  = 1;
modbus_t    *mb;


//--------------------------------------------------------------------------------
class InterfaceJointVelHandller
{
public:
    void callback(const mobot6_interface::InterfaceJointVel::ConstPtr & message)
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

    const mobot6_interface::InterfaceJointVel & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    mobot6_interface::InterfaceJointVel mCrtMsg;
    bool mNewMsg;
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


int ReadModbus_Long (int reg_pos)
{
    int regs, lVal;
    uint16_t reg_table[2] = {0};

    regs = modbus_read_registers(mb, reg_pos, 2, reg_table);
    if (regs == -1)
    {
        ROS_WARN("ERROR:%s: Read Modbus Fail.", __func__);
        return -111111;
    }

    lVal = (int)(reg_table[1]<<16 & 0xFFFF0000) | (reg_table[0] & 0xFFFF);
    return lVal;
}

int ReadModbus_Short (int reg_pos)
{
    int regs, lVal;
    uint16_t reg_table[1] = {0};

    regs = modbus_read_registers(mb, reg_pos, 1, reg_table);
    if (regs == -1)
    {
        ROS_WARN("ERROR:%s: Read Modbus Fail.", __func__);
        return -111111;
    }

    lVal = (int)((short)(reg_table[0] & 0xFFFF));
    return lVal;
}

int WriteModbus_Long (int reg_pos, const int lVal)
{
    int regs;
    uint16_t reg_table[2] = {0};

    reg_table[0] = (uint16_t)(lVal & 0xFFFF);
    reg_table[1] = (uint16_t)((lVal>>16) & 0xFFFF);

    regs = modbus_write_registers(mb, reg_pos, 2, reg_table);
    if (regs == -1)
    {
        ROS_WARN("ERROR:%s: Write Modbus Fail. regs[%d].", __func__, regs);
        return -111111;
    }
    return 1;
}

int WriteModbus_Short (int reg_pos, const int lVal)
{
    int regs;
    uint16_t reg_table[1] = {0};

    reg_table[0] = (uint16_t)(lVal);

    regs = modbus_write_registers(mb, reg_pos, 1, reg_table);
    if (regs == -1)
    {
        ROS_WARN("ERROR:%s: Write Modbus Fail. regs[%d].", __func__,regs);
        return -111111;
    }
    return 1;
}


int main(int argc, char **argv)
{
    ros::init ( argc, argv, "vehicle_interface" );
    ros::NodeHandle Node_Handle;
    ROS_INFO ( "initializing ROS succeed" );
    bool    p_RReset = false;
    ros::Duration(5.0).sleep();
    while ( ros::ok() )
    {
        ROS_INFO ( "mian loop started" );
        //
        const double            p_GMAS_TIME              = VEHICLE_GMAS_LOOP;
        const double            p_LOOP_TIME              = VEHICLE_INTERFACE_LOOP;
        const double            p_FEEDBACK_TIME          = VEHICLE_STATE_LOOP;

        const char              p_ROBOT_IP[]             = "192.168.1.127";     // "192.168.1.127"
        int                     p_ROBOT_PORT             = 502;

        ROS_INFO ( "setting parameters succeed" );
        //
        StringHandllerOnce              INTERFACE_COMMAND_HANDLLER;
        InterfaceJointVelHandller       INTERFACE_VELOCITY_HANDLLER;

        ros::Subscriber subInterfaceCommand =
                Node_Handle.subscribe<std_msgs::String>(
                    INTERFACE_COMMAND_VEHICLE, 3, &StringHandllerOnce::callback,
                    &INTERFACE_COMMAND_HANDLLER);
        ros::Subscriber subInterfaceVelocity =
                Node_Handle.subscribe<mobot6_interface::InterfaceJointVel>(
                    INTERFACE_VELOCITY_VEHICLE, 3, &InterfaceJointVelHandller::callback,
                    &INTERFACE_VELOCITY_HANDLLER);
        ros::Publisher  pJointStates =
                Node_Handle.advertise<sensor_msgs::JointState> (
                    INTERFACE_JOINT_STATES_VEHICLE, 3, false );
        ros::Publisher  pInterfaceStates =
                Node_Handle.advertise<std_msgs::String> (
                    INTERFACE_STATES_VEHICLE, 3, false );
        ros::Publisher  pPressureStates =
                Node_Handle.advertise<std_msgs::Float32> (
                    INTERFACE_PRESSURE, 3, false );
        ROS_INFO ( "initializing subscriber and publisher succeed" );
        //
        mb = modbus_new_tcp( p_ROBOT_IP, p_ROBOT_PORT );  
        modbus_set_slave( mb, 1 );  

        uint32_t    tv_sec = 0;
        uint32_t    tv_usec = 500000;  
        modbus_set_response_timeout( mb, tv_sec, tv_usec );

        int connect_flag = modbus_connect(mb);
        while(connect_flag)
        {
            ROS_INFO ( "Failed to connect to GMAS, res[%d].", connect_flag );
            for ( size_t i = 3; i > 0; --i )
            {
                ROS_INFO ( "Retry in %lu seconds.", i );
                ros::Duration(1).sleep();
            }
            connect_flag = modbus_connect(mb);
        }
        ROS_INFO ( "Connecting to GMAS successfully, res[%d].", connect_flag );
        //
        //---excpetions NO.1 is thrown - begin ------------------------------------------
        try
        {
            int iCnt = 0;
            if (reset_flag)
            {
                for (size_t i = 0; i < MAX_AXES; ++i)
                {
                    int resp = ReadModbus_Short( Map_Status[i] );
                    std::cout << "Joint_Status_" << i+1 <<": " << resp <<"\t";
                }
                std::cout <<std::endl;

                int resp = WriteModbus_Short( COMMAND_POS, 21 );
                if (resp > 0)
                    ROS_INFO( "Write the reset_flag(NO.21) process successfully." );
                else
                    ROS_WARN("Error: modbus write for reset_flag process failed.");

                bool check_21 = false;
                if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                {
                    check_21 = true;
                }
                while( check_21 )
                {
                    int regs1 = ReadModbus_Short(COMMAND_POS);
                    int regs2 = ReadModbus_Short(FUNC_STATE);
                    if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                        check_21 = true;
                    else
                        check_21 = false;
                    ROS_INFO("Wait for the end of the state_function 21. cmd_pos[%d], fun_state[%d]", regs1, regs2);
                    //
                    for (size_t i = 0; i < MAX_AXES; ++i)
                    {
                        int resp = ReadModbus_Short( Map_Status[i] );
                        std::cout << "Joint_Status_" << i+1 <<": " << resp <<"\t";
                    }
                    std::cout <<std::endl;
                    ros::Duration( 0.2 ).sleep();
                    //
                    iCnt++;
                    if(iCnt >= 150)
                    {
                        p_RReset = true;
                        iCnt = 0;
                        break;
                    }
                }
                //
                if( p_RReset )
                {
                    reset_flag = 1;
                    ROS_ERROR("The state_function 21 has been blocked, re-reset the vehicle.");
                }
                else if( !p_RReset )
                {
                    reset_flag = 0;
                    ROS_INFO("reset and re-enabled the motors ...");
                }
            }
            iCnt = 0;
            int check_count =0;
            for (size_t i = 0; i < MAX_AXES; ++i)
            {
                int regs1 = ReadModbus_Short(Map_OpMode[i]);
                int regs2 = ReadModbus_Short(Map_Status[i]);
                while (((regs1 != 1) || (regs2 != 1)) && (!p_RReset))
                {
                    regs1 = ReadModbus_Short( Map_OpMode[i] );
                    regs2 = ReadModbus_Short( Map_Status[i] );
                    ROS_ERROR("Axis [a%02d] is in OPMode[%d] and in Status[%d].\n", i+1, regs1, regs2);
                    ros::Duration(0.2).sleep();
                    check_count++;
                    if(check_count >= 300)
                    {
                        p_RReset = true;
                        check_count = 0;
                        break;
                    }
                }
                if( !p_RReset )
                {
                    ROS_INFO("Axis [a%02d] is in SYNC_POSITION Op-Mode[%d] and in STAND_STILL status[%d].\n", i+1, regs1, regs2);
                    iCnt++;
                }
                else if( p_RReset )
                {
                    break;
                }
            }
            if (iCnt == MAX_AXES)
            {
                ROS_INFO("Initializing GMAS succeed.");
            }
            int s_MODE = MODE_VELOCITY;
            if( p_RReset )
            {
                s_MODE = MODE_RESET;
                p_RReset = false;
            }
            //  define several variables before starting the second-level loop (marked as "s")
            bool                    s_BASE_MOVING               = false;
            bool                    s_SWING_MOVING              = false;
            bool                    s_HOME_CHECKMODE            = false;

            int                     s_REFRESH_COUNT             = 0;
            int                     s_HOME_COUNT                = 0;
            double                  s_LAST_FEEDBACK_TIME        = 0.0;
            std::vector<double>     s_LAST_FEEDBACK ( MAX_AXES, 0.0 );

            std::vector<double>     s_CURRENT_COMMAND ( MAX_AXES, 0.0 );
            std::vector<int>        s_CURRENT_VEL_DIR ( MAX_AXES, 0 );
            std::vector<double>     s_LAST_COMMAND ( MAX_AXES, 0.0 );
            std::vector<int>        s_LAST_VEL_DIR ( MAX_AXES, 0 );

            std::string     strActions;
            std::string     strStatus;
            strActions.reserve( 256 );
            strStatus.reserve( 256 );

            bool    f_swingarm_limitation[SWINGARM_NUM] = { false, false };
            bool    r_swingarm_limitation[SWINGARM_NUM] = { false, false };

            //--------------------------------------------------------------------------
            ROS_INFO ( "control loop started" );
            while( ros::ok() && s_MODE != MODE_RESET )
            {
                strActions.clear();
                double start = ros::Time::now().toSec();

                int iCnt;

                //---excpetions NO.2 is thrown - begin ------------------------------------------
                try
                {
                    ros::spinOnce();
                    if (s_MODE == MODE_HOME)
                    {
                        if ( s_HOME_COUNT <= 20 )
                        {
                            ROS_INFO("[part 2-2] re-check and re-set the operation mode.");
                            iCnt = 0;
                            std::cout << "[Axis Status in HOME_MODE]:";
                            for (size_t i = 0; i < MAX_AXES; ++i)
                            {
                                int resp = ReadModbus_Short(Map_Status[i]);
                                std::cout << "  Axis_" << i+1 << ": " << resp;
                                if ( resp == 1 )
                                    iCnt++;
                            }
                            std::cout << std::endl;

                            // if all axes are in STAND_STILL, add onr to s_HOME_COUNT, otherwise clear s_HOME_COUNT
                            if ( iCnt == MAX_AXES )
                                s_HOME_COUNT++;
                            else
                                s_HOME_COUNT = 0;
                        }
                        else
                        {
                            s_HOME_COUNT = 0;
                            int resp = WriteModbus_Long (COMMAND_POS, 22);
                            if (resp > 0)
                            {
                                ROS_INFO("[Important] Write the Mode_Switch(NO.22) process successfully.");
                                s_HOME_CHECKMODE = true;
                            }
                            else
                                ROS_ERROR("Error: modbus write for Mode_Switch(NO.22) process failed.");

                            int count = 0;
                            bool check_22 = false;
                            if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                            {
                                check_22 = true;
                            }
                            while( check_22 )
                            {
                                int regs1 = ReadModbus_Short(COMMAND_POS);
                                int regs2 = ReadModbus_Short(FUNC_STATE);
                                if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                    check_22 = true;
                                else
                                    check_22 = false;
                                ROS_INFO("Wait for the end of the state_function 22. cmd_pos[%d], fun_state[%d]", regs1, regs2);
                                ros::Duration( 0.2 ).sleep();
                            }
                        }
                        if ( s_HOME_CHECKMODE )
                        {
                            iCnt = 0;
                            std::cout << "[Axis OpMode in HOME_MODE]:";
                            for (size_t i = 0; i < MAX_AXES; ++i)
                            {
                                int resp = ReadModbus_Short(Map_OpMode[i]);
                                std::cout << "  Axis_" << i+1 << ": " << resp;
                                if ( resp == 1 ) // "CYCLIC POSITION"
                                    iCnt++;
                            }
                            std::cout << "count: " << iCnt << std::endl;

                            if (iCnt == MAX_AXES)
                            {
                                s_MODE = MODE_VELOCITY;
                                s_HOME_CHECKMODE = false;
                            }
                        }
                    }

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
                                reset_flag = 1;
                            }
                            else if ( input == SWING_CMD_HOME )
                            {
                                s_MODE = MODE_HOME;
                                s_HOME_COUNT = 0;
                                int resp = WriteModbus_Long (COMMAND_POS, 23); 
                                if (resp > 0)
                                    ROS_INFO("[Important] Write the Motor_Home(NO.23) process successfully.");
                                else
                                    ROS_ERROR("Error: modbus write for Motor_Home process failed.");

                                iCnt = 0;
                                bool check_23 = false;
                                if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                {
                                    check_23 = true;
                                }
                                while( check_23 )
                                {
                                    int regs1 = ReadModbus_Short(COMMAND_POS);
                                    int regs2 = ReadModbus_Short(FUNC_STATE);
                                    if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                    {
                                        check_23 = true;
                                        iCnt++;
                                    }
                                    else
                                        check_23 = false;
                                    ROS_INFO("Wait for the end of the state_function 23. cmd_pos[%d], fun_state[%d]", regs1, regs2);
                                    ros::Duration( 0.2 ).sleep();
                                    if (iCnt >= 100)
                                    {
                                        s_MODE = MODE_VELOCITY;
                                        ROS_FATAL("The state_function 23 has been blocked, please reset it or close the hardware.");
                                        break;
                                    }
                                }
                            }
                        }
                        //
                        else if ( s_MODE == MODE_HOME )
                        {
                            if ( input == SWING_CMD_HOMESTOP )
                            {
                                int resp = WriteModbus_Long (COMMAND_POS, 24); 
                                if (resp > 0)
                                    ROS_INFO("[Important] Write the Home_Stop(NO.24) process successfully.");
                                else
                                    ROS_ERROR("Error: modbus write for Home_Stop process failed.");

                                iCnt = 0;
                                bool check_24 = false;
                                if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                {
                                    check_24 = true;
                                }
                                while( check_24 )
                                {
                                    int regs1 = ReadModbus_Short(COMMAND_POS);
                                    int regs2 = ReadModbus_Short(FUNC_STATE);
                                    if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                        check_24 = true;
                                    else
                                        check_24 = false;
                                    ROS_INFO("Wait for the end of the state_function 24. cmd_pos[%d], fun_state[%d]", regs1, regs2);
                                    ros::Duration( 0.2 ).sleep();
                                }
                            }
                        }
                    }
                    double cost_1 = ros::Time::now().toSec() - start_1;
                    ROS_DEBUG("1. INTERFACE_COMMAND_HANDLLER end. cost_time: %lf ms.", cost_1*1000);

                    // read the interface command from "/interface_velocity_vehicle"
                    double start_2 = ros::Time::now().toSec();
                    if ( INTERFACE_VELOCITY_HANDLLER.newMsg() && s_MODE==MODE_VELOCITY )
                    {
                        ROS_DEBUG("[part 2-4] get the interface velocity from '/interface_velocity_vehicle'");

                        mobot6_interface::InterfaceJointVel  input = INTERFACE_VELOCITY_HANDLLER.fetchMsg();
                        //
                        for (size_t i = 0; i < SWINGARM_NUM; ++i)
                        {
                            if ((!f_swingarm_limitation[i]) && (!r_swingarm_limitation[i])) 
                            {
                                ROS_DEBUG("swingarm is in the appropriate range.");
                            }
                            else if ((f_swingarm_limitation[i]) && (!r_swingarm_limitation[i])) 
                            {
                                if ( (input.direction[i] == POSITIVE_DIRECTION) &&
                                     (input.command_id[i] > 0.0) )
                                {
                                    input.command_id[i] = 0.0;
                                    input.direction[i] = POSITIVE_DIRECTION;
                                    if( i == 0 )
                                        ROS_WARN("The forward limitation of left swingarm.");
                                    else if( i == 1 )
                                        ROS_WARN("The forward limitation of right swingarm.");
                                }
                            }
                            else if ((!f_swingarm_limitation[i]) && (r_swingarm_limitation[i])) // over the reverse limit
                            {
                                if ( (input.direction[i] == NEGATIVE_DIRECTION) &&
                                     (input.command_id[i] > 0.0) )
                                {
                                    input.command_id[i] = 0.0;
                                    input.direction[i] = POSITIVE_DIRECTION;
                                    if( i == 0 )
                                        ROS_WARN("The reverse limitation of left swingarm.");
                                    else if( i == 1 )
                                        ROS_WARN("The reverse limitation of right swingarm.");
                                }
                            }
                        }
                        //
                        for (size_t i = 0; i < input.joint_names.size(); ++i)
                        {
                            s_CURRENT_COMMAND[i] = input.command_id[i];
                            s_CURRENT_VEL_DIR[i] = input.direction[i];
                        }

                        ROS_DEBUG("get the interface_velocity command");

                        bool refresh_flag = false;
                        for (size_t i = 0; i < input.joint_names.size(); ++i)
                        {
                            if ((s_CURRENT_COMMAND[i] != s_LAST_COMMAND[i]) || (s_CURRENT_VEL_DIR[i] != s_LAST_VEL_DIR[i]))
                                refresh_flag = true;
                        }

                        if ( refresh_flag )
                            s_REFRESH_COUNT++;
                        else
                            s_REFRESH_COUNT = 0;

                        if (refresh_flag && s_REFRESH_COUNT>=3)
                        {
                            for (size_t i = 0; i < input.joint_names.size(); ++i)
                            {
                                if ((s_CURRENT_COMMAND[i] != s_LAST_COMMAND[i]) || (s_CURRENT_VEL_DIR[i] != s_LAST_VEL_DIR[i]))
                                {
                                    s_LAST_COMMAND[i] = s_CURRENT_COMMAND[i];
                                    s_LAST_VEL_DIR[i] = s_CURRENT_VEL_DIR[i];
                                }
                                else
                                {
                                    s_CURRENT_COMMAND[i] = VELOCITY_CONTINUE;
                                    s_CURRENT_VEL_DIR[i] = POSITIVE_DIRECTION;
                                }
                            }
                        }

                        for (size_t i = 0; i < input.joint_names.size(); ++i)
                        {
                            bool check_VelCmd = false;
                            int rep1 = ReadModbus_Short(Map_VelCmdMode[i]);
                            if (rep1 != 0) //|| (rep2 != 0))
                            {
                                check_VelCmd = true;
                            }
                            while( check_VelCmd )
                            {
                                if (ReadModbus_Short(Map_VelCmdMode[i])!=0)
                                    check_VelCmd = true;
                                else
                                    check_VelCmd = false;
                                ROS_ERROR("[ERROR] Refresh Command: Wait for Axis a%02d[%d] to read command.",
                                          i+1, rep1);
                                ros::Duration( 0.2 ).sleep();
                            }
                        }

                        if (refresh_flag && s_REFRESH_COUNT>=3) //
                        {
                            std::cout << "[TEST][TEST][TEST] New command: ";
                            for (size_t i = 0; i < input.joint_names.size(); ++i)
                                std::cout << "Joint_" << i+1 << ": " << s_CURRENT_COMMAND[i] << "  ";
                            std::cout << std::endl;
                            StringAppend ( strActions, "2. velocity_command\t" );

                            uint16_t reg_table[2*MAX_AXES] = {0};
                            for (size_t i = 0; i < input.joint_names.size(); ++i)
                            {
                                reg_table[2*i]   = (uint16_t)((int)(s_CURRENT_COMMAND[i]) & 0xFFFF);
                                reg_table[2*i+1] = (uint16_t)(s_CURRENT_VEL_DIR[i] & 0xFFFF);
                            }

                            int regs = modbus_write_registers(mb, INPUT_START_INDEX, 2*input.joint_names.size(), reg_table);
                            if (regs == -1)
                            {
                                ROS_ERROR("[ERROR] Write velocity command parameters to modbus failed. regs[%d].", regs);
                                return -111111;
                            }
ROS_ERROR("[Current Command] %d,%d",s_CURRENT_COMMAND[2],s_CURRENT_COMMAND[3]);
                            int resp = WriteModbus_Long (COMMAND_POS, 26);
                            if (resp > 0)
                                ROS_INFO("[Important] Write the Motor_Command(NO.26) process successfully.");
                            else
                                ROS_ERROR("Error: modbus write for Motor_Command process failed.");

                            bool check_25 = false;
                            if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                            {
                                check_25 = true;
                            }
                            iCnt = 0;
                            while( check_25 )
                            {
                                int regs1 = ReadModbus_Short(COMMAND_POS);
                                int regs2 = ReadModbus_Short(FUNC_STATE);
                                if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                    check_25 = true;
                                else
                                    check_25 = false;
                                ROS_WARN("Wait for the end of the state_function 25. cmd_pos[%d], fun_state[%d]", regs1, regs2);
                                ros::Duration( 0.2 ).sleep();

                                iCnt++;
                                if (iCnt == 100)
                                {
                                    s_MODE = MODE_RESET;
                                    reset_flag = 1;
                                    ROS_ERROR("ERROR: the block counter is over threshold, reset the motors (#25).");
                                    break;
                                }
                            }
                            s_REFRESH_COUNT = 0;
                        }
                    }
                    double cost_2 = ros::Time::now().toSec() - start_2;
                    ROS_DEBUG("2. INTERFACE_VELOCITY_HANDLLER end. cost_time: %lf ms.", cost_2*1000);

                    double start_3 = ros::Time::now().toSec();
                    if ( s_MODE != MODE_RESET )
                    {
                        ROS_DEBUG("[part 2-5] get the joint states from modbus.");
                        if ( ros::Time::now().toSec() + p_LOOP_TIME - s_LAST_FEEDBACK_TIME - p_FEEDBACK_TIME >= 0 )
                        {
                            StringAppend ( strActions, "2. joint_feedback\t" );
                            std::vector<int> feedback_reply( MAX_AXES );
                            std::vector<double> feedback( MAX_AXES );
                            std::vector<std::string> feedback_names( MAX_AXES );
                            std::vector<double> velocity( MAX_AXES );

                            for (size_t i = 0; i < MAX_AXES; ++i)
                            {
                                feedback_reply[i] = ReadModbus_Long(Map_Position[i]);
                                if (feedback_reply[i] == -111111)
                                {
                                    ROS_ERROR("Error: Axis feedback position [a%02d] is not available now.", i+1);
                                    throw Result ( ERROR_INVALID_FEEDBACK, "can not get feedback data" );
                                }
                            }

                            for (size_t i = 0; i < MAX_AXES; ++i)
                            {
                                try
                                {
                                    feedback[i] = ((double)feedback_reply[i])/MODBUS_RATIO/180.0*3.1415926;
                                    feedback_names[i] = VEHICLE_JOINT[i];
                                    velocity[i] = (feedback[i] - s_LAST_FEEDBACK[i])/(ros::Time::now().toSec() - s_LAST_FEEDBACK_TIME);
                                }
                                catch (...)
                                {
                                    throw Result ( ERROR_INVALID_FEEDBACK, "can not retrieve feedback data" );
                                }
                            }

                            sensor_msgs::JointState jointState;
                            jointState.header.stamp = ros::Time::now();
                            jointState.name = feedback_names;
                            jointState.position = feedback;
                            jointState.velocity = velocity;
                            pJointStates.publish( jointState );

                            s_LAST_FEEDBACK_TIME = ros::Time::now().toSec();
                            s_LAST_FEEDBACK = feedback;

                            for (size_t i = 0; i < SWINGARM_NUM; ++i)
                            {
                                if( feedback[i] >= LIMIT_SWINGARM[2*i] ) // the forward limitation of swingarms
                                {
                                    f_swingarm_limitation[i] = true;
                                    r_swingarm_limitation[i] = false;
                                }
                                else if( feedback[i] <= LIMIT_SWINGARM[2*i+1] ) // the reverse limitation of swingarms
                                {
                                    f_swingarm_limitation[i] = false;
                                    r_swingarm_limitation[i] = true;
                                }
                                else if ((feedback[i]<LIMIT_SWINGARM[2*i]) && (feedback[i]>LIMIT_SWINGARM[2*i+1]))
                                {
                                    f_swingarm_limitation[i] = false;
                                    r_swingarm_limitation[i] = false;
                                }
                            }
                        }
                    }
                    else
                    {
                        s_LAST_FEEDBACK_TIME = 0.0;
                        std::fill( s_LAST_FEEDBACK.begin(), s_LAST_FEEDBACK.end(), 0.0 );
                    }
                    double cost_3 = ros::Time::now().toSec() - start_3;
                    ROS_DEBUG("3. publish joint_states_vehicle end. cost_time: %lf ms.", cost_3*1000);

                    double start_4 = ros::Time::now().toSec();
                    std::vector<int>  s_Status( MAX_AXES, 0 );
                    ROS_DEBUG("[part 2-6] send status to coordinator.");
                    //
                    for (size_t i = 0; i < MAX_AXES; ++i)
                    {
                        int resp = ReadModbus_Short( Map_Status[i] );
                        while( resp <= 0 )
                        {
                            ROS_ERROR( "[ERROR] Axis a%02d can not get the right status, resp[%d].", i+1, resp );
                            resp = ReadModbus_Short( Map_Status[i] );
                            ros::Duration(0.2).sleep();
                        }
                        s_Status[i] = resp;
                    }
                    //
                    //  determine the s_SWING_MOVING states
                    if ((s_Status[0] == 5) || (s_Status[1] == 5)) //"ERROR_STOP"
                    {
                        s_SWING_MOVING = false;
                        ROS_ERROR("[ERROR] the swing axis is in [ERROR_STOP] status, reset them.");
                    }
                    else if ((s_Status[0] == 2) || (s_Status[1] == 2)) // "DISABLED"
                    {
                        s_SWING_MOVING = false;
                        ROS_ERROR("[ERROR] the swing axis is in [DISABLED] status, reset them.");
                    }
                    else if ((s_Status[0] == 1) && (s_Status[1] == 1)) // "STAND_STILL"
                    {
                        s_SWING_MOVING = false;
                    }
                    else
                    {
                        s_SWING_MOVING = true;
                    }
                    //
                    // determine the s_BASE_MOVING states
                    if ((s_Status[2] == 5) || (s_Status[3] == 5)) //"ERROR_STOP"
                    {
                        s_BASE_MOVING = false;
                        ROS_ERROR("[ERROR] the base axis is in [ERROR_STOP] status, reset them.");
                    }
                    else if ((s_Status[2] == 2) || (s_Status[3] == 2)) // "DISABLED"
                    {
                        s_BASE_MOVING = false;
                        ROS_ERROR("[ERROR] the base axis is in [DISABLED] status, reset them.");
                    }
                    else if ((s_Status[2] == 1) && (s_Status[3] == 1)) // "STAND_STILL"
                    {
                        s_BASE_MOVING = false;
                    }
                    else
                    {
                        s_BASE_MOVING = true;
                    }
                    //
                    //  fill in the interface state (s_MODE, s_SWING_MOVING and s_BASE_MOVING)
                    strStatus.clear();
                    if ( s_MODE == MODE_VELOCITY )
                        StringAppend ( strStatus, INTERFACE_STATE_VELOCITY INTERFACE_STATE_SPARATER );
                    else if ( s_MODE == MODE_HOME )
                        StringAppend ( strStatus, INTERFACE_STATE_HOME INTERFACE_STATE_SPARATER );
                    else if ( s_MODE == MODE_RESET )
                        StringAppend ( strStatus, INTERFACE_STATE_RESET INTERFACE_STATE_SPARATER );
                    //
                    if ( s_SWING_MOVING )
                        StringAppend ( strStatus, INTERFACE_STATE_SWING_MOVING INTERFACE_STATE_SPARATER );
                    else
                        StringAppend ( strStatus, INTERFACE_STATE_SWING_STABLE INTERFACE_STATE_SPARATER );
                    //
                    if ( s_BASE_MOVING )
                        StringAppend ( strStatus, INTERFACE_STATE_BASE_MOVING);
                    else
                        StringAppend ( strStatus, INTERFACE_STATE_BASE_STABLE);
                    //
                    // send status through "/interface_states_vehicle" topic
                    std_msgs::String output;
                    output.data = strStatus;
                    pInterfaceStates.publish( output );
                    //
                    //  get the error_flag information
                    int resp = ReadModbus_Short (ERROR_FLAG);
                    ROS_DEBUG("[Important] Error Flag: [%d]", resp);
                    //
                    //  send the pressure data to topic "/interface_pressure"
                    double resp1 = (double)ReadModbus_Long (SENSED_PRESSURE);
                    //ROS_INFO("[Important] sensed_pressure: [%f]", resp1);
                    std_msgs::Float32 press_msg;
                    press_msg.data = resp1/PRESSURE_RATIO;
                    pPressureStates.publish(press_msg);
                    //
                    double cost_4 = ros::Time::now().toSec() - start_4;
                    ROS_DEBUG("4. Send Status end. cost_time: %lf ms.", cost_4*1000);

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
            } // the second-level control-loop
            ROS_INFO("second-level control-loop ended");
        }
        //
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
        pJointStates.shutdown();
        pInterfaceStates.shutdown();
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
    } // main loop
    ros::waitForShutdown();
    return 0;
}





