#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "mobot6_interface/NuclearControl.h"

#include <serial/serial.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <math.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <log4cxx/logger.h>

#include <vector>

typedef     const char*     CStrPtr;
//--------------------------------------------------------------------------------

// 1. define the input and output label macros
#define     _in     const &   // input parameter of right value
#define     _inc    const     // input parameter of right value
#define     _out          &   // output parameter of left value
#define     _ret    const     // return by value (use c++ complier optimaization)
#define     _wt           &   // return value of writable left value
#define     _rd     const &   // return value of readonly left value

// 2. define the states of interface node
#define     INTERFACE_SPARATER          "\t"
#define     INTERFACE_NOTHING           "NaN"

#define     INTERFACE_RESP_NORMAL       "NORMAL"
#define     INTERFACE_RESP_OVERTIME     "OVERTIME"
#define     INTERFACE_RESP_UNMATCH      "UNMATCH"
#define     INTERFACE_RESP_UNREADY      "UNREADY"

#define     INTERFACE_STATE_MOVING      "MOVING"
#define     INTERFACE_STATE_STABLE      "STABLE"

#define     INTERFACE_MODE_MOVE         "MOVE"
#define     INTERFACE_MODE_HOME         "HOME"
#define     INTERFACE_MODE_RESET        "RESET"

// 3. define the joint_names and corresponding measuremnts
#define     JOINT_NUCLEAR_NUM           3
std::string SENSOR_JOINT[] = { "joint_sensor", "intensity_gm", "intensity_neutron" };

// 4. define
#define     INTERFACE_COMMAND_NUCLEAR       "/interface_command_nuclear"
#define     INTERFACE_STATES_NUCLEAR        "/interface_states_nuclear"
#define     INTERFACE_JOINT_STATES_NUCLEAR  "/joint_states_nuclear"

#define     STP_CMD_MOVE        "STEPMOVE"
#define     STP_CMD_HOME        "STEPHOME"
#define     STP_CMD_STOP        "STEPSTOP"
#define     STP_CMD_RESET       "STEPRESET"
#define     STP_CMD_REQ         "STEPREQ"   // inertial command
#define     STP_CMD_OTHER       "STEPOTHER" // inertial command

#define     STP_STRING_MOVE1    "{\"Seq\":1,\"DeviceID\":\"C0001\",\"Command\":\"COM_STP_POINT\",\"Data\":[{\"Point\":\"75.000\",\"Speed\":10}]}\n"
#define     STP_STRING_MOVE     "{\"Seq\":1,\"DeviceID\":\"C0001\",\"Command\":\"COM_STP_POINT\",\"Data\":[{\"Point\":\"CMD_POS\",\"Speed\":CMD_SPEED}]}\n"
#define     STP_STRING_HOME     "{\"Seq\":1,\"DeviceID\":\"C0001\",\"Command\":\"COM_STP_ZERO\"}\n"
#define     STP_STRING_STOP     "{\"Seq\":1,\"DeviceID\":\"C0001\",\"Command\":\"COM_STP_STOP\"}\n"
#define     STP_STRING_REQ      "{\"Seq\":1,\"DeviceID\":\"C0001\",\"Command\":\"COM_VAL_REQ\"}\n"

#define     CMD_STRING_POS      "CMD_POS"
#define     CMD_STRING_SPEED    "CMD_SPEED"
#define     CMD_STRING_MOVE     "COM_STP_POINT"
#define     CMD_STRING_HOME     "COM_STP_ZERO"
#define     CMD_STRING_STOP     "COM_STP_STOP"
#define     CMD_STRING_REQ      "COM_VAL_REQ"

#define     NUCLEAR_STATE_LOOP  0.300

const int   MODE_RESET    = 0;
const int   MODE_MOVE     = 1;
const int   MODE_HOME     = 2;


//######################################################
//###  1. define the class - NuclearCommandHandller  ###
//######################################################
class NuclearCommandHandller
{
public:
    void callback(const mobot6_interface::NuclearControl::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    NuclearCommandHandller()
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

    const mobot6_interface::NuclearControl & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    mobot6_interface::NuclearControl mCrtMsg;
    bool mNewMsg;
};


//###################################################
//###  2. define the function - CommandProcessor  ###
//###################################################
int CommandProcessor( const std::string command,
                      const double param_1,
                      const double param_2,
                      std::string& output )
{
    std::string     output_;
    if ( command == STP_CMD_MOVE )
    {
        std::stringstream ss_;
        ss_ << std::fixed << std::setprecision(2) << param_1;

        std::string input_ = STP_STRING_MOVE;
        std::string str_param1 = ss_.str();
        std::string str_param2 = std::to_string((int)(ceil(param_2)));
        boost::algorithm::replace_all(input_, CMD_STRING_POS, str_param1);
        boost::algorithm::replace_all(input_, CMD_STRING_SPEED, str_param2);
        output_ = input_;
        std::cout << "[aaaa]" << output_ << std::endl;
        std::cout << "[bbbb]" << STP_STRING_MOVE1 << std::endl;
    }
    else if ( command == STP_CMD_HOME )
    {
        output_ = STP_STRING_HOME;
    }
    else if ( command == STP_CMD_STOP )
    {
        output_ = STP_STRING_STOP;
    }
    else if ( command == STP_CMD_REQ )
    {
        output_ = STP_STRING_REQ;
    }
    else if ( command == STP_CMD_OTHER )
    {
        return 666;
    }
    else
    {
        ROS_ERROR("[Function CommandProcessor] unavailable command: %s", command.c_str());
        return -111;
    }

    output = output_;
    return 0;
}


//####################################################
//###  3. define the function - CommandExtraction  ###
//####################################################
int CommandExtraction( std::string input,
                       std::string& command,
                       int& result,
                       double& param_1,
                       double& param_2,
                       double& param_3)
{
    std::vector<std::string> split_result;
    boost::split( split_result, input, boost::is_any_of(" \":{}[],"),
                  boost::algorithm::token_compress_on );
    /*
    for (size_t i=0; i<split_result.size(); ++i)
        std::cout<<i+1<<": "<<split_result[i]<<"\t";
    std::cout<<std::endl;
    */
    if ((split_result.size()>10) && (split_result.size()<18))
    {
        std::string string_check = split_result[6];
        ROS_INFO("[Function CommandExtraction] %s", string_check.c_str());
        if (string_check == CMD_STRING_MOVE)
        {
            command = split_result[6];
            result = atoi(split_result[8].c_str());
            param_1 = atof(split_result[11].c_str());
            param_2 = 0.0;
            param_3 = 0.0;
        }
        else if (string_check == CMD_STRING_HOME)
        {
            command = split_result[6];
            result = atoi(split_result[8].c_str());
            param_1 = 0.0;
            param_2 = 0.0;
            param_3 = 0.0;
        }
        else if (string_check == CMD_STRING_STOP)
        {
            command = split_result[6];
            result = atoi(split_result[8].c_str());
            param_1 = atof(split_result[11].c_str());
            param_2 = 0.0;
            param_3 = 0.0;
        }
        else if (string_check == CMD_STRING_REQ)
        {
            // char *offset;
            command = split_result[6];
            result = atoi(split_result[8].c_str());
            param_1 = atof(split_result[11].c_str());
            param_2 = atof(split_result[13].c_str());
            param_3 = 0.0;
            // param_3 = (double)strtol(split_result[14].c_str(),&offset,16);
        }
        else
        {
            //ROS_ERROR("[Function CommandExtraction] unavailable string: %s", string_check.c_str());20190329
            return -111;
        }
    }
    else
    {
        //ROS_ERROR("[Function CommandExtraction] unavailable string: %s", input.c_str());20190329
        return -111;
    }
    return 0;
}


//#############################################
//###  4. define the StringAppend Function  ###
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
//###  5. define the Main Function  ###
//#####################################
int main(int argc, char** argv)
{
    ros::init ( argc, argv, "nuclear_interface" );
    ros::NodeHandle Node_Handle;
    ROS_INFO ( "initializing ROS succeed" );
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
                // ros::console::g_level_lookup[ros::console::levels::Debug]);
    ros::Duration(2.0).sleep();

    // 1. init the first level loop
    bool    reset_flag      = false;
    bool    first_home_flag = true;
    while( ros::ok() )
    {
        ROS_INFO ( "mian loop started" );
        //
        // 1>>1. set the loop time, device_port, baund_rate and timeout
        const double        p_LOOP_TIME     = NUCLEAR_STATE_LOOP;

        const std::string   p_DEV_PORT      = "/dev/ttyUSB2";
        const int           p_BAUD_RATE     = 115200;
        const int           p_TIMEOUT       = 1000;
        ROS_INFO ( "setting parameters succeed" );
        //
        // 1>>2. init the subscribers and publishers
        NuclearCommandHandller      NUCLEAR_COMMAND_HANDLLER;
        ros::Subscriber subNuclearCmd =
                Node_Handle.subscribe<mobot6_interface::NuclearControl>(
                    INTERFACE_COMMAND_NUCLEAR, 3, &NuclearCommandHandller::callback,
                    &NUCLEAR_COMMAND_HANDLLER);
        ros::Publisher  pubNuclearStates =
                Node_Handle.advertise<std_msgs::String>(
                    INTERFACE_STATES_NUCLEAR, 3, false);
        ros::Publisher  pubJointStates =
                Node_Handle.advertise<sensor_msgs::JointState>(
                    INTERFACE_JOINT_STATES_NUCLEAR, 3, false);
        ROS_INFO ( "initializing subscriber and publisher succeed" );
        //
        // 1>>3. try to connect to nuclear sensor in each 3 seconds
        serial::Serial  Serial_Handle;
        // set parameters of serial port
        try{
            Serial_Handle.setPort( p_DEV_PORT );
            Serial_Handle.setBaudrate( p_BAUD_RATE );
            serial::Timeout tiemout = serial::Timeout::simpleTimeout( p_TIMEOUT );
            Serial_Handle.setTimeout( tiemout );
            Serial_Handle.open();
        }
        catch(serial::IOException& e){
            ROS_ERROR("Unable to open port.");
        }
        // check wether serial_port has been connected, otherwise re-connecting
        while( !Serial_Handle.isOpen() )
        {
            ROS_INFO ( "Failed to connect to serial_port.");
            for ( size_t i = 2; i > 0; --i )
            {
                ROS_INFO ( "Retry in %lu seconds.", i );
                ros::Duration(1).sleep();
            }
            //
            try{ Serial_Handle.open(); }
            catch(serial::IOException& e) { ROS_ERROR("Unable to open port."); }
            ros::Duration(0.5).sleep();
        }
        ROS_INFO ( "Connecting to Serial Port successfully." );
        //
        // 1>>4. the reset process
        if (reset_flag)
        {
            if ( Serial_Handle.available() )
            {
                std::string clear_string_ = Serial_Handle.read(Serial_Handle.available());
                ROS_WARN_STREAM("[1>>4] clear serial buffer before receiving RESET response: " << clear_string_);
            }
            //
            std::string output_;
            int resp = CommandProcessor(STP_CMD_HOME, 0.0, 0.0, output_);
            if (resp != 0)
            {
                ROS_ERROR("[1>>4] can not convert the HMOE command: %d, shut down the power.", resp);

            }
            else if (resp == 0)
            {
                Serial_Handle.write(output_); // send the serial_port data
                ROS_INFO_STREAM("Serial Write: " << output_);
            }
        }
        //
        int iCnt = 0;
        while( ros::ok() && reset_flag)
        {
            if (Serial_Handle.available())
            {
                std::string read_ = Serial_Handle.read(Serial_Handle.available());
                ROS_INFO_STREAM("[1>>4] Reading from serial port: " << read_);

                std::string command_;
                int result_;
                double position_, param2_, param3_;
                int resp = CommandExtraction(read_, command_, result_, position_, param2_, param3_);
                if (resp != 0)
                    ROS_ERROR("[1>>4] can not extract the HOME string:[%d]", resp);

                if ((command_==CMD_STRING_HOME) && (result_==0))
                {
                    ROS_INFO("[1>>4] the HOME process has finished.");
                    reset_flag = false;
                }
                else
                {
                    if (command_ != CMD_STRING_HOME)
                        ROS_ERROR("[1>>4] don't match the HOME request.");
                    else if (result_ == 1)
                        ROS_ERROR("[1>>4] get error result, HOME process failed.");
                    ROS_ERROR("[1>>4] unknown errors, please reset the serial port.");
                }
                break;
            }
            else
            {
                ROS_INFO_STREAM("[1>>4] waiting to read response: " << iCnt);
                iCnt++;
            }
            //
            ros::Duration(0.5).sleep();
        }
        //
        // 1>>5. init the s_MODE variable
        int s_MODE = MODE_MOVE;
        if ( reset_flag )
            s_MODE = MODE_RESET;
        //
        // 1>>6. define several variables before starting the second-level loop
        int     s_WAITING_COUNT     = 0;
        bool    s_STATE_MOVING      = false;
        bool    s_HOME_READY        = true;

        bool    s_RESP_OVERTIME     = false;
        bool    s_RESP_MATCH        = true;
        double  s_JOINT_POSITION    = 0.0;
        double  s_GM_INTENSITY      = 0.0;
        double  s_NEUTRON_INTENSITY = 0.0;

        // bool    s_RESP_WAIT_FLAG    = false;
        // int     s_RESP_WAIT_CNT     = 0;
        std::string     strStatus;
        strStatus.reserve( 256 );

        //
        // 1>>7. waiting for the initial homing process for 2 minutes
        int counter = 0;
        while( (counter <= 3/3) && (first_home_flag))  // 90
        {
            ros::Duration(3).sleep();
            counter++;
            ROS_INFO_STREAM("waiting for the initial setup process: "<<counter);
        }
        first_home_flag = false;
        //------------------------ the second loop start --------------------------------
        // 2. initialize the second_level loop
        ROS_INFO ( "control loop started" );
        while( ros::ok() && s_MODE != MODE_RESET )
        {
            double start = ros::Time::now().toSec();
            ros::spinOnce();
            // 2>>1. re-check the serial port and switch the OpMode && MotionStatus
            double start_1 = ros::Time::now().toSec();
            if ( s_STATE_MOVING && Serial_Handle.available() )
            {
                // 2>>1>>1. check the response of serail port
                std::string read_ = Serial_Handle.read(Serial_Handle.available());
                ROS_INFO_STREAM("[2>>1>>1] Reading from serial port: " << read_);

                std::string command_;
                int result_;
                double param1_, param2_, param3_;
                int resp = CommandExtraction(read_, command_, result_, param1_, param2_, param3_);
                if (resp != 0)
                    ROS_ERROR("[2>>1>>1] can not extract the string:[%d]", resp);
                // 2>>1>>2. deal with the "MODE_HOME" condition
                if ((s_MODE==MODE_HOME) && (resp==0))
                {
                    if ((command_==CMD_STRING_HOME) && (result_==0))
                    {
                        ROS_INFO("[2>>1>>2] get the HOME response, switch to MODE_MOVE and STABLE.");
                        s_STATE_MOVING = false;
                        s_MODE = MODE_MOVE;
                        s_HOME_READY = true;
                    }
                    else
                    {
                        if (command_ != CMD_STRING_HOME)
                            ROS_ERROR("[2>>1>>2] don't match the HOME request.");
                        else if (result_ == 1)
                            ROS_ERROR("[2>>1>>2] get error result, obtaining HOME response failed.");
                        ROS_ERROR("[2>>1>>2] unknown errors, please reset the serial port.");
                    }
                }
                // 2>>1>>3. deal with the "MODE_MOVE" condition
                else if ((s_MODE==MODE_MOVE) && (resp==0))
                {
                    if ((command_==CMD_STRING_MOVE) && (result_==0))
                    {
                        ROS_INFO("[2>>1>>2] get the MOVE response, switch the STABLE.");
                        s_STATE_MOVING = false;
                    }
                    else
                    {
                        if (command_ != CMD_STRING_MOVE)
                            ROS_ERROR("[2>>1>>2] don't match the MOVE request.");
                        else if (result_ == 1)
                            ROS_ERROR("[2>>1>>2] get error result, obtaining MOVE response failed.");
                        ROS_ERROR("[2>>1>>2] unknown errors, please reset the serial port.");
                    }
                }
                // --------------------------- just for testing ----------------------------
                // 2>>1>>4. deal with the other unknown condition
                else if (resp == 0)
                {
                    ROS_ERROR_STREAM("some unknown condition in [2>>1]: " << command_);
                }
            }
            // 2>>1>>5. deal with the condition that no response in serial port
            else if ( s_STATE_MOVING && !Serial_Handle.available() )
            {
                s_WAITING_COUNT++;
                if (s_WAITING_COUNT >= (int)(120.0/NUCLEAR_STATE_LOOP))
                {
                    s_WAITING_COUNT = 0;
                    s_MODE = MODE_RESET;
                    reset_flag = true;
                    ROS_INFO("[2>>1>>5] waiting for home/move response overtime, abort and reset it.");
                }
                else
                {
                    if (s_WAITING_COUNT % 10 == 0)
                        ROS_INFO("[2>>1>>5] waiting for home/move response");
                }
            }
            double cost_1 = ros::Time::now().toSec() - start_1;
            ROS_DEBUG("1. MOVE_WAITING process end. cost_time: %lf ms.", cost_1*1000);
            //
            // 2>>2. get the command from "/nuclear_command" and determine written string
            double start_2 = ros::Time::now().toSec();
            if ((NUCLEAR_COMMAND_HANDLLER.newMsg()) && (s_MODE!=MODE_RESET))
            {
                mobot6_interface::NuclearControl  input_ = NUCLEAR_COMMAND_HANDLLER.fetchMsg();
                std::string command_ = input_.commandID;
                // clear the current serial port
                if ((command_!=STP_CMD_OTHER) && (!s_STATE_MOVING))
                {
                    ROS_INFO("[2>>2] get interface command from '/nuclear_command'");
                    while( Serial_Handle.available() )
                    {
                        std::string clear_string_ = Serial_Handle.read(Serial_Handle.available());
                        ROS_WARN_STREAM("[2>>2] clear serial buffer before requesting command: " << clear_string_);
                    }
                }
                // 2>>2>>1. the condition that RESET is requested
                if ( command_ == STP_CMD_RESET )
                {
                    s_MODE = MODE_RESET;
                    reset_flag = true;
                }
                // 2>>2>>2. the condition with MODE_MOVE && STABLE_STATE
                else if ((s_MODE==MODE_MOVE) && (!s_STATE_MOVING))
                {
                    if (command_ == STP_CMD_HOME)
                    {
                        std::string output_;
                        int resp = CommandProcessor(STP_CMD_HOME, 0.0, 0.0, output_);
                        if (resp != 0)
                        {
                            ROS_ERROR("[2>>2>>2] can not convert the HMOE command: %d", resp);
                        }
                        else if (resp == 0)
                        {
                            Serial_Handle.write(output_); // send the serial_port data
                            ROS_INFO_STREAM("Serial Write: " << output_);
                            //
                            s_STATE_MOVING = true;
                            s_WAITING_COUNT = 0;
                            s_MODE = MODE_HOME;
                            s_HOME_READY = false;
                        }
                    }
                    else if (command_ == STP_CMD_MOVE)
                    {
                        std::string output_;
                        double  position_ = input_.data[0];
                        double  speed_ = input_.data[1];
                        int resp = CommandProcessor(STP_CMD_MOVE, position_, speed_, output_);
                        if (resp != 0)
                        {
                            ROS_ERROR("[2>>2>>2] can not convert the MOVE command: %d", resp);
                        }
                        else if (resp == 0)
                        {
                            Serial_Handle.write(output_); // send the serial_port data
                            ROS_INFO_STREAM("[2>>2>>2] Serial Write: " << output_);
                            //
                            s_STATE_MOVING = true;
                            s_WAITING_COUNT = 0;
                        }
                    }
                }
                // 2>>2>>3. the condition with MODE_MOVE && MOVING_STATE
                else if ((s_MODE==MODE_MOVE) && (s_STATE_MOVING))
                {
                    if (command_ == STP_CMD_STOP)
                    {
                        std::string output_;
                        int resp = CommandProcessor(STP_CMD_STOP, 0.0, 0.0, output_);
                        if (resp != 0)
                        {
                            ROS_ERROR("[2>>2>>3] can not convert STOP command in MODE_MOVE: %d", resp);
                        }
                        else if (resp == 0)
                        {
                            Serial_Handle.write(output_); // send the serial_port data
                            ROS_INFO_STREAM("[2>>2>>3] Serial Write: " << output_);
                            ros::Duration(0.1).sleep();

                            int iCnt = 0;
                            while(iCnt <= 50)
                            {
                                if (Serial_Handle.available())
                                {
                                    std::string read_ = Serial_Handle.read(Serial_Handle.available());
                                    ROS_INFO_STREAM("[2>>2>>3] Reading from serial port: " << read_);

                                    std::string command_;
                                    int result_;
                                    double position_, param2_, param3_;
                                    int resp = CommandExtraction(read_, command_, result_, position_, param2_, param3_);
                                    if (resp != 0)
                                        ROS_ERROR("[2>>2>>32] can not extract the STOP string:[%d]", resp);

                                    if ((command_==CMD_STRING_STOP) && (result_==0))
                                    {
                                        ROS_INFO("[2>>2>>3] stop the MOVE motion successfully.");
                                        s_STATE_MOVING = false;
                                    }
                                    else
                                    {
                                        if (command_ != CMD_STRING_STOP)
                                            ROS_ERROR("[2>>2>>3] don't match the STOP request.");
                                        else if (result_ == 1)
                                            ROS_ERROR("[2>>2>>3] get error result, stop MOVE motion failed.");
                                        ROS_ERROR("[2>>2>>32] unknown errors, please reset the serial port.");
                                    }
                                    break;
                                }
                                else
                                {
                                    ROS_INFO_STREAM("[2>>2>>3] waiting to read response: " << iCnt);
                                    iCnt++;
                                }
                                //
                                ros::Duration(0.05).sleep();
                            }
                            // wait for STOP response overtime
                            if (iCnt > 50)
                                ROS_ERROR("[2>>2>>3] waiting for STOP response overtime, reset the serial port.");
                        }
                    }
                }
                // 2>>2>>4. the condition with MODE_HOME && STABLE_STATE
                else if ((s_MODE==MODE_HOME) && (!s_STATE_MOVING))
                {
                    ROS_ERROR("[2>>2>>4] this is actually a faulty condition, check the logic diagram.");
                }
                // 2>>2>>5. the condition with MODE_HOME && MOVING_STATE
                else if ((s_MODE==MODE_HOME) && (s_STATE_MOVING))
                {
                    if (command_ == STP_CMD_STOP)
                    {
                        std::string output_;
                        int resp = CommandProcessor(STP_CMD_STOP, 0.0, 0.0, output_);
                        if (resp != 0)
                        {
                            ROS_ERROR("[2>>2>>5] can not convert STOP command in MODE_HOME: %d", resp);
                        }
                        else if (resp == 0)
                        {
                            Serial_Handle.write(output_); // send the serial_port data
                            ROS_INFO_STREAM("[2>>2>>5] Serial Write: " << output_);
                            ros::Duration(0.1).sleep();

                            int iCnt = 0;
                            while(iCnt <= 50)
                            {
                                if (Serial_Handle.available())
                                {
                                    std::string read_ = Serial_Handle.read(Serial_Handle.available());
                                    ROS_INFO_STREAM("[2>>2>>5] Reading from serial port: " << read_);

                                    std::string command_;
                                    int result_;
                                    double position_, param2_, param3_;
                                    int resp = CommandExtraction(read_, command_, result_, position_, param2_, param3_);
                                    if (resp != 0)
                                        ROS_ERROR("[2>>2>>5] can not extract the STOP string:[%d]", resp);

                                    if ((command_==CMD_STRING_STOP) && (result_==0))
                                    {
                                        ROS_INFO("[2>>2>>5] stop the HOME motion successfully.");
                                        s_STATE_MOVING = false;
                                        s_MODE = MODE_MOVE;
                                    }
                                    else
                                    {
                                        if (command_ != CMD_STRING_STOP)
                                            ROS_ERROR("[2>>2>>5] don't match the STOP request.");
                                        else if (result_ == 1)
                                            ROS_ERROR("[2>>2>>5] get error result, stop MOVE motion failed.");
                                        ROS_ERROR("[2>>2>>5] unknown errors, please reset the serial port.");
                                    }
                                    break;
                                }
                                else
                                {
                                    ROS_INFO_STREAM("[2>>2>>5] waiting to read STOP response: " << iCnt);
                                    iCnt++;
                                }
                                //
                                ros::Duration(0.05).sleep();
                            }
                            // wait for STOP response overtime
                            if (iCnt > 50)
                                ROS_ERROR("[2>>2>>5] waiting for STOP response overtime, reset the serial port.");
                        }
                    }
                }
            }
            double cost_2 = ros::Time::now().toSec() - start_2;
            ROS_DEBUG("2. NUCLEAR_COMMAND_HANDLLER end. cost_time: %lf ms.", cost_2*1000);
            //
            // 2>>3. obtain the current joint position and GM value
            double start_3 = ros::Time::now().toSec();
            s_RESP_OVERTIME = false;
            s_RESP_MATCH = true;
            // 2>>3>>1. determine the stable status and request position && intensity
            if (!s_STATE_MOVING && s_HOME_READY)
            {
                std::string output_;
                int resp = CommandProcessor(STP_CMD_REQ, 0.0, 0.0, output_);
                if (resp != 0)
                {
                    ROS_ERROR("[2>>3] can not convert STOP command in MODE_MOVE: %d", resp);
                }
                else if (resp == 0)
                {
                    if (Serial_Handle.available())
                    {
                        std::string clear_string_ = Serial_Handle.read(Serial_Handle.available());
                        ROS_WARN_STREAM("[2>>3>>1] clear serial buffer before requesting: " << clear_string_);
                    }
                    Serial_Handle.write(output_); // send the serial_port data
                    ROS_INFO_STREAM("[2>>3>>1] Serial Write: " << output_);
                    ros::Duration(0.1).sleep();

                    int iCnt = 0;
                    while(iCnt <= 50)
                    {
                        if (Serial_Handle.available())
                        {
                            std::string read_ = Serial_Handle.read(Serial_Handle.available());
                            ROS_INFO_STREAM("[2>>3] Reading from serial port: " << read_);

                            std::string command_;
                            int result_;
                            double position_, gm_value_, neutron_;
                            int resp = CommandExtraction(read_, command_, result_, position_, gm_value_, neutron_);
                            if (resp != 0)
                                //ROS_ERROR("[2>>3] can not extract the REQ string:[%d]", resp);20190329

                            if ((command_==CMD_STRING_REQ) && (result_==0))
                            {
                                ROS_INFO("[2>>3] request REQ command successfully.");
                                s_JOINT_POSITION = position_;
                                s_GM_INTENSITY = gm_value_;
                                s_NEUTRON_INTENSITY = neutron_;
                            }
                            else
                            {
                                if (command_ != CMD_STRING_REQ)
				    int aa = 1;
                                    //ROS_ERROR("[2>>3] don't match the REQ request.");//20190329
                                else if (result_ == 1)
                                    ROS_ERROR("[2>>3] get error result, REQ sensor failed. [res: %d]", result_);
                                s_RESP_MATCH = false;
                            }
                            break;
                        }
                        else
                        {
                            ROS_INFO_STREAM("[2>>3] waiting to read REQ response: " << iCnt);
                            iCnt++;
                        }
                        //
                        ros::Duration(0.05).sleep();
                    }
                    // wait for REQ response overtime
                    if (iCnt > 50)
                    {
                        s_RESP_OVERTIME = true;
                        ROS_ERROR("[2>>3] waiting for REQ response overtime, retry it.");
                    }
                }
            }
            // 2>>3>>2. deal with other status and fill in measurements
            if (s_STATE_MOVING || s_RESP_OVERTIME || !s_RESP_MATCH)
            {
                s_JOINT_POSITION = -999.0;
                s_GM_INTENSITY = -999.0;
                s_NEUTRON_INTENSITY = -999.0;
            }
            double cost_3 = ros::Time::now().toSec() - start_3;
            ROS_DEBUG("3. STATE_REQUEST end. cost_time: %lf ms.", cost_3*1000);
            //
            // 2>>4. send status to "/interface_states_nuclear"
            double start_4 = ros::Time::now().toSec();
            strStatus.clear();
            // 2>>4>>1. determine the Operation Mode
            if ( s_MODE == MODE_MOVE )
                StringAppend( strStatus, INTERFACE_MODE_MOVE INTERFACE_SPARATER );
            else if ( s_MODE == MODE_HOME )
                StringAppend( strStatus, INTERFACE_MODE_HOME INTERFACE_SPARATER );
            else if ( s_MODE == MODE_RESET )
                StringAppend( strStatus, INTERFACE_MODE_RESET INTERFACE_SPARATER );
            // 2>>4>>2. determine the moving/stable status
            if ( s_STATE_MOVING )
                StringAppend( strStatus, INTERFACE_STATE_MOVING INTERFACE_SPARATER );
            else
                StringAppend( strStatus, INTERFACE_STATE_STABLE INTERFACE_SPARATER );
            // 2>>4>>3. determine the request state and response data
            if ( s_RESP_MATCH && !s_RESP_OVERTIME && s_HOME_READY)
            {
                StringAppend( strStatus, INTERFACE_RESP_NORMAL INTERFACE_SPARATER );
                StringAppend( strStatus, "%s\t", std::to_string(s_JOINT_POSITION).c_str() );
                StringAppend( strStatus, "%s\t", std::to_string(s_GM_INTENSITY).c_str() );
                StringAppend( strStatus, "%s", std::to_string(s_NEUTRON_INTENSITY).c_str() );
            }
            else if ( !s_RESP_MATCH )
            {
                StringAppend( strStatus, INTERFACE_RESP_UNMATCH INTERFACE_SPARATER );
                StringAppend( strStatus, INTERFACE_NOTHING INTERFACE_SPARATER );
                StringAppend( strStatus, INTERFACE_NOTHING INTERFACE_SPARATER );
                StringAppend( strStatus, INTERFACE_NOTHING );
            }
            else if ( s_RESP_OVERTIME )
            {
                StringAppend( strStatus, INTERFACE_RESP_OVERTIME INTERFACE_SPARATER );
                StringAppend( strStatus, INTERFACE_NOTHING INTERFACE_SPARATER );
                StringAppend( strStatus, INTERFACE_NOTHING INTERFACE_SPARATER );
                StringAppend( strStatus, INTERFACE_NOTHING );
            }
            else if ( !s_HOME_READY )
            {
                StringAppend( strStatus, INTERFACE_RESP_UNREADY INTERFACE_SPARATER );
                StringAppend( strStatus, INTERFACE_NOTHING INTERFACE_SPARATER );
                StringAppend( strStatus, INTERFACE_NOTHING INTERFACE_SPARATER );
                StringAppend( strStatus, INTERFACE_NOTHING );
            }
            // 2>>4>>4. send status to "/interface_states_nuclear"
            std_msgs::String    status_output;
            status_output.data = strStatus;
            pubNuclearStates.publish( status_output );
            //
            // 2>>5. send status to "/joint_states_nuclear"
            std::vector<std::string> joint_names( JOINT_NUCLEAR_NUM );
            std::vector<double> feedback( JOINT_NUCLEAR_NUM );
            for (size_t i = 0; i < JOINT_NUCLEAR_NUM; ++i)
                joint_names[i] = SENSOR_JOINT[i];
            feedback[0] = s_JOINT_POSITION;
            feedback[1] = s_GM_INTENSITY;
            feedback[2] = s_NEUTRON_INTENSITY;
            //
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.name = joint_names;
            joint_state.position = feedback;
            pubJointStates.publish( joint_state );
            //
            double cost_4 = ros::Time::now().toSec() - start_4;
            ROS_DEBUG("4. PUBLISHE_PROCESS end. cost_time: %lf ms.", cost_4*1000);
            //
            // 2>>6. control the loop time and check the time over conditions
            double sleep_time = p_LOOP_TIME - ( ros::Time::now().toSec() - start );
            if ( sleep_time > 0 )
            {
                ros::Duration( sleep_time ).sleep();
                ROS_DEBUG("5. Total Cost Time: %lf ms", (p_LOOP_TIME-sleep_time)*1000);
            }
            else
                ROS_WARN ( "control loop over time: %f s", -sleep_time );
        }
        ROS_INFO("second-level control-loop ended");
        //------------------------ the second loop end --------------------------------

        // stop the motion if motor is moving
        if (s_STATE_MOVING)
        {
            Serial_Handle.write( STP_STRING_STOP ); // send the serial_port data
            ROS_INFO_STREAM("Serial Write: " << STP_STRING_STOP);
            ros::Duration(0.1).sleep();
            //
            int iCnt = 0;
            while(iCnt <= 50)
            {
                if (Serial_Handle.available())
                {
                    std::string read_ = Serial_Handle.read(Serial_Handle.available());
                    ROS_INFO_STREAM("Reading from serial port: " << read_);

                    std::string command_;
                    int result_;
                    double position_, param2_, param3_;
                    int resp = CommandExtraction(read_, command_, result_, position_, param2_, param3_);
                    if (resp != 0)
                        ROS_ERROR("[3] can not extract the STOP string:[%d]", resp);

                    if ((command_==CMD_STRING_STOP) && (result_==0))
                    {
                        ROS_INFO("[2>>2>>4] stop the HOME motion successfully.");
                        s_STATE_MOVING = false;
                    }
                    else
                    {
                        if (command_ != CMD_STRING_STOP)
                            ROS_ERROR("[3] don't match the STOP request.");
                        else if (result_ == 1)
                            ROS_ERROR("[3] get error result, stop MOVE motion failed.");
                        ROS_ERROR("[3] unknown errors, please reset the serial port.");
                    }
                    break;
                }
                else
                {
                    ROS_INFO_STREAM("[3] waiting to read STOP response: " << iCnt);
                    iCnt++;
                }
                ros::Duration(0.05).sleep();
            }
            //
            if (iCnt > 50)
                ROS_ERROR("[3] waiting for STOP response overtime, reset the serial port.");
        }
        // suppose the sensor has been stopped, close the serial port
        if (!s_STATE_MOVING)
        {
            Serial_Handle.close();
            ROS_INFO("[3] has closed the serial port, restart sensor after 1 seconds.");
            ros::Duration(1).sleep();
        }
        else
        {
            while(ros::ok())
            {
                ROS_ERROR("[3] can't stop the motor, close the power and restart it.");
                ros::Duration(0.5).sleep();
            }
        }
        //
        subNuclearCmd.shutdown();
        pubNuclearStates.shutdown();
        pubJointStates.shutdown();
        ROS_INFO( "shutting down subscriber and publisher succeed" );
    }
    // main loop end
    ros::waitForShutdown();
    return 0;
}












