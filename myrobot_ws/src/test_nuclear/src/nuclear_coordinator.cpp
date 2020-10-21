#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include "mobot6_interface/NuclearControl.h"
#include "mobot6_interface/Intensity.h"
#include "mobot6_interface/PointIntensity.h"

#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <numeric>
#include <boost/algorithm/string.hpp>
#include <log4cxx/logger.h>

#include <vector>

//--------------------------------------------------------------------------------

// 1. define the corresponding topics
#define     NUCLEAR_COMMAND             "/nuclear_command"              // keyboard pub, coord sub
#define     INTERFACE_STATES_NUCLEAR    "/interface_states_nuclear"     // inter pub, coord sub
#define     INTERFACE_COMMAND_NUCLEAR   "/interface_command_nuclear"    // coord pub, inter sub
#define     UNCLEAR_INTENSITY           "/nuclear_intensity"
#define     UNCLEAR_POINT1_INTENSITY    "/nuclear_intensity_point1"
#define     UNCLEAR_POINT2_INTENSITY    "/nuclear_intensity_point2"
#define     VEHICLE_ODOMETRY            "/odom"				// coord sub, from "/vehicle_odometry"

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

// 3. define the corresponding command from pre-process
#define     STP_CMD_MOVE        "STEPMOVE"
#define     STP_CMD_HOME        "STEPHOME"
#define     STP_CMD_STOP        "STEPSTOP"
#define     STP_CMD_RESET       "STEPRESET"
#define     STP_CMD_AUTO        "STEPAUTO"
#define     STP_CMD_AUTOSTOP    "STEPAUTOSTOP"
#define     STP_CMD_DETECT      "STEPDETECT"
#define     STP_CMD_SAVEFILE    "STEPSAVE"
#define     STP_CMD_REQ         "STEPREQ"   // inertial command
#define     STP_CMD_OTHER       "STEPOTHER" // inertial command

// 4. define the dof and loop_time
#define     NUCLEAR_DOF         2  // just for NuclearControl message
#define     AVER_SAMPLE_NUM     3
#define     AVER_DETECT_NUM     8
#define     PRE_MAX_ROTATION    13
#define     POINT_DETECT_NUM    2

#define     NUCLEAR_COORD_LOOP  0.300

// 5. define the file_saving path
#define     AUTO_SAMPLE_PATH        "/home/hit/SampleLog.txt"
#define     AUTO_DETECT_PATH        "/home/hit/DetectLog.txt"
//#define     PUB_DETECT_PATH         "/home/hit/PubDetectLog.txt"
#define     STREAM_TEST_SEGMENT     "########################################"
#define     STREAM_SEQ_SEGMENT      "----------------------------------------"


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


//#################################################
//###  3. define the class NavOdometryHandller  ###
//#################################################
class NavOdometryHandller
{
public:
    void callback(const nav_msgs::Odometry::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    NavOdometryHandller()
        : mCrtMsg()
        , mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const nav_msgs::Odometry & fetchMsg()
    {
        mNewMsg = false;
        //ROS_WARN("navigation command - fetchMsg successfully???");
        return mCrtMsg;
    }

protected:
    nav_msgs::Odometry mCrtMsg;
    bool mNewMsg;
};


//#########################################
//###  4. define the Function quat2yaw  ###
//#########################################
double quat2yaw(const geometry_msgs::Quaternion msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    return yaw;
}


//#####################################
//###  5. define the Main Function  ###
//#####################################
int main(int argc, char **argv)
{
    ros::init(argc, argv, "nuclear_coordinator");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle     Node_Handle;
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
                // ros::console::g_level_lookup[ros::console::levels::Debug]);

    // 1>>1. init the topic-subscribed handllers
    StringHandllerOnce          INTERFACE_STATE_HANDLLER;
    NuclearCommandHandller      NUCLEAR_COMMAND_HANDLLER;
    NavOdometryHandller         NAV_ODOMETRY_HANDLLER;

    std::ofstream       write_ofs;
    const double        p_LOOP_TIME     = NUCLEAR_COORD_LOOP;
    bool                new_nav_flag    = false;
    nav_msgs::Odometry  nav_odometry;

    // 1>>2. init the subscribers and publishers
    ros::Subscriber subNuclearCmd =
            Node_Handle.subscribe<mobot6_interface::NuclearControl>(
                NUCLEAR_COMMAND, 3, &NuclearCommandHandller::callback,
                &NUCLEAR_COMMAND_HANDLLER);
    ros::Subscriber subInterfaceStates =
            Node_Handle.subscribe<std_msgs::String>(
                INTERFACE_STATES_NUCLEAR, 3, &StringHandllerOnce::callback,
                &INTERFACE_STATE_HANDLLER);
    ros::Subscriber subVehicleOdom =
            Node_Handle.subscribe<nav_msgs::Odometry>(
                VEHICLE_ODOMETRY, 3, &NavOdometryHandller::callback,
                &NAV_ODOMETRY_HANDLLER);

    ros::Publisher  pubNuclearCmd =
            Node_Handle.advertise<mobot6_interface::NuclearControl>(
                INTERFACE_COMMAND_NUCLEAR, 3, false);
    ros::Publisher  pubIntensityState =
            Node_Handle.advertise<mobot6_interface::Intensity>(
                UNCLEAR_INTENSITY, 3, false);
    ros::Publisher  pubIntensityPoint1 =
            Node_Handle.advertise<mobot6_interface::PointIntensity>(
                UNCLEAR_POINT1_INTENSITY, 3, false);
    ros::Publisher  pubIntensityPoint2 =
            Node_Handle.advertise<mobot6_interface::PointIntensity>(
                UNCLEAR_POINT2_INTENSITY, 3, false);

    // 1>>3. define and init some parameters
    std::string     NuclearMode;
    std::string     NuclearState;
    std::string     RespState;

    bool            interface_motion    = false;  // about operation mode
    bool            interface_stable    = false;   // about sensor axis status
    double          joint_rotation      = 0.0;
    double          gm_intensity        = 0.0;
    double          neutron_intensity   = 0.0;

    // 1>>4. define the variables in auto_sample process
    bool            s_AUTO_SAMPLE       = false;
    bool            s_SAMPLE_BGN        = false;
    bool            s_AUTOMOVE_READY    = false;
    bool            s_AUTOMOVE_WAIT     = false;
    bool            s_AUTOSTOP_FLAG     = false;

    int             s_sample_seq        = 1;
    int             s_sample_idx        = 0;
    int             s_buffer_idx        = 0;
    int             s_sample_num        = 0;
    double          s_sample_interval   = 0.0;    //

    std::vector<double>  s_intensity_buffer(AVER_SAMPLE_NUM, 0.0);
    std::vector<double>  s_sample_rotation(PRE_MAX_ROTATION, 0.0);
    std::vector<double>  s_sample_intensity(PRE_MAX_ROTATION, 0.0);

    bool                 reserve_flag           = false;
    int                  reserve_seq            = 0;
    double               reserve_rotation_max   = 0.0;
    double               reserve_intensity_max  = 0.0;
    std::vector<double>  reserve_rotation;
    std::vector<double>  reserve_intensity;

    // 1>>5. define the variables in auto_detect process
    bool            s_DETECT_SAMPLE     = false;
    bool            s_DETECT_BGN        = false;
    bool            s_DETMOVE_READY     = false;
    bool            s_DETMOVE_WAIT      = false;
    bool            s_DETECT_UPDATE     = false;

    int             s_detect1_seq       = 1;
    int             s_detect2_seq       = 1;
    int             s_detect_idx        = 0;
    int             s_detbuf_idx        = 0;
    int             s_detupdate_num     = 0;
    double          s_detect_rot        = 0.0;

    std::vector<double>  s_detect_buffer(POINT_DETECT_NUM, 0.0);
    std::vector<double>  s_detect_rotation(POINT_DETECT_NUM, 0.0);
    std::vector<double>  s_detect_intensity(POINT_DETECT_NUM, 0.0);

    bool                 res_det1_flag          = false;
    int                  res_det1_seq           = 0;
    std::vector<double>  res_det1_rotation;
    std::vector<double>  res_det1_intensity;

    bool                 res_det2_flag          = false;
    int                  res_det2_seq           = 0;
    std::vector<double>  res_det2_rotation;
    std::vector<double>  res_det2_intensity;
    nav_msgs::Odometry   odom_auto_;
    nav_msgs::Odometry   odom_det1_;
    nav_msgs::Odometry   odom_det2_;
    // 1>>6. waiting for interface states to ensure sensor initial homing has ended
    while(ros::ok())
    {
        ros::spinOnce();
        if ( INTERFACE_STATE_HANDLLER.newMsg() )
        {
            std_msgs::String input_ = INTERFACE_STATE_HANDLLER.fetchMsg();
            std::string input = input_.data;
            std::vector<std::string> result_;
            boost::split( result_, input, boost::is_any_of(" \t"),
                          boost::algorithm::token_compress_on );

            NuclearMode = result_[0];
            NuclearState = result_[1];
            RespState = result_[2];

            // check interface status to determine wether init has ended
            if ((NuclearMode == INTERFACE_MODE_MOVE) &&
                    (NuclearState == INTERFACE_STATE_STABLE) &&
                    (RespState == INTERFACE_RESP_NORMAL))
            {
                joint_rotation = atof(result_[3].c_str());
                gm_intensity = atof(result_[4].c_str());
                neutron_intensity = atof(result_[5].c_str());

                interface_motion = true;
                interface_stable = true;
                break;
            }
        }
        ros::Duration(0.5).sleep();
        ROS_INFO("waiting for nuclear interface states update.");//20190329
    }

    // 1>>7. write the started time into the LogFile
    time_t  time_seconds = time(0);
    struct tm*  now_time = localtime(&time_seconds);
    write_ofs.open( AUTO_SAMPLE_PATH, std::ofstream::out | std::ofstream::app );
    write_ofs << std::endl << std::endl << STREAM_TEST_SEGMENT << std::endl;
    write_ofs << "AutoSampling started:\t" << now_time->tm_hour << ":"
              << now_time->tm_min << ":"
              << now_time->tm_sec << std::endl;
    write_ofs.close();
    //
    write_ofs.open( AUTO_DETECT_PATH, std::ofstream::out | std::ofstream::app );
    write_ofs << std::endl << std::endl << STREAM_TEST_SEGMENT << std::endl;
    write_ofs << "AutoDetection started:\t" << now_time->tm_hour << ":"
              << now_time->tm_min << ":"
              << now_time->tm_sec << std::endl;
    write_ofs.close();

    // 2. start the main loop
    ROS_INFO("main loop start");
    while( ros::ok() )
    {
        double start = ros::Time::now().toSec();
        ros::spinOnce();
        //
        // 2>>1. get nuclear interface states from "/interface_states_nuclear"
        double start_1 = ros::Time::now().toSec();
        bool new_sample_status_ = false; // just for autostop check
        bool new_detect_status_ = false; // just for detectstop check
        interface_motion = false;
        interface_stable = false;
        // 2>>1>>1. obtain interface_states and detremine variable states
        if ( INTERFACE_STATE_HANDLLER.newMsg() )
        {
            ROS_DEBUG("[part_1] get the interface states.");
            std_msgs::String  input_ = INTERFACE_STATE_HANDLLER.fetchMsg();
            std::string  input = input_.data;
            std::vector<std::string> result_;

            boost::split( result_, input, boost::is_any_of(" \t"),
                          boost::algorithm::token_compress_on );
            NuclearMode  = result_[0];
            NuclearState = result_[1];
            RespState    = result_[2];

            if (NuclearMode == INTERFACE_MODE_MOVE)
                interface_motion = true;
            if (NuclearState == INTERFACE_STATE_STABLE)
                interface_stable = true;
            if ((RespState == INTERFACE_RESP_NORMAL) && interface_motion && interface_stable)
            {
                joint_rotation = atof(result_[3].c_str());
                gm_intensity = atof(result_[4].c_str());
                neutron_intensity = atof(result_[5].c_str());
                std::cout<<"[2>>1>>1]"<<joint_rotation<<" "<<gm_intensity<<" "<<neutron_intensity<<std::endl;
            }
            // just for autostop process for [2>>2>>1]
            if (s_AUTO_SAMPLE && s_AUTOSTOP_FLAG)
                new_sample_status_ = true;
            // just for detectstop process for [2>>3>>1]
            if (s_DETECT_SAMPLE && s_AUTOSTOP_FLAG)
                new_detect_status_ = true;
        }
        // 2>>1>>2. deal with the pre_sampling determination
        if ((s_AUTO_SAMPLE) && (!s_SAMPLE_BGN) & (!s_AUTOMOVE_READY))
        {
            if ((NuclearState==INTERFACE_STATE_MOVING) && (!s_AUTOMOVE_WAIT))
            {
                s_AUTOMOVE_WAIT = true;
                ROS_WARN("[2>>1>>2] identify that zero_process has started.");
            }
            else if ((fabs(joint_rotation)<=0.1) && (s_AUTOMOVE_WAIT))
            {
                s_AUTOMOVE_READY = true;
                ROS_WARN("[2>>1>>2] identify that zero_process has finished.");
            }
            //else
                //ROS_ERROR("[2>>1>>2] un-handled condition: MODE[%s], HOME_WAIT[%d], HOME_READY[%d].",
                          //aaaaaNuclearMode.c_str(), s_AUTOMOVE_WAIT, s_AUTOMOVE_READY);
        }
        // 2>>1>>3. deal with the pre_detection determination
        if ((s_DETECT_SAMPLE) && (!s_DETECT_BGN) & (!s_DETMOVE_READY))
        {
            if ((NuclearState==INTERFACE_STATE_MOVING) && (!s_DETMOVE_WAIT))
            {
                s_DETMOVE_WAIT = true;
                ROS_WARN("[2>>1>>3] identify that pre_motion has started.");
            }
            else if ((fabs(joint_rotation-s_detect_rot)<=0.1) && (s_DETMOVE_WAIT))
            {
                s_DETMOVE_READY = true;
                ROS_WARN("[2>>1>>3] identify that pre_motion has finished.");
            }
            //else
                //ROS_ERROR("[2>>1] un-handled condition: MODE[%s], DETECT_WAIT[%d], DETECT_READY[%d].",
                          //NuclearMode.c_str(), s_DETMOVE_WAIT, s_DETMOVE_READY);
        }
        double cost_1 = ros::Time::now().toSec() - start_1;
        ROS_DEBUG("1. INTERFACE_STATE_HANDLLER end. cost_time: %lf ms.", cost_1*1000);
        //
        // 2>>2. deal with the auto_sampling process
        double start_2 = ros::Time::now().toSec();
        if ( s_AUTO_SAMPLE )
        {
            ROS_DEBUG("[part_2] conduct the auto_sampling process.");
            ROS_INFO("[2>>2] AUTO_SAMPLE[%d], buffer_idx[%d], sample_idx[%d], sample_seq[%d], det1_seq[%d], det2_seq[%d]",
                     s_AUTO_SAMPLE, s_buffer_idx, s_sample_idx, reserve_seq, res_det1_seq, res_det2_seq);
            // 2>>2>>1. deal with the condition that sampling has started
            if ( s_SAMPLE_BGN && !s_AUTOSTOP_FLAG)
            {
                if (interface_motion && interface_stable && (s_buffer_idx<AVER_SAMPLE_NUM))
                {
                    if (s_buffer_idx == 0)
                    {
                        s_intensity_buffer[s_buffer_idx] = gm_intensity;
                        s_buffer_idx++;
                    }
                    else if (s_buffer_idx < AVER_SAMPLE_NUM)
                    {
                        if (gm_intensity != s_intensity_buffer[s_buffer_idx-1])
                        {
                            s_intensity_buffer[s_buffer_idx] = gm_intensity;
                            s_buffer_idx++;
                        }
                    }
                    ROS_INFO("[2>>2>>1] intensity_buffer[%d]: %f.", s_buffer_idx-1, s_intensity_buffer[s_buffer_idx-1]);
                }
                else if (interface_motion && interface_stable && (s_buffer_idx==AVER_SAMPLE_NUM))
                {
                    double  temp_aver_ = 0.0;
                    for (size_t i=0; i<AVER_SAMPLE_NUM; ++i)
                        temp_aver_ = temp_aver_+s_intensity_buffer[i]/(double)(AVER_SAMPLE_NUM);
                    ROS_INFO("[2>>2>>1] calculate the %d-st intensity, size:[%d], aver:[%f].",
                             s_sample_idx+1, s_intensity_buffer.size(), temp_aver_);
                    //
                    // fill in the measurement vector
                    double  temp_rot_ = (double)s_sample_idx * s_sample_interval;
                    s_sample_rotation[s_sample_idx] = temp_rot_;
                    s_sample_intensity[s_sample_idx] = temp_aver_;
                    //
                    // determin wether publish STEPMOVE command or end the auto_sample process
                    if (s_sample_idx < s_sample_num-1) // idx starts form 0, while num starts from 1
                    {
                        // publish STEPMOVE command
                        mobot6_interface::NuclearControl  output_;
                        output_.commandID = STP_CMD_MOVE;
                        output_.data.resize( NUCLEAR_DOF );
                        output_.data[0] = temp_rot_ + s_sample_interval;
                        output_.data[1] = 10.0;
                        pubNuclearCmd.publish( output_ );
                        ROS_INFO("[2>>2>>1] publish STEPMOVE command in auto_sampling mode, destination[%f].", output_.data[0]);
                        // add the auto_sample index
                        s_sample_idx++;
                    }
                    else if (s_sample_idx == s_sample_num-1)
                    {
                        if (reserve_flag)
                        {
                            ROS_ERROR("[2>>2>>1] the previous measurements have not been saved, please save them firstly.");
                        }
                        else
                        {
                            reserve_flag = true;
                            reserve_seq = s_sample_seq;
                            s_sample_seq++;
                            //
                            reserve_rotation.clear();
                            reserve_intensity.clear();
                            reserve_rotation  = s_sample_rotation;
                            reserve_intensity = s_sample_intensity;
                            //
                            reserve_intensity_max = 0.0;
                            for (int i=0; i<s_sample_rotation.size(); ++i)
                            {
                                if (reserve_intensity[i] > reserve_intensity_max)
                                {
                                    reserve_intensity_max = reserve_intensity[i];
                                    reserve_rotation_max = reserve_rotation[i];
                                }
                            }
                        }
                        s_sample_idx = 0;
                        s_AUTO_SAMPLE = false;
                        ROS_INFO("[2>>2>>1] the auto_sample process has ended");
                    }
                    // handle "s_buffer_idx" and "s_intensity_buffer"
                    s_buffer_idx = 0;
                    s_intensity_buffer.clear();
                    s_intensity_buffer.resize(AVER_SAMPLE_NUM, 0.0);
                }
                else if (interface_motion && !interface_stable)
                {
                    ROS_INFO("[2>>2>>1] waiting for motion stable in auto_sample process.");
                }
                else
                {
                    ROS_ERROR("[2>>2>>1] unknown condition: MODE[%s], STATUS[%s].",
                              NuclearMode.c_str(), NuclearState.c_str());
                }
            }
            // 2>>2>>2. deal with the condition that pre_sampling process should be conducted
            else if ( !s_SAMPLE_BGN && !s_AUTOSTOP_FLAG )
            {
                if (!s_AUTOMOVE_WAIT && !s_AUTOMOVE_READY)
                {
                    mobot6_interface::NuclearControl  output_;
                    output_.commandID = STP_CMD_MOVE;
                    output_.data.resize(NUCLEAR_DOF, 0.0);
                    output_.data[0] = 0.0;
                    output_.data[1] = 10.0;
                    pubNuclearCmd.publish(output_);
                    ROS_INFO("[2>>2>>2] publish STEPMOVE command to 0.0 in auto_sampling mode.");
                }
                else if (s_AUTOMOVE_WAIT && !s_AUTOMOVE_READY)
                {
                    ROS_INFO("[2>>2>>2] waiting for the end of ZERO process.");
                }
                else if (s_AUTOMOVE_WAIT && s_AUTOMOVE_READY)
                {
                    s_SAMPLE_BGN = true;
                    s_sample_idx = 0;
                    s_buffer_idx = 0;
                    s_intensity_buffer.clear();
                    s_sample_rotation.clear();
                    s_sample_intensity.clear();
                    s_intensity_buffer.resize(AVER_SAMPLE_NUM, 0.0);
                    s_sample_rotation.resize(s_sample_num, 0.0);
                    s_sample_intensity.resize(s_sample_num, 0.0);
                }
                else
                    ROS_ERROR("[2>>2>>2] unknown condition occured.");
            }
        }
        // 2>>2>>3. deal with the condition that AUTOSTOP has been activated
        if ( s_AUTOSTOP_FLAG && new_sample_status_ )
        {
            // test wether the status has been stopped
            if ( interface_motion && interface_stable )
            {
                s_sample_idx = 0;
                s_AUTO_SAMPLE = false;
                s_AUTOSTOP_FLAG = false;
                ROS_WARN("[2>>2>>3] the auto_sample process has paused and aborted.");
            }
        }
        double cost_2 = ros::Time::now().toSec() - start_2;
        ROS_DEBUG("2. AUTO_SAMPLE process end. cost_time: %lf ms.", cost_2*1000);
        //
        // 2>>3. deal with the detect_sampling process
        double start_3 = ros::Time::now().toSec();
        if ( s_DETECT_SAMPLE )
        {
            ROS_DEBUG("[part_3] conduct the detect_sampling process.");
            ROS_INFO("[2>>3] DETECT_SAMPLE[%d], DETECT_BGN[%d], detbuf_idx[%d], detect_idx[%d], sample_num[%d]",
                     s_DETECT_SAMPLE, s_DETECT_BGN, s_detbuf_idx, s_detect_idx, s_detupdate_num);
            // 2>>3>>1. deal with the condition that detection has started
            if ( s_DETECT_BGN && !s_AUTOSTOP_FLAG)
            {
                if (interface_motion && interface_stable && (s_detbuf_idx<AVER_DETECT_NUM))
                {
                    if (s_detbuf_idx == 0)
                    {
                        s_detect_buffer[s_detbuf_idx] = gm_intensity;
                        s_detbuf_idx++;
                    }
                    else if (s_detbuf_idx < AVER_DETECT_NUM)
                    {
                        if (gm_intensity != s_detect_buffer[s_detbuf_idx-1])
                        {
                            s_detect_buffer[s_detbuf_idx] = gm_intensity;
                            s_detbuf_idx++;
                        }
                    }
                    ROS_INFO("[2>>3>>1] detect_buffer[%d]: %f.", s_detbuf_idx-1, s_detect_buffer[s_detbuf_idx-1]);
                }
                else if (interface_motion && interface_stable && (s_detbuf_idx==AVER_DETECT_NUM))
                {
                    double  temp_aver_ = 0.0;
                    for (size_t i=0; i<AVER_DETECT_NUM; ++i)
                        temp_aver_ = temp_aver_+s_detect_buffer[i]/(double)(AVER_DETECT_NUM);
                    ROS_INFO("[2>>3>>1] calculate the %d-st intensity, size:[%d], aver:[%f].",
                             s_detect_idx+1, s_detect_buffer.size(), temp_aver_);
                    for (size_t i=0; i<AVER_DETECT_NUM; ++i)
                        std::cout<<s_detect_buffer[i]<<std::endl;
                    //
                    // fill in the measurement vector
                    s_detect_rotation[s_detect_idx] = fmod(s_detect_rot+(double)(s_detect_idx)*180.0, 360.0);
                    s_detect_intensity[s_detect_idx] = temp_aver_;
                    //
                    // determin wether publish STEPMOVE command or end the auto_sample process
                    if (s_detect_idx < POINT_DETECT_NUM-1) // idx starts form 0, while num starts from 1
                    {
                        // publish STEPMOVE command
                        mobot6_interface::NuclearControl  output_;
                        output_.commandID = STP_CMD_MOVE;
                        output_.data.resize( NUCLEAR_DOF );
                        output_.data[0] = s_detect_rot + 180.0;
                        output_.data[1] = 10.0;
                        pubNuclearCmd.publish( output_ );
                        ROS_INFO("[2>>3>>1] publish STEPMOVE command in auto_detection mode, destination[%f].", output_.data[0]);
                        // add the auto_sample index
                        s_detect_idx++;
                    }
                    else if (s_detect_idx == POINT_DETECT_NUM-1)
                    {
                        if (s_detupdate_num == 1)
                        {
                            if (res_det1_flag)
                            {
                                ROS_FATAL("[2>>3>>1] please save the previous measurements[%d] firstly.", s_detupdate_num);
                            }
                            else
                            {
                                res_det1_flag = true; //true
                                res_det1_seq = s_detect1_seq;
                                s_detect1_seq++;
                                //
                                res_det1_rotation.clear();
                                res_det1_intensity.clear();
                                res_det1_rotation  = s_detect_rotation;
                                res_det1_intensity = s_detect_intensity;
                            }
                        }
                        //
                        else if (s_detupdate_num == 2)
                        {
                            if (res_det2_flag)
                            {
                                ROS_FATAL("[2>>3>>1] please save the previous measurements[%d] firstly.", s_detupdate_num);
                            }
                            else
                            {
                                res_det2_flag = true; // true
                                res_det2_seq = s_detect2_seq;
                                s_detect2_seq++;
                                //
                                res_det2_rotation.clear();
                                res_det2_intensity.clear();
                                res_det2_rotation  = s_detect_rotation;
                                res_det2_intensity = s_detect_intensity;
                            }
                        }
                        //
                        s_detect_idx = 0;
                        s_detupdate_num = 0;
                        s_DETECT_SAMPLE = false;
                        s_DETECT_BGN    = false;
                        s_DETMOVE_READY = false;
                        s_DETMOVE_WAIT  = false;
                        ROS_INFO("[2>>3>>1] the auto_detection process has ended");
                    }
                    // handle "s_detbuf_idx" and "s_detect_buffer"
                    s_detbuf_idx = 0;
                    s_detect_buffer.clear();
                    s_detect_buffer.resize(POINT_DETECT_NUM, 0.0);
                }
                else if (interface_motion && !interface_stable)
                {
                    ROS_INFO("[2>>3>>1] waiting for motion stable in auto_detection process.");
                }
                else
                {
                    ROS_ERROR("[2>>3>>1] unknown condition: MODE[%s], STATUS[%s].",
                              NuclearMode.c_str(), NuclearState.c_str());
                }
            }
            // 2>>3>>2. deal with the condition that pre_detection process should be conducted
            else if ( !s_DETECT_BGN && !s_AUTOSTOP_FLAG )
            {
                if (!s_DETMOVE_WAIT && !s_DETMOVE_READY)
                {
                    mobot6_interface::NuclearControl  output_;
                    output_.commandID = STP_CMD_MOVE;
                    output_.data.resize(NUCLEAR_DOF, 0.0);
                    output_.data[0] = s_detect_rot;
                    output_.data[1] = 10.0;
                    pubNuclearCmd.publish(output_);
                    ROS_INFO("[2>>3>>2] publish STEPMOVE command to %f in auto_detect mode.", s_detect_rot);
                }
                else if (s_DETMOVE_WAIT && !s_DETMOVE_READY)
                {
                    ROS_INFO("[2>>3>>2] waiting for the end of motion process.");
                }
                else if (s_DETMOVE_WAIT && s_DETMOVE_READY)
                {
                    s_DETECT_BGN = true;
                    s_detect_idx = 0;
                    s_detbuf_idx = 0;
                    s_detect_buffer.clear();
                    s_detect_rotation.clear();
                    s_detect_intensity.clear();
                    s_detect_buffer.resize(POINT_DETECT_NUM, 0.0);
                    s_detect_rotation.resize(POINT_DETECT_NUM, 0.0);
                    s_detect_intensity.resize(POINT_DETECT_NUM, 0.0);
                }
                else
                    ROS_ERROR("[2>>3>>2] unknown condition occured.");
            }
        }
        // 2>>3>>3. deal with the condition that DETECTSTOP has been activated
        if ( s_AUTOSTOP_FLAG && new_detect_status_ )
        {
            // test wether the status has been stopped
            if ( interface_motion && interface_stable )
            {
                s_detect_idx = 0;
                s_DETECT_SAMPLE = false;
                s_AUTOSTOP_FLAG = false;
                ROS_WARN("[2>>3>>3] the auto_detection process has paused and aborted.");
            }
        }
        double cost_3 = ros::Time::now().toSec() - start_3;
        ROS_DEBUG("3. DETECT_SAMPLE process end. cost_time: %lf ms.", cost_3*1000);
        //
        // 2>>4. get control command and parameters from "/nuclear_command"
        double start_4 = ros::Time::now().toSec(); ;
        if ((NUCLEAR_COMMAND_HANDLLER.newMsg()) && (NuclearMode!=INTERFACE_MODE_RESET))
        {
            ROS_DEBUG("[part_4] get the control commands from '/nuclear_command'.");
            mobot6_interface::NuclearControl  input_ = NUCLEAR_COMMAND_HANDLLER.fetchMsg();
            std::string command_ = input_.commandID;
            // 2>>4>>1. the condition with AUTO_SAMPLE status, and "STEPAUTOSTOP" is requested
            if ((s_AUTO_SAMPLE) && (command_==STP_CMD_AUTOSTOP))
            {
                s_AUTOSTOP_FLAG = true;
                mobot6_interface::NuclearControl  output_;
                output_.commandID = STP_CMD_STOP;
                output_.data.clear();
                output_.data.resize(NUCLEAR_DOF, 0.0);
                pubNuclearCmd.publish( output_ );
                ROS_WARN("[2>>4>>1] send 'STEPAUTOSTOP' command to interface, AUTOSTOP_FLAG:[%d].", s_AUTOSTOP_FLAG);
            }
            // 2>>4>>2. the condition with DETECT_SAMPLE status, and "STEPDETECT" is requested
            else if ((s_DETECT_SAMPLE) && (command_==STP_CMD_AUTOSTOP))
            {
                s_AUTOSTOP_FLAG = true;
                mobot6_interface::NuclearControl  output_;
                output_.commandID = STP_CMD_STOP;
                output_.data.clear();
                output_.data.resize(NUCLEAR_DOF, 0.0);
                pubNuclearCmd.publish( output_ );
                ROS_WARN("[2>>4>>2] send 'STEPDETECTSTOP' command to interface, AUTOSTOP_FLAG:[%d].", s_AUTOSTOP_FLAG);
            }
            //
            // handle the condition that auto_sampling process has not been activated
            else if (!s_AUTO_SAMPLE)
            {
                ROS_INFO("AAAAA [%s] [%s]", NuclearMode.c_str(), NuclearState.c_str());
                // 2>>4>>3. the condition that "STEPRESET" is requested
                if ( command_ == STP_CMD_RESET )
                {
                    pubNuclearCmd.publish( input_ );
                    ROS_WARN("[2>>4>>3] send 'STEPRESET' command to interface.");
                }
                // 2>>4>>4. the condition with MODE_MOVE && STABLE_STATE
                else if ((NuclearMode==INTERFACE_MODE_MOVE) && (NuclearState==INTERFACE_STATE_STABLE))
                {
                    if (command_ == STP_CMD_HOME)
                    {
                        pubNuclearCmd.publish( input_ );
                        ROS_WARN("[2>>4>>4] send 'STEPHOME' command to interface.");
                    }
                    else if (command_ == STP_CMD_MOVE)
                    {
                        pubNuclearCmd.publish( input_ );
                        ROS_WARN("[2>>4>>4] send 'STEPMOVE' command to interface.");
                    }
                    else if (command_ == STP_CMD_AUTO)
                    {
                        // reset the falgs
                        s_AUTO_SAMPLE   = true;
                        s_SAMPLE_BGN    = false;
                        s_AUTOSTOP_FLAG = false;
                        if ( fabs(joint_rotation) <= 0.1 )
                        {
                            s_AUTOMOVE_READY = true;
                            s_AUTOMOVE_WAIT = true;
                        }
                        else
                        {
                            s_AUTOMOVE_READY = false;
                            s_AUTOMOVE_WAIT = false;
                        }
                        // calculate the "s_sample_num" and "s_sample_interval"
                        s_sample_num = (int)(360.0/input_.data[0])+1;
                        s_sample_interval = 360.0/(double)(s_sample_num-1);

                        ROS_WARN("[2>>4>>4] get the 'STEPAUTO' command, interval:[%f], num:[%d]",
                                 s_sample_interval, s_sample_num);
                    }
                    else if (command_ == STP_CMD_DETECT)
                    {
                        s_detupdate_num = (int)(input_.data[0]);
                        s_detect_rot = input_.data[1];
                        // reset the detect flags
                        s_DETECT_SAMPLE = true;
                        s_DETECT_BGN    = false;
                        s_AUTOSTOP_FLAG  = false;
                        s_DETECT_UPDATE = false;
                        if ( fabs(joint_rotation-s_detect_rot) <= 0.1 )
                        {
                            s_DETMOVE_READY = true;
                            s_DETMOVE_WAIT = true;
                        }
                        else
                        {
                            s_DETMOVE_READY = false;
                            s_DETMOVE_WAIT = false;
                        }
                        //
                        ROS_WARN("[2>>4>>4] get the 'STEPDETECT' command.");
                    }
                    else if (command_ == STP_CMD_SAVEFILE)
                    {
                        if (reserve_seq >= 1)
                            reserve_flag = true;
                        if (res_det1_seq >= 1)
                            res_det1_flag = true;
                        if (res_det2_seq >= 1)
                            res_det2_flag = true;
                        //
                        ROS_WARN("[2>>4>>4] get the 'STEPSAVEFILE' command. res[%d], res_det1[%d], res_det2[%d].",
                                 reserve_flag, res_det1_flag, res_det2_flag);
                    }
                }
                // 2>>4>>5. the condition with MODE_MOVE && MOVING_STATE
                else if ((NuclearMode==INTERFACE_MODE_MOVE) && (NuclearState==INTERFACE_STATE_MOVING))
                {
                    if (command_ == STP_CMD_STOP)
                    {
                        pubNuclearCmd.publish( input_ );
                        ROS_WARN("[2>>4>>5] send 'STEPSTOP' command to interface.");
                    }
                }
                // 2>>4>>6. the condition with MODE_HOME && STABLE_STATE
                else if ((NuclearMode==INTERFACE_MODE_HOME) && (NuclearState==INTERFACE_STATE_MOVING))
                {
                    if (command_ == STP_CMD_STOP)
                    {
                        pubNuclearCmd.publish( input_ );
                        ROS_WARN("[2>>4>>6] send 'STEPSTOP' command to interface.");
                    }
                }
            }
        }
        double cost_4 = ros::Time::now().toSec() - start_4;
        ROS_DEBUG("4. AUTO_SAMPLE process end. cost_time: %lf ms.", cost_4*1000);
        //
        // 2>>5. save and publish the intensity information of auto_sampling and auto_detection
        // 2>>5>>1. obtain the
        new_nav_flag = false;
        double  pos_x_, pos_y_, theta_;
        if ( NAV_ODOMETRY_HANDLLER.newMsg() )
        {
            new_nav_flag = true;
            nav_odometry = NAV_ODOMETRY_HANDLLER.fetchMsg();
            pos_x_ = nav_odometry.pose.pose.position.x;
            pos_y_ = nav_odometry.pose.pose.position.y;
            theta_ = quat2yaw(nav_odometry.pose.pose.orientation);
            if(reserve_flag)
		odom_auto_=nav_odometry;
            if(res_det1_flag)
		odom_det1_=nav_odometry;
            if(res_det2_flag)
		odom_det2_=nav_odometry;

        }
        // 2>>5>>2. publish the auto_sampling measurements to "/nuclear_intensity"
        if ( reserve_seq >= 1 && new_nav_flag )
        {
            // publish auto_sample message to topic
            mobot6_interface::Intensity  intensity_state_;
            intensity_state_.header.stamp = ros::Time::now();
            intensity_state_.detect_seq = reserve_seq;
            intensity_state_.pose = odom_auto_.pose.pose;
            intensity_state_.detect_rotation = reserve_rotation;
            intensity_state_.detect_intensity = reserve_intensity;
            intensity_state_.detect_rotation_max = reserve_rotation_max;
            intensity_state_.detect_intensity_max = reserve_intensity_max;
            pubIntensityState.publish( intensity_state_ );
            // save auto_sample message to "SampleLog.txt"
            if ( reserve_flag )
            {
                // obtain the timer and odom_pose
                time_seconds = time(0);
                now_time = localtime(&time_seconds);
                // save the file to "SampleLog.txt"
                write_ofs.open( AUTO_SAMPLE_PATH, std::ofstream::out | std::ofstream::app );
                write_ofs << std::endl << STREAM_SEQ_SEGMENT << std::endl;
                write_ofs << "time:\t" << now_time->tm_hour << ":"
                          << now_time->tm_min << ":"
                          << now_time->tm_sec << std::endl;
                write_ofs << "sequence:\t" << reserve_seq << std::endl;
                write_ofs << "odom_pose:\t" << pos_x_ << "\t"
                          << pos_y_ << "\t" << theta_ << std::endl;
                write_ofs << "rotation_max:\t" << reserve_rotation_max << std::endl;
                write_ofs << "intensity_max:\t" << reserve_intensity_max << std::endl;
                write_ofs.close();
                //
                reserve_flag = false;
            }
        }
        // 2>>5>>3. publish the auto_detection measurements of point_1 to "/nuclear_intensity_point1"
        if ( res_det1_seq >= 1 && new_nav_flag )
        {
            // publish auto_detection message to topic
            mobot6_interface::PointIntensity  intensity_point1_;
            intensity_point1_.header.stamp = ros::Time::now();
            intensity_point1_.detect_seq = res_det1_seq;
            intensity_point1_.pose = odom_det1_.pose.pose;
            intensity_point1_.detect_rotation = res_det1_rotation[0];
            intensity_point1_.forward_intensity = res_det1_intensity[0];
            intensity_point1_.inverse_intensity = res_det1_intensity[1];
            pubIntensityPoint1.publish( intensity_point1_ );
            // save auto_detect message to "DetectLog.txt"
            if ( res_det1_flag )
            {
                // obtain the timer and odom_pose
                time_seconds = time(0);
                now_time = localtime(&time_seconds);
                // save the file to "DetectLog.txt"
                write_ofs.open( AUTO_DETECT_PATH, std::ofstream::out | std::ofstream::app );
                write_ofs << std::endl << STREAM_SEQ_SEGMENT << std::endl;
                write_ofs << "time:\t" << now_time->tm_hour << ":"
                          << now_time->tm_min << ":"
                          << now_time->tm_sec << std::endl;
                write_ofs << "point:\t" << 1 << std::endl;
                write_ofs << "sequence:\t" << res_det1_seq << std::endl;
                write_ofs << "odom_pose:\t" << pos_x_ << "\t"
                          << pos_y_ << "\t" << theta_ << std::endl;
                write_ofs << "rotation:\t" << res_det1_rotation[0] <<std::endl;
                write_ofs << "forward_intensity:\t" << res_det1_intensity[0] << std::endl;
                write_ofs << "inverse_intensity:\t" << res_det1_intensity[1] << std::endl;
                write_ofs.close();
                //
                res_det1_flag = false;
            }
        }
        // 2>>5>>4. publish the auto_detection measurements of point_2 to "/nuclear_intensity_point2"
        if ( res_det2_seq >= 1 && new_nav_flag )
        {
            // publish auto_detection message to topic
            mobot6_interface::PointIntensity  intensity_point2_;
            intensity_point2_.header.stamp = ros::Time::now();
            intensity_point2_.detect_seq = res_det2_seq;
            intensity_point2_.pose = odom_det2_.pose.pose;
            intensity_point2_.detect_rotation = res_det2_rotation[0];
            intensity_point2_.forward_intensity = res_det2_intensity[0];
            intensity_point2_.inverse_intensity = res_det2_intensity[1];
            pubIntensityPoint2.publish( intensity_point2_ );
            // save auto_detect message to "DetectLog.txt"
            if ( res_det2_flag )
            {
                // obtain the timer and odom_pose
                time_seconds = time(0);
                now_time = localtime(&time_seconds);
                // save the file to "DetectLog.txt"
                write_ofs.open( AUTO_DETECT_PATH, std::ofstream::out | std::ofstream::app );
                write_ofs << std::endl << STREAM_SEQ_SEGMENT << std::endl;
                write_ofs << "time:\t" << now_time->tm_hour << ":"
                          << now_time->tm_min << ":"
                          << now_time->tm_sec << std::endl;
                write_ofs << "point:\t" << 2 << std::endl;
                write_ofs << "sequence:\t" << res_det2_seq << std::endl;
                write_ofs << "odom_pose:\t" << pos_x_ << "\t"
                          << pos_y_ << "\t" << theta_ << std::endl;
                write_ofs << "rotation:\t" << res_det2_rotation[0] <<std::endl;
                write_ofs << "forward_intensity:\t" << res_det2_intensity[0] << std::endl;
                write_ofs << "inverse_intensity:\t" << res_det2_intensity[1] << std::endl;
                write_ofs.close();
                //
                res_det2_flag = false;
            }
        }
        //
        // 2>>6. time control
        double sleep_time = p_LOOP_TIME - ( ros::Time::now().toSec() - start );
        if ( sleep_time > 0 )
        {
            ros::Duration ( sleep_time ).sleep();
            ROS_DEBUG( "total cost time: %lf ms", (p_LOOP_TIME-sleep_time)*1000 );
        }
        else
            ROS_WARN( "control loop over time: %f ms", -sleep_time*1000 );
    } // main loop end
    ros::waitForShutdown();
    return 0;
}
