#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include "mobot6_interface/Intensity.h"

#include <math.h>
#include <iostream>
#include <log4cxx/logger.h>
#include <vector>

typedef     const char*     CStrPtr;

//--------------------------------------------------------------------------------
#define     UNCLEAR_INTENSITY       "/nuclear_intensity"
#define     VEHICLE_NAVIGATION      "/vehicle_nav_command"
#define     STATE_GENERATOR         "/generator_state"

#define     NAVIGATION_LOOP         0.050
#define     MATH_PI                 3.14159265
#define     PLAN_LENGTH_STEP        2.0

#define     _in     const &
#define     _inc    const
#define     _out          &
#define     _ret    const
#define     _wt           &
#define     _rd     const &

//################################################
//###  1. define the class StringHandllerOnce  ###
//################################################
class NuclearIntensityHandller
{
public:
    void callback(const mobot6_interface::Intensity::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    NuclearIntensityHandller()
        : mCrtMsg()
        , mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const mobot6_interface::Intensity & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    mobot6_interface::Intensity mCrtMsg;
    bool mNewMsg;
};


//#########################################
//###  2. define the Function quat2yaw  ###
//#########################################
double quat2yaw(const geometry_msgs::Quaternion msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    return yaw;
}


//############################################
//###  3. define the Destination Function  ###
//############################################
// note: variable "sensor_rot" should be in radius
int Plan_Destination ( const geometry_msgs::Pose base_pose,
                       const double sensor_rot,
                       const double plan_step,
                       geometry_msgs::Pose& plan_pose )
{
    double  pos_x_ = base_pose.position.x;
    double  pos_y_ = base_pose.position.y;
    double  pos_theta_ = quat2yaw( base_pose.orientation );
    double  orientation_ = pos_theta_ - sensor_rot;

    double  plan_x_ = pos_x_ + plan_step * cos(orientation_);
    double  plan_y_ = pos_y_ + plan_step * sin(orientation_);
    geometry_msgs::Quaternion   plan_quat_ =
            tf::createQuaternionMsgFromRollPitchYaw(
                0.0, 0.0, orientation_ );

    plan_pose.position.x = plan_x_;
    plan_pose.position.y = plan_y_;
    plan_pose.position.z = 0.0;
    plan_pose.orientation = plan_quat_;

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
//###  4. define the Main Function  ###
//#####################################
int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_generator");
    ros::NodeHandle     Node_Handle;
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(
                // ros::console::g_level_lookup[ros::console::levels::Debug]);

    // 1>>1. init the subscribers and publishers
    NuclearIntensityHandller    INTENSITY_HANDLLER;
    ros::Subscriber subIntensityMsg =
            Node_Handle.subscribe<mobot6_interface::Intensity>(
                UNCLEAR_INTENSITY, 3, &NuclearIntensityHandller::callback,
                &INTENSITY_HANDLLER);
    ros::Publisher  pubNavigationCmd =
            Node_Handle.advertise<geometry_msgs::PoseStamped>(
                VEHICLE_NAVIGATION, 3, false);
    ros::Publisher  pubGenState =
            Node_Handle.advertise<std_msgs::String>(
                STATE_GENERATOR, 3, false);

    // 1>>2. define several variables
    bool    new_command = false;
    int     current_seq = 0;
    geometry_msgs::Pose current_pose;
    geometry_msgs::Pose current_cmd_pose;

    // 1>>3. waiting for intensity message to init the navigation command
    while(ros::ok())
    {
        ros::spinOnce();
        if ( INTENSITY_HANDLLER.newMsg() )
        {
            mobot6_interface::Intensity input_ = INTENSITY_HANDLLER.fetchMsg();
            double  max_rot_ = input_.detect_rotation_max * MATH_PI / 180.0;
            //
            geometry_msgs::PoseStamped  output_;
            output_.header.stamp = ros::Time::now();
            output_.header.seq = input_.detect_seq;
            Plan_Destination( input_.pose, max_rot_, PLAN_LENGTH_STEP, output_.pose );
            pubNavigationCmd.publish( output_ );
            //
            new_command = true;
            current_seq = input_.detect_seq;
            current_pose = input_.pose;
            current_cmd_pose = output_.pose;
            break;
        }
        ros::Duration(0.5).sleep();
        ROS_INFO("waiting for intensity messages update.");
    }

    // 2. start the main loop
    ROS_INFO("main loop start");
    while(ros::ok())
    {
        double  start = ros::Time::now().toSec();
        ros::spinOnce();
        // 2>>1. obtain the new intensity message
        if ( INTENSITY_HANDLLER.newMsg() )
        {
            mobot6_interface::Intensity input_ = INTENSITY_HANDLLER.fetchMsg();
            double  max_rot_ = input_.detect_rotation_max * MATH_PI / 180.0; // added
            ROS_WARN("detect_seq:[%d]", input_.detect_seq);
            //
            geometry_msgs::PoseStamped  output_;
            output_.header.stamp = ros::Time::now();
            output_.header.seq = input_.detect_seq;
            Plan_Destination( input_.pose, max_rot_, PLAN_LENGTH_STEP, output_.pose );
            pubNavigationCmd.publish( output_ );
            //
            if ( new_command )
            {
                if ( (fabs(current_pose.position.x-input_.pose.position.x)>0.01)||
                     (fabs(current_pose.position.y-input_.pose.position.y)>0.01)||
                     (fabs(current_pose.orientation.z-input_.pose.orientation.z)>0.08) )
                    new_command = false;
            }
            current_pose = input_.pose;
            //
            if ( current_seq < input_.detect_seq )
            {
                new_command = true;
                current_seq = input_.detect_seq;
                current_cmd_pose = output_.pose;
            }
        }
        else
        {
            geometry_msgs::PoseStamped  output_;
            output_.header.stamp = ros::Time::now();
            output_.header.seq = current_seq;
            output_.pose = current_cmd_pose;
            pubNavigationCmd.publish( output_ );
        }
        //
        // 2>>2. publish the generator status
        std::string     stateout;
        if ( new_command )
            StringAppend( stateout, "NEW_COMMAND  " );
        else
            StringAppend( stateout, "OLD_COMMAND  " );
        //
        StringAppend( stateout, "%d  ", current_seq );
        //
        StringAppend( stateout, "%f  ", current_cmd_pose.position.x );
        StringAppend( stateout, "%f  ", current_cmd_pose.position.y );
        StringAppend( stateout, "%f  ", quat2yaw(current_cmd_pose.orientation) );
        //
        std_msgs::String  status_;
        status_.data = stateout;
        pubGenState.publish( status_ );
        //
        // 2>>3. time control
        double sleep_time = NAVIGATION_LOOP - ( ros::Time::now().toSec() - start );
        if ( sleep_time > 0 )
        {
            ros::Duration ( sleep_time ).sleep();
            ROS_DEBUG( "total cost time: %lf ms", (NAVIGATION_LOOP-sleep_time)*1000 );
        }
        else
            ROS_WARN( "control loop over time: %lf ms", -sleep_time*1000 );
    }
    // main loop end
    return 0;
}


