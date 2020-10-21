#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
//#include <sys/time.h>
#include <boost/algorithm/string.hpp>
#include <boost/system/error_code.hpp>

#include <log4cxx/logger.h>

// -----------the dependent header files from xc-------------
#include <cmath>
#include <vector>
#include <deque>
#include <cerrno>
#include <cstdarg>
#include <cstring>
#include <stdexcept>
#include <stdint.h>

typedef const char* CStrPtr;
//-----------------------------------------------------------


// 1. function input, output parameter and return value prefix
#define _in   const &   // input parameter of right value
#define _inc  const     // input parameter of right value
#define _out        &   // output parameter of left value

#define _wt         &   // return value of writable left value
#define _rd   const &   // return value of readonly left value
#define _ret  const     // return by value (use c++ complier optimaization)

// 2. control loop durations in seconds
#define     GMAS_LOOP         0.025       // 0.050
#define     COORDINATOR_LOOP  0.020
#define     INTERFACE_LOOP    0.020       // 0.0125

// 3. define the relevent topics
// define the subscribed topics
#define     INTERFACE_JOINT_PATH_COMMAND  "/joint_path_command"         // <trajectory_msgs::JointTrajectory>
#define     INTERFACE_COMMAND             "/interface_command"
#define     INTERFACE_COMMAND_GRIP        "/interface_command_grip"
#define     INTERFACE_JOINT_STATES        "/mobot_vehicle/joint_states" // gazebo pub, coord sub
// define the published topics
#define     INTERFACE_STATES              "/interface_states"
#define     INTERFACE_ARM_COMMAND         "/mobot_vehicle/manipulator_position_controllers/command"
#define     INTERFACE_GRIP_VEL_COMMAND    "/mobot_vehicle/gripper_velocity_controller/command"
#define     INTERFACE_GRIP_POS_COMMAND    "/mobot_vehicle/gripper_position_controllers/command"

// 4. enumerate the motion commands subscribed from coordinator
#define     INTERFACE_CMD_MOVE            "MOVE"
#define     INTERFACE_CMD_CLEAR           "CLEAR" // dummy parameter??
#define     INTERFACE_CMD_HALT            "HALT"  // dummy parameter??
#define     INTERFACE_CMD_STOP            "STOP"
#define     INTERFACE_CMD_RESET           "RESET"

// 5. enumerate the motion sates publisher to coordinator
#define     INTERFACE_STATE_SPARATER      "\t"
#define     INTERFACE_STATE_MOVE          "MOVE"
#define     INTERFACE_STATE_INIT          "INIT"
#define     INTERFACE_STATE_STOP          "STOP"
#define     INTERFACE_STATE_HOME          "HOME"

#define     INTERFACE_STATE_MOVING        "MOVING"
#define     INTERFACE_STATE_STABLE        "STABLE"

#define     INTERFACE_STATE_DELAYED       "DELAYED"
#define     INTERFACE_STATE_NORMAL        "NORMAL"

// 6. define the joint_names in ROS setting
#define     JOINT1_ROS_AXIS     "joint_turret_rotate"
#define     JOINT2_ROS_AXIS     "joint_bigarm_pitch"
#define     JOINT3_ROS_AXIS     "joint_bigarm_linear"
#define     JOINT4_ROS_AXIS     "joint_forearm_pitch"
#define     JOINT5_ROS_AXIS     "joint_forearm_rotate"
#define     JOINT6_ROS_AXIS     "joint_wrist_pitch"
#define     JOINT7_ROS_AXIS     "joint_wrist_rotate"
#define     JOINT8_ROS_AXIS     "joint_rod_left1"

std::string JOINTS_ROS_AXIS[] = { JOINT1_ROS_AXIS,
                                  JOINT2_ROS_AXIS,
                                  JOINT3_ROS_AXIS,
                                  JOINT4_ROS_AXIS,
                                  JOINT5_ROS_AXIS,
                                  JOINT6_ROS_AXIS,
                                  JOINT7_ROS_AXIS,
                                  JOINT8_ROS_AXIS };

// 9. define the position limitation of manipulator and gripper
#define     LIMIT_POSITION_ARM          0.3491  // 20deg
#define     LIMIT_FOLLOW_TRAJECTORY     0.1396  // 8deg

#define     LIMIT_FORWARD_GRIP          1.2758
#define     LIMIT_REVERSE_GRIP          -0.01
double      LIMIT_GRIPPER[] = { LIMIT_FORWARD_GRIP, LIMIT_REVERSE_GRIP };

// 10. define the gripper command string
#define         GRIP_FORWARD            "GripForward"
#define         GRIP_REVERSE            "GripReverse"
#define         GRIP_STOP               "GripStop"

#define         GRIP_GAZEBO_FORWARD     0.2
#define         GRIP_GAZEBO_REVERSE     -0.2
#define         GRIP_GAZEBO_STOP        0.0

// 11. define the state positions of corresponding modbus registers
#define         ARM_JOINT_DOF           7  // just for arm_command, so 7 is right
#define         GRIP_JOINT_DOF          4  // maybe 1_joint / 4_joints, we need to practice
#define         MAX_AXES                8

// 12. enumerate mode states in interface
const int MODE_INIT  = 0;
const int MODE_MOVE  = 1;
const int MODE_STOP  = 2;

// 13. enumerate error flags in interface
const int ERROR_UNHANDLEABLE_INPUT = 1;
const int ERROR_INVALID_TRAJECTORY = 2;
const int ERROR_INVALID_FEEDBACK   = 3;
const int ERROR_INVALID_STATE      = 4;

//
class JointTrajectoryHandller
{
public:
    void callback(const trajectory_msgs::JointTrajectory::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    JointTrajectoryHandller()
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

    const trajectory_msgs::JointTrajectory & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    trajectory_msgs::JointTrajectory mCrtMsg;
    bool mNewMsg;
};

//
class StringHandller
{
public:
    void callback(const std_msgs::String::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
    }

    StringHandller()
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

    const std_msgs::String & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    std_msgs::String mCrtMsg;
    bool mNewMsg;
};

//
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


//
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


//
class PTTrajectory
{
public:

        PTTrajectory(std::vector< std::string> _in gmasNames, double _in timestep)
                : mAxis(gmasNames),
                  mData(gmasNames.size()),
                  mTimeStep(timestep)
	{
	}

        PTTrajectory(std::vector< std::string> _in gmasNames, size_t _in steps, double _in timestep)
                : mAxis(gmasNames),
                  mData(gmasNames.size()),
                  mTimeStep(timestep)
	{
		Resize(steps);
	}

	PTTrajectory(PTTrajectory _in v)
                : mAxis(v.mAxis),
                  mData(v.mData),
                  mTimeStep(v.mTimeStep)
	{

	}

	virtual ~PTTrajectory()
	{

	}

        inline void Reset(std::vector< std::string> _in gmasNames, size_t _in steps, double _in timestep)
	{
                mAxis = gmasNames;
		mData.resize(mAxis.size());
		mTimeStep = timestep;
		Resize(steps);
	}

	inline size_t _ret NumOfAxis() const
	{
		return mAxis.size();
	}

	inline size_t _ret NumOfPoints()
	{
		return mData[0].size();
	}

	inline void clear()
	{
		for (size_t i = 0; i < mData.size(); ++i)
		{
                        mData[i].clear();
		}
	}

	inline void Resize(size_t _in steps)
	{
		for (size_t i = 0; i < mData.size(); ++i)
		{
			mData[i].resize(steps);
		}
	}

	inline std::vector<double> _ret GetPoint(size_t _in step)
	{
		std::vector<double> ret(NumOfAxis());

		for (size_t i = 0; i < NumOfAxis(); ++i)
			ret[i] = mData[i][step];

		return ret;
	}

	inline void Set(size_t _in axis, size_t _in step, double _in v)
	{
		mData[axis][step] = v;
	}

	inline double _ret Get(size_t _in axis, size_t _in step) const
	{
		return mData[axis][step];
	}

	inline void SetStep(size_t _in step, std::vector<double> _in v)
	{
		for (size_t i = 0; i < NumOfAxis(); ++i)
			mData[i][step] = v[i];

		return;
	}

	inline void PushBack(std::vector<double> _in v)
	{
		for (size_t i = 0; i < NumOfAxis(); ++i)
			mData[i].push_back(v[i]);

		return;
	}

        inline std::vector<double> _ret PopFront( double _in ratio = 1.0,
                                               size_t _in except = 99,
                                               double _in ratio_except = 1.0 )
        {
            std::vector<double> ret( NumOfAxis() );
            for (size_t i = 0; i < NumOfAxis(); ++i)
            {
                if (i != except)  // the usual joint type and ratio
                    ret[i] = (mData[i].front()*ratio);
                else  // the excepted joint type and ratio
                    ret[i] = (mData[i].front()*ratio_except);
                mData[i].pop_front();
            }
            return ret;
	}

	inline void Append(PTTrajectory _in v)
	{
		for (size_t i = 0; i < NumOfAxis(); ++i)
		{
			mData[i].insert(mData[i].end(),
				v.mData[i].begin(),
				v.mData[i].end());
		}
	}

public:
	std::vector< std::deque<double> >  mData;
        std::vector< std::string >         mAxis;
	double                             mTimeStep;
};

//
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


//
class TrajectoryProcessor
{
public:
    TrajectoryProcessor(double _in step)
        : mAxisROS(),
          mAxisGMAS(),
          mTimeStep(step)
    {

    }

    TrajectoryProcessor(TrajectoryProcessor _in v)
        : mAxisROS(v.mAxisROS),
          mAxisGMAS(v.mAxisGMAS),
          mTimeStep(v.mTimeStep)
    {

    }

    virtual ~TrajectoryProcessor()
    {

    }

    void AddAxis(std::string _in joint, std::string _in gmas)
    {
        mAxisROS.push_back(joint);
        mAxisGMAS.push_back(gmas);
    }

    size_t _ret NumOfAxis()
    {
        return mAxisROS.size();
    }

    std::vector<std::string> _rd JOINTS() const
    {
        return mAxisROS;
    }

    std::vector<std::string> _rd GMAS() const
    {
        return mAxisGMAS;
    }

    PTTrajectory _ret MakeTrajectory()
    {
        return PTTrajectory(mAxisGMAS, mTimeStep);
    }

    Result _ret GenerateMap(trajectory_msgs::JointTrajectory _in trajecotry,
                            std::vector<size_t> _out ros2trj)
    {
        // check size
        if (trajecotry.joint_names.size() != NumOfAxis())
        {
            return Result(ERROR_INVALID_TRAJECTORY, "invalid trajecotry of disagreed joints");
        }
        // check map
        ros2trj.resize(NumOfAxis());
        for (size_t i = 0; i < trajecotry.joint_names.size(); ++i)
        {
            bool maped = false;
            for (size_t j = 0; j < NumOfAxis(); ++j)
            {
                if (trajecotry.joint_names[i] == mAxisROS[j])
                {
                    ros2trj[i] = j;
                    maped = true;
                }
            }
            if (!maped)
            {
                return Result(ERROR_INVALID_TRAJECTORY, "invalid trajecotry of disagreed joints");
            }
        }
        // check points size
        if (trajecotry.points.size() <= 0)
        {
            return Result(ERROR_INVALID_TRAJECTORY, "invalid trajecotry of empty data");
        }

        // return
        return Result::Success();
    }

    Result _ret TranslateDirect(trajectory_msgs::JointTrajectory _in trajectory,
                                PTTrajectory _out outputPOS,
                                std::vector<size_t> _out ros2trj)
    {
        // check
        Result res = GenerateMap(trajectory, ros2trj);
        if (res.is_failed())
            return res;

        // copy and check time step
        outputPOS.Reset(mAxisGMAS, trajectory.points.size(), mTimeStep); // trajectory.points.size()-1
        for (size_t p = 0; p < trajectory.points.size(); ++p)
        {
            trajectory_msgs::JointTrajectoryPoint pnt = trajectory.points[p];
            for (size_t i = 0; i < NumOfAxis(); ++i)
                outputPOS.Set(ros2trj[i], p, pnt.positions[i]);
        }

        // return
        return Result::Success();
    }


protected:
        std::vector<std::string> mAxisROS;
        std::vector<std::string> mAxisGMAS;
        double                   mTimeStep;
};

//
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


//
int main ( int argc, char **argv )
{
    // 1>> init ROS, do nothing, if a excpetion is thrown -------------------------
    ROS_INFO ( "initializing ROS" );
    ros::init ( argc, argv, "mobot_interface" );
    ros::NodeHandle NODE;
    ROS_INFO ( "initializing ROS succeed" );
    ros::Duration(1.5).sleep();

    // 2>> initialize the first-level loop
    while ( ros::ok() )
    {
        ROS_INFO ( "mian-loop begins" );
        ROS_INFO ( "setting parameters" );
        //
        const double            p_GMAS_TIME              = GMAS_LOOP;
        const double            p_LOOP_TIME              = INTERFACE_LOOP;
        //
        TrajectoryProcessor     TRAJ_PROCESSOR ( p_GMAS_TIME );
        for (size_t i = 0; i < ARM_JOINT_DOF; ++i)
        {
            TRAJ_PROCESSOR.AddAxis( JOINTS_ROS_AXIS[i], JOINTS_ROS_AXIS[i] );
        }
        //
        const size_t            p_MinimalStartSize     = 3;

        ROS_INFO ( "setting parameters succeed" );
        //
        ROS_INFO ( "initializing subscriber and publisher" );
        JointTrajectoryHandller TRAJECTORY_HANDLLER;
        StringHandller          COMMAND_HANDLLER;
        StringHandller          GRIP_COMMAND_HANDLLER;
        JointStateHandller      JOINT_STATES_HANDLLER;

        ros::Subscriber subscriber_joint_path =
                NODE.subscribe<trajectory_msgs::JointTrajectory> (
                    INTERFACE_JOINT_PATH_COMMAND, 1024, &JointTrajectoryHandller::callback,
                    &TRAJECTORY_HANDLLER );
        ros::Subscriber subscriber_command =
                NODE.subscribe<std_msgs::String> (
                    INTERFACE_COMMAND, 3, &StringHandller::callback,
                    &COMMAND_HANDLLER );
        ros::Subscriber subscriber_grip_command =
                NODE.subscribe<std_msgs::String> (
                    INTERFACE_COMMAND_GRIP, 3, &StringHandller::callback,
                    &GRIP_COMMAND_HANDLLER );
        ros::Subscriber subscriber_joint_states =
                NODE.subscribe<sensor_msgs::JointState> (
                    INTERFACE_JOINT_STATES, 3, &JointStateHandller::callback,
                    &JOINT_STATES_HANDLLER );

        ros::Publisher  pInterfaceStates =
                NODE.advertise<std_msgs::String>(INTERFACE_STATES, 1024);
        ros::Publisher  pInterfaceArmCmd =
                NODE.advertise<std_msgs::Float64MultiArray>(INTERFACE_ARM_COMMAND, 1024);
        ros::Publisher  pInterfaceGripCmd =
                NODE.advertise<std_msgs::Float64>(INTERFACE_GRIP_VEL_COMMAND, 1024);
        ros::Publisher  pInterfaceGripPosCmd =
                NODE.advertise<std_msgs::Float64MultiArray>(INTERFACE_GRIP_POS_COMMAND, 1024);

        ROS_INFO ( "initializing subscriber and publisher succeed" );
        //
        //---excpetions NO.2 is thrown - begin ------------------------------------------
        try
        {
            int s_MODE  = MODE_INIT;
            ROS_INFO ( "starting control-loop" );
            //
            PTTrajectory            s_TRAJECTORY             = TRAJ_PROCESSOR.MakeTrajectory();
            double                  s_LAST_GRIP_FEEDBACK     = 0.0;  // the gripper joint feedback
            std::vector<double>     s_ARM_JOINT_STATES( TRAJ_PROCESSOR.NumOfAxis(), 0.0 );
            //
            std::vector<double>     s_ARM_LAST_COMMAND( TRAJ_PROCESSOR.NumOfAxis(), 0.0 );

            bool                    s_MOVING                 = false;
            bool                    s_GRIP_MOVING            = false;
            double                  s_CURRENT_GRIP_CMD       = 0.0;
            double                  s_LAST_GRIP_CMD          = 0.0;
            int                     s_REFRESH_COUNT          = 0;
            //
            int                     s_TrajDelayThreshold     = 5;
            int                     s_NOTRAJ_COUNT           = 0;
            int                     s_DELAY_COUNT            = 0;
            bool                    s_DELAY_FLAG             = false;
            //
            std::string             strActions;
            std::string             strStatus;
            strActions.reserve ( 256 );
            strStatus.reserve ( 256 );
            // 
            std::vector<size_t> ros2trj(TRAJ_PROCESSOR.NumOfAxis(), 0);
            for (size_t i = 0; i < TRAJ_PROCESSOR.NumOfAxis(); ++i)
                ros2trj[i] = i;
            //--------------------------------------------------------------------------
            while ( ros::ok() )
            {
                ros::spinOnce();
                if ( JOINT_STATES_HANDLLER.newMsg() )
                {
                    sensor_msgs::JointState temp_joint_state = JOINT_STATES_HANDLLER.fetchMsg();
                    int temp_count = 0;
                    for (size_t i = 0; i < temp_joint_state.name.size(); ++i)
                    {
                        for (size_t j = 0; j < TRAJ_PROCESSOR.NumOfAxis(); ++j)
                        {
                            if( TRAJ_PROCESSOR.JOINTS()[j] == temp_joint_state.name[i] )
                            {
                                s_ARM_LAST_COMMAND[j] = temp_joint_state.position[i];
                                s_ARM_JOINT_STATES[j] = temp_joint_state.position[i];
                                temp_count++;
                            }
                        }
                    }
                    if (temp_count == TRAJ_PROCESSOR.NumOfAxis())
                        break;
                    else
                        ROS_ERROR("cannot init all the manipulator joint_states.");
                }
                ros::Duration(0.25).sleep();
                ROS_INFO("waiting for joint state update.");
            }
            //--------------------------------------------------------------------------
            // 3>> initialize the second-level loop
            ROS_INFO ( "control loop started" );
            while( ros::ok() && s_MODE != MODE_STOP )
            {
                strActions.clear();
                double start = ros::Time::now().toSec();
                bool manipulator_limitation;
                bool f_grip_limitation;
                bool r_grip_limitaion;
                int iCnt;

                //---excpetions NO.1 is thrown------------------------------------------
                try
                {
                    bool communicated = false;
                    //---spin-----------------------------------------------------------
                    ros::spinOnce();
                    //
                    double start_3 = ros::Time::now().toSec();
                    //
                    if ( COMMAND_HANDLLER.newMsg() )
                    {
                        std_msgs::String input_ = COMMAND_HANDLLER.fetchMsg();
                        std::string input = input_.data;
                        boost::to_upper ( input );
                        std::cout << "get input " <<input << std::endl;

                        std::vector<std::string> result;
                        boost::split( result, input, boost::is_any_of(" "), boost::algorithm::token_compress_on );
                        std::string COMMAND = result[0];
                        StringAppend ( strActions, "1. %s\t", input.c_str () );
                        //
                        if ( s_MODE == MODE_MOVE )
                        {
                            if (COMMAND == INTERFACE_CMD_STOP)
                            {
                                s_MODE = MODE_STOP;
                            }
                            else if (COMMAND == INTERFACE_CMD_CLEAR)
                            {
                                s_TRAJECTORY.clear();
                            }
                        }
                        //
                        else if ( s_MODE == MODE_INIT )
                        {
                            if ( COMMAND == INTERFACE_CMD_STOP )
                            {
                                s_TRAJECTORY.clear();
                                TRAJECTORY_HANDLLER.clear ();
                                s_MODE = MODE_STOP;
                                ROS_WARN("INIT STATE: STOP COMMAND");
                            }
                            else if ( COMMAND == INTERFACE_CMD_MOVE )
                            {
                                s_TRAJECTORY.clear();
                                TRAJECTORY_HANDLLER.clear ();
                                s_MODE = MODE_MOVE;
                                ROS_WARN("INIT STATE: MOVE COMMAND");
                            }
                        }
                    }
                    double cost_3 = ros::Time::now().toSec() - start_3;
                    ROS_DEBUG("3. COMMAND_HANDLLER end. cost_time: %lf ms.", cost_3*1000);
                    //
                    double start_4 = ros::Time::now().toSec();
                    //
                    if ( TRAJECTORY_HANDLLER.newMsg() )
                    {
                        StringAppend ( strActions, "2. new_trajectory\t" );
                        trajectory_msgs::JointTrajectory input = TRAJECTORY_HANDLLER.fetchMsg();
                        //
                        if ( s_MODE == MODE_MOVE )
                        {
                            PTTrajectory output = TRAJ_PROCESSOR.MakeTrajectory();
                            Result ec = TRAJ_PROCESSOR.TranslateDirect ( input, output, ros2trj );
                            if ( ec.is_failed() )
                                throw ec;
                            s_TRAJECTORY.Append ( output );
                        }
                        //
                        else
                        {
                            s_TRAJECTORY.clear();
                        }
                        //
                        s_NOTRAJ_COUNT = 0;
                        s_DELAY_COUNT = 0;
                    }
                    // determine wether the input is delayed trajectory
                    else
                    {
                        if ( s_MODE == MODE_MOVE )
                        {
                            s_NOTRAJ_COUNT++;
                            if ((s_NOTRAJ_COUNT >= 3) && (s_TRAJECTORY.NumOfPoints() >= s_TrajDelayThreshold))
                            {
                                s_NOTRAJ_COUNT = 0;
                                s_DELAY_COUNT++;
                            }
                            else if ((s_NOTRAJ_COUNT >= 3) && (s_TRAJECTORY.NumOfPoints() < s_TrajDelayThreshold))
                            {
                                s_NOTRAJ_COUNT = 0;
                                s_DELAY_COUNT = 0;
                            }
                        }
                        else
                        {
                            s_NOTRAJ_COUNT = 0;
                            s_DELAY_COUNT = 0;
                        }
                    }

                    double cost_4 = ros::Time::now().toSec() - start_4;
                    ROS_DEBUG("4. TRAJECTORY_HANDLLER end. cost_time: %lf ms.", cost_4*1000);
                    //
                    double start_5 = ros::Time::now().toSec();
                    //
                    if ( s_MODE != MODE_STOP )
                    {
                        StringAppend ( strActions, "3. new_trajectory\t" );
                        //
                        double temp_grip_feedback;
                        int temp_count = 0;
                        if ( JOINT_STATES_HANDLLER.newMsg() )
                        {
                            sensor_msgs::JointState input = JOINT_STATES_HANDLLER.fetchMsg();
                            bool grip_feedback_flag = false;
                            for (size_t i = 0; i < input.name.size(); ++i)  // JOINT8_ROS_AXIS
                            {
                                if( input.name[i] == JOINT8_ROS_AXIS )
                                {
                                    temp_grip_feedback = input.position[i];
                                    grip_feedback_flag = true;
                                    s_LAST_GRIP_FEEDBACK = temp_grip_feedback;
                                }
                                for (size_t j = 0; j < TRAJ_PROCESSOR.NumOfAxis(); ++j)
                                {
                                    if( input.name[i] == TRAJ_PROCESSOR.JOINTS()[j] )
                                    {
                                        s_ARM_JOINT_STATES[j] = input.position[i];
                                        temp_count++;
                                    }
                                }
                            }
                            if( temp_count != TRAJ_PROCESSOR.NumOfAxis() )
                                ROS_ERROR("Cannot get all the joint states from joint_states topic. temp_count[%d]", temp_count);
                            if( !grip_feedback_flag )
                            {
                                temp_grip_feedback = s_LAST_GRIP_FEEDBACK;
                            }
                        }
                        //
                        if( temp_grip_feedback >= LIMIT_GRIPPER[0] )
                        {
                            f_grip_limitation = true;
                            r_grip_limitaion  = false;
                        }
                        else if( temp_grip_feedback <= LIMIT_GRIPPER[1] )
                        {
                            f_grip_limitation = false;
                            r_grip_limitaion  = true;
                        }
                        else if((temp_grip_feedback<LIMIT_GRIPPER[0]) && (temp_grip_feedback>LIMIT_GRIPPER[1]))
                        {
                            f_grip_limitation = false;
                            r_grip_limitaion  = false;
                        }
                        manipulator_limitation = false;
                        for (size_t i = 0; i < TRAJ_PROCESSOR.NumOfAxis(); ++i)
                        {
                            size_t  temp_traj_num = s_TRAJECTORY.NumOfPoints();
                            if ( (int)temp_traj_num >= 1 )
                            {
                                if ( abs(s_ARM_JOINT_STATES[i]-s_TRAJECTORY.GetPoint(temp_traj_num-1)[i])
                                     >= LIMIT_POSITION_ARM )
                                    manipulator_limitation = true;
                            }
                        }
                    }

                    double cost_5 = ros::Time::now().toSec() - start_5;
                    ROS_DEBUG("5. TRAJECTORY_FEEDBACK end. cost_time: %lf ms.", cost_5*1000);
                    //
                    double start_6 = ros::Time::now().toSec();

                    if ( s_MODE == MODE_MOVE )
                    {
                        if( manipulator_limitation )
                        {
                            ROS_ERROR("trajectory limitation");
                            s_DELAY_FLAG = true;
                            manipulator_limitation = false;
                        }
                        else if ((s_DELAY_COUNT >= 2) && (!manipulator_limitation))
                        {
                            ROS_ERROR("[Trajectory Clear] buffer_num[%zu].", s_TRAJECTORY.NumOfPoints());
                            s_TRAJECTORY.clear();
                            s_DELAY_FLAG = true; // inform the coordinator of stopping trajectory_command
                            s_DELAY_COUNT = 0;
                        }
                        else
                        {
                            if ( (!s_MOVING) && (s_TRAJECTORY.NumOfPoints() >= p_MinimalStartSize) && (!s_GRIP_MOVING) )
                            {
                                s_MOVING = true;
                            }
                            if ( s_MOVING  && !communicated )
                            {
                                communicated = true;
                                bool follow_trajectory_flag = true;
                                for (size_t i = 0; i < TRAJ_PROCESSOR.NumOfAxis(); ++i)
                                {
                                    if( abs(s_ARM_JOINT_STATES[i] - s_ARM_LAST_COMMAND[i]) > LIMIT_FOLLOW_TRAJECTORY )
                                        follow_trajectory_flag = false;
                                    ROS_INFO("the deviation of axis %02d: [%f]", i+1, abs(s_ARM_JOINT_STATES[i]-s_ARM_LAST_COMMAND[i]));
                                }
                                // 
                                if ((follow_trajectory_flag) && (s_TRAJECTORY.NumOfPoints()>0))
                                {
                                    StringAppend ( strActions, "4. send_trajectory\t" );
                                    std::vector<double> aline = s_TRAJECTORY.PopFront();
				    //
                                    std_msgs::Float64MultiArray output;
                                    output.data.resize( s_TRAJECTORY.NumOfAxis() );
                                    for (size_t i = 0; i < TRAJ_PROCESSOR.NumOfAxis(); ++i)
                                        output.data[i] = aline[i];
                                    pInterfaceArmCmd.publish( output );
				    //
                                    s_ARM_LAST_COMMAND = aline;
                                }
                                ROS_WARN("interface_buf_num: [%zu]", s_TRAJECTORY.NumOfPoints());

                                if (s_TRAJECTORY.NumOfPoints() == 0)
                                {
                                    s_MOVING = false;
                                    s_DELAY_FLAG = false;
                                    ROS_INFO ( "->  buffer empty:　stopped  <-");
                                }
                            }
		 	    //
                            else if ( !s_MOVING && !communicated )
                            {
                                if ( GRIP_COMMAND_HANDLLER.newMsg() )
                                {
                                    std_msgs::String input_ = GRIP_COMMAND_HANDLLER.fetchMsg();
                                    std::string input = input_.data;
                                    if ((!f_grip_limitation) && (!r_grip_limitaion))
                                    {
                                        if (input == GRIP_FORWARD)
                                            s_CURRENT_GRIP_CMD = GRIP_GAZEBO_FORWARD;
                                        else if (input == GRIP_REVERSE)
                                            s_CURRENT_GRIP_CMD = GRIP_GAZEBO_REVERSE;
                                        else if (input == GRIP_STOP)
                                            s_CURRENT_GRIP_CMD = GRIP_GAZEBO_STOP;
                                    }
                                    else if ((f_grip_limitation) && (!r_grip_limitaion))
                                    {
                                        if (input == GRIP_FORWARD)
                                        {
                                            s_CURRENT_GRIP_CMD = GRIP_GAZEBO_STOP;
                                            ROS_WARN("The forward limitation of gripper.");
                                        }
                                        else if (input == GRIP_REVERSE)
                                            s_CURRENT_GRIP_CMD = GRIP_GAZEBO_REVERSE;
                                        else if (input == GRIP_STOP)
                                            s_CURRENT_GRIP_CMD = GRIP_GAZEBO_STOP;
                                    }
                                    else if ((!f_grip_limitation) && (r_grip_limitaion))
                                    {
                                        if (input == GRIP_FORWARD)
                                            s_CURRENT_GRIP_CMD = GRIP_GAZEBO_FORWARD;
                                        else if (input == GRIP_REVERSE)
                                        {
                                            s_CURRENT_GRIP_CMD = GRIP_GAZEBO_STOP;
                                            ROS_WARN("The reverse limitation of gripper.");
                                        }
                                        else if (input == GRIP_STOP)
                                            s_CURRENT_GRIP_CMD = GRIP_GAZEBO_STOP;
                                    }
                                    //
                                    bool refresh_flag = false;
                                    if (s_CURRENT_GRIP_CMD != s_LAST_GRIP_CMD)
                                        refresh_flag = true;

                                    if (refresh_flag)
                                        s_REFRESH_COUNT++;
                                    else
                                        s_REFRESH_COUNT = 0;
                                    //
                                    if (refresh_flag && s_REFRESH_COUNT>=3)
                                    {
                                        std::cout << "New gripper command:" << s_CURRENT_GRIP_CMD << std::endl;

                                        std_msgs::Float64 vel_output;
                                        vel_output.data = s_CURRENT_GRIP_CMD;
                                        pInterfaceGripCmd.publish( vel_output );

                                        s_LAST_GRIP_CMD = s_CURRENT_GRIP_CMD;

                                        if ((s_CURRENT_GRIP_CMD == GRIP_GAZEBO_FORWARD) || (s_CURRENT_GRIP_CMD == GRIP_GAZEBO_REVERSE))
                                            s_GRIP_MOVING = true;
                                        else if (s_CURRENT_GRIP_CMD == GRIP_GAZEBO_STOP)
                                            s_GRIP_MOVING = false;

                                        s_REFRESH_COUNT = 0;
                                    }
                                }
                            }
                            std_msgs::Float64MultiArray pos_output;
                            pos_output.data.resize( GRIP_JOINT_DOF-1 );
                            for (size_t i = 0; i < GRIP_JOINT_DOF-1; ++i)
                                pos_output.data[i] = s_LAST_GRIP_FEEDBACK;
                            pInterfaceGripPosCmd.publish( pos_output );
                        }
                    }
                    //
                    else
                    {
                        s_MOVING = false;
                        //
                        std_msgs::Float64 vel_output;
                        vel_output.data = 0.0;
                        pInterfaceGripCmd.publish( vel_output );
                        //
                        std_msgs::Float64MultiArray pos_output;
                        pos_output.data.resize( GRIP_JOINT_DOF-1 );
                        for (size_t i = 0; i < GRIP_JOINT_DOF-1; ++i)
                            pos_output.data[i] = s_LAST_GRIP_FEEDBACK;
                        pInterfaceGripPosCmd.publish( pos_output );
                    }

                    double cost_6 = ros::Time::now().toSec() - start_6;
                    ROS_DEBUG("6. GMAS_COMMUNICATE end. cost_time: %lf ms.", cost_6*1000);
                    //
                    double start_7 = ros::Time::now().toSec();

                    strStatus.clear();
                    //
                    if ( s_MODE == MODE_MOVE )
                        StringAppend ( strStatus, INTERFACE_STATE_MOVE INTERFACE_STATE_SPARATER );
                    else if ( s_MODE == MODE_INIT )
                        StringAppend ( strStatus, INTERFACE_STATE_INIT INTERFACE_STATE_SPARATER );
                    else if ( s_MODE == MODE_STOP )
                        StringAppend ( strStatus, INTERFACE_STATE_STOP INTERFACE_STATE_SPARATER );
                    //
                    if ( s_MOVING )
                        StringAppend ( strStatus, INTERFACE_STATE_MOVING INTERFACE_STATE_SPARATER );
                    else
                        StringAppend ( strStatus, INTERFACE_STATE_STABLE INTERFACE_STATE_SPARATER );
                    //
                    if ( s_DELAY_FLAG )
                        StringAppend ( strStatus, INTERFACE_STATE_DELAYED INTERFACE_STATE_SPARATER );
                    else
                        StringAppend ( strStatus, INTERFACE_STATE_NORMAL INTERFACE_STATE_SPARATER );
                    //
                    StringAppend ( strStatus, "%4zu\t", s_TRAJECTORY.NumOfPoints());
                    ROS_DEBUG ( "%s" INTERFACE_STATE_SPARATER "%s", strStatus.c_str (), strActions.c_str () );
                    //
                    std_msgs::String output;
                    output.data = strStatus;
                    pInterfaceStates.publish( output );

                    double cost_7 = ros::Time::now().toSec() - start_7;
                    ROS_DEBUG("7. Send Status end. cost_time: %lf ms.", cost_7*1000);
                    //
                    double sleep_time = p_LOOP_TIME - ( ros::Time::now().toSec() - start );
                    if ( sleep_time > 0 )
                    {
                        ros::Duration ( sleep_time ).sleep();
                        ROS_DEBUG("8. Cost Time: %lf ms", (p_LOOP_TIME-sleep_time)*1000);
                    }
                    else
                        ROS_WARN ( "control loop over time: %f s", -sleep_time );
                }
                //---excpetions NO.1 is thrown------------------------------------------
                catch ( Result err )
                {
                    if ( err.code() == ERROR_UNHANDLEABLE_INPUT )
                    {
                        ROS_ERROR ( "recognized error occured: %s. "
                                    "input is ignored.",
                                    err.message() );
                    }
                    else if ( err.code() == ERROR_INVALID_TRAJECTORY )
                    {
                        ROS_ERROR ( "recognized error occured: %s. "
                                    "PMAC program is stoped.",
                                    err.message() );
                        s_MODE = MODE_INIT;
                    }
                    else if ( err.code() == ERROR_INVALID_FEEDBACK )
                    {
                        ROS_ERROR ( "recognized error occured: %s. "
                                    "please TURN OFF the hardware IMMEDIATELY" ,
                                    err.message() );
                    }
                    else if ( err.code() == ERROR_INVALID_STATE )
                    {
                        s_MODE = MODE_STOP;
                        ROS_ERROR ( "Invalid state: %s. "
                                    "elmo is set to RESET command" ,
                                    err.message() );
                    }
                    else
                    {
                        ROS_ERROR ( "recognized error occured: %s. "
                                    "please TURN OFF the hardware IMMEDIATELY",
                                    err.message() );
                    }
                }
            } // the second-level control-loop
            ROS_INFO("second-level control-loop ended");
        }
        //---excpetions NO.2 is thrown - end --------------------------------------------
        catch ( exception ec )
        {
            ROS_ERROR ( "a unhandleable boost exception ourcced. emergency stop.\n"
                        "------------------------------NOTE------------------------------\n"
                        "There is NO tested method to recover the robot from this except-\n"
                        "ion right now.\n "
                        "xc:exception %s",
                        ec.Message() );
        }
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
        //---excpetions NO.2 ended----------------------------------------------------
        //
        subscriber_joint_path.shutdown();
        subscriber_command.shutdown();
        subscriber_grip_command.shutdown();
        subscriber_joint_states.shutdown();
        pInterfaceStates.shutdown();
        pInterfaceArmCmd.shutdown();
        pInterfaceGripCmd.shutdown();
        pInterfaceGripPosCmd.shutdown();

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




