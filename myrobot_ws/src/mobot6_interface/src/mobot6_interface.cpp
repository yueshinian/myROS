#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "std_msgs/String.h"


#include "modbus/modbus.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
//#include <sys/time.h>
#include <boost/algorithm/string.hpp>
#include <boost/system/error_code.hpp>

#include <log4cxx/logger.h>

// -----------the dependent header files from xc-------------
#include <vector>
#include <deque>
#include <cerrno>
#include <cstdarg>
#include <cstring>
#include <stdexcept>
#include <stdint.h>

typedef const char* CStrPtr;
//-----------------------------------------------------------


#define _in   const &   
#define _inc  const     
#define _out        &   

#define _wt         &   
#define _rd   const &   
#define _ret  const     

#define     GMAS_LOOP         0.060       // 0.050
#define     COORDINATOR_LOOP  0.050
#define     STATE_LOOP        0.150       // 0.0375
#define     INTERFACE_LOOP    0.050       // 0.0125

#define     INTERFACE_JOINT_PATH_COMMAND  "/joint_path_command"	
#define     INTERFACE_COMMAND             "/interface_command"
#define     INTERFACE_COMMAND_GRIP        "/interface_command_grip"

#define     INTERFACE_FEEDBACK_STATES     "/feedback_states"
#define     INTERFACE_JOINT_STATES_ARM    "/joint_states_arm"
#define     INTERFACE_STATES              "/interface_states"

#define     INTERFACE_CMD_MOVE            "MOVE"
#define     INTERFACE_CMD_SPLINE          "SPLINE"
#define     INTERFACE_CMD_CLEAR           "CLEAR" // dummy parameter??
#define     INTERFACE_CMD_HALT            "HALT"  // dummy parameter??
#define     INTERFACE_CMD_STOP            "STOP"
#define     INTERFACE_CMD_HOME            "HOME"
#define     INTERFACE_CMD_HOMESTOP        "HOMESTOP"
#define     INTERFACE_CMD_RESET           "RESET"

#define     INTERFACE_STATE_SPARATER      "\t"
#define     INTERFACE_STATE_MOVE          "MOVE"
#define     INTERFACE_STATE_SPLINE        "SPLINE"
#define     INTERFACE_STATE_INIT          "INIT"
#define     INTERFACE_STATE_STOP          "STOP"
#define     INTERFACE_STATE_HOME          "HOME"

#define     INTERFACE_STATE_MOVING        "MOVING"
#define     INTERFACE_STATE_STABLE        "STABLE"

#define     INTERFACE_STATE_DELAYED       "DELAYED"
#define     INTERFACE_STATE_NORMAL        "NORMAL"

#define     JOINT1_ROS_AXIS     "joint_turret_rotate"
#define     JOINT2_ROS_AXIS     "joint_bigarm_pitch"
#define     JOINT3_ROS_AXIS     "joint_forearm_pitch"
#define     JOINT4_ROS_AXIS     "joint_forearm_rotate"
#define     JOINT5_ROS_AXIS     "joint_wrist_pitch"
#define     JOINT6_ROS_AXIS     "joint_wrist_rotate"
#define     JOINT7_ROS_AXIS     "joint_rod_left1"

std::string JOINTS_ROS_AXIS[] = { JOINT1_ROS_AXIS,
                                  JOINT2_ROS_AXIS,
                                  JOINT3_ROS_AXIS,
                                  JOINT4_ROS_AXIS,
                                  JOINT5_ROS_AXIS,
                                  JOINT6_ROS_AXIS,
                                  JOINT7_ROS_AXIS };

int MAP_ROS2GMAS[] = { 1, 5, 4, 3, 2, 0, 6, 7 };

#define     VecName_Axis1       "a01"   // JOINT2
#define     VecName_Axis2       "a03"   // JOINT6
#define     VecName_Axis3       "a04"   // JOINT5
#define     VecName_Axis4       "a05"   // JOINT4
#define     VecName_Axis5       "a06"   // JOINT3
#define     VecName_Axis6       "a08"   // JOINT1
#define     OtherName_Axis1     "a02"
#define     OtherName_Axis2     "a07"   // Linear_Joint

std::string JOINTS_GMAS_AXIS[] = { VecName_Axis1,
                                   VecName_Axis2,
                                   VecName_Axis3,
                                   VecName_Axis4,
                                   VecName_Axis5,
                                   VecName_Axis6,
                                   OtherName_Axis1,
                                   OtherName_Axis2 };

int MAP_GMAS2ROS[] = { 6, 1, 5, 4, 3, 2, 7, 8 };  

#define     HOME_AXIS_ALL       "AXISALL"
#define     HOME_AXIS_1         "AXIS1"
#define     HOME_AXIS_2         "AXIS2"
#define     HOME_AXIS_3         "AXIS3"
#define     HOME_AXIS_4         "AXIS4"
#define     HOME_AXIS_5         "AXIS5"
#define     HOME_AXIS_6         "AXIS6"
#define     HOME_AXIS_7         "AXIS7"

std::string HOME_AXIS[] = { HOME_AXIS_ALL,
                            HOME_AXIS_1,
                            HOME_AXIS_2,
                            HOME_AXIS_3,
                            HOME_AXIS_4,
                            HOME_AXIS_5,
                            HOME_AXIS_6,
                            HOME_AXIS_7 };

#define     LIMIT_FORWARD_GRIP      3
#define     LIMIT_REVERSE_GRIP      -0.01
double      LIMIT_GRIPPER[] = { LIMIT_FORWARD_GRIP, LIMIT_REVERSE_GRIP };

#define         GRIP_FORWARD            "GripForward"
#define         GRIP_REVERSE            "GripReverse"
#define         GRIP_STOP               "GripStop"

#define         GRIP_GMAS_FORWARD       1
#define         GRIP_GMAS_REVERSE       2
#define         GRIP_GMAS_STOP          11

#define MAX_AXES            7
#define COMMAND_POS         0

#define STATE_START_INDEX   30
#define PVT_START_INDEX     150
#define HOME_COMM           2
#define GRIP_CMD            3

#define FUNC_STATE          30
#define MODBUS_RUNNING      31
#define PVT_INIT            32
#define PVT_INDEX           33
#define PVT_MOVE            35
#define SUM_POINTS          36
#define REM_POINTS          38
#define STATE_GROUP         40
#define BUF_REM             41
#define ERROR_FLAG          44
#define PROCESS_STEP        45
#define PROCESS_SUB_STEP    46

#define OpMode_Axis1        51
#define OpMode_Axis2        61
#define OpMode_Axis3        71
#define OpMode_Axis4        81
#define OpMode_Axis5        91
#define OpMode_Axis6        101
#define OpMode_Axis7        111

#define Status_Axis1        52
#define Status_Axis2        62
#define Status_Axis3        72
#define Status_Axis4        82
#define Status_Axis5        92
#define Status_Axis6        102
#define	Status_Axis7        112

#define VelStatus_Axis1     53
#define VelStatus_Axis2     63
#define VelStatus_Axis3     73
#define VelStatus_Axis4     83
#define VelStatus_Axis5     93
#define VelStatus_Axis6     103
#define	VelStatus_Axis7     113

#define Pos_Axis1           54
#define Pos_Axis2           64
#define Pos_Axis3           74
#define Pos_Axis4           84
#define Pos_Axis5           94
#define Pos_Axis6           104
#define	Pos_Axis7           114

#define Vel_Axis1           56
#define Vel_Axis2           66
#define Vel_Axis3           76
#define Vel_Axis4           86
#define Vel_Axis5           96
#define Vel_Axis6           106
#define	Vel_Axis7           116

int Map_OpMode[]    = { OpMode_Axis1, OpMode_Axis2, OpMode_Axis3, OpMode_Axis4,
                        OpMode_Axis5, OpMode_Axis6, OpMode_Axis7 };
int Map_Status[]    = { Status_Axis1, Status_Axis2, Status_Axis3, Status_Axis4,
                        Status_Axis5, Status_Axis6, Status_Axis7 };
int Map_VelStatus[] = { VelStatus_Axis1, VelStatus_Axis2, VelStatus_Axis3, VelStatus_Axis4,
                        VelStatus_Axis5, VelStatus_Axis6, VelStatus_Axis7 }; // dummy
int Map_Position[]  = { Pos_Axis1, Pos_Axis2, Pos_Axis3, Pos_Axis4, Pos_Axis5, Pos_Axis6, Pos_Axis7 };
int Map_Velocity[]  = { Vel_Axis1, Vel_Axis2, Vel_Axis3, Vel_Axis4, Vel_Axis5, Vel_Axis6, Vel_Axis7 }; // dummy

const int MODE_INIT   = 0;
const int MODE_MOVE   = 1;
const int MODE_STOP   = 2;
const int MODE_HOME   = 3;
const int MODE_SPLINE = 4;

const int ERROR_UNHANDLEABLE_INPUT = 1;
const int ERROR_INVALID_TRAJECTORY = 2;
const int ERROR_INVALID_FEEDBACK = 3;

const int ERROR_INVALID_STATE = 4;

bool stop_flag = 1; // 0

modbus_t    *mb;


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

        inline std::vector<int> _ret PopFront( double _in ratio,
                                               size_t _in except = 99,
                                               double _in ratio_except = 100.0 )
        {
            std::vector<int> ret( NumOfAxis() );
            for (size_t i = 0; i < NumOfAxis(); ++i)
            {
                if (i != except)  // the usual joint type and ratio
                    ret[i] = (int)(mData[i].front()*ratio);
                else  // the excepted joint type and ratio
                    ret[i] = (int)(mData[i].front()*ratio_except);
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
        {
            return res;
        }

        // copy and check time step
        outputPOS.Reset(mAxisGMAS, trajectory.points.size(), mTimeStep); // trajectory.points.size()-1
        for (size_t p = 0; p < trajectory.points.size(); ++p)
        {
            trajectory_msgs::JointTrajectoryPoint pnt = trajectory.points[p];

            for (size_t i = 0; i < NumOfAxis(); ++i)
            {
                outputPOS.Set(ros2trj[i], p, pnt.positions[i]);
            }
        }

        // return
        return Result::Success();
    }


protected:
        std::vector<std::string> mAxisROS;
        std::vector<std::string> mAxisGMAS;
        double                   mTimeStep;
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


void InsertLongVarToShortArr(uint16_t* spArr, long lVal)
{
        *spArr      = (uint16_t)(lVal & 0xFFFF);
        *(spArr+1)  = (uint16_t)((lVal>>16) & 0xFFFF);
}

int _ret WriteModbus_PVT (int _in reg_pos,
                          std::vector<int> _in pvt,
                          int _in pvt_num,
                          int _in axis_num,
                          double _in wait_time = 0.05)
{
    int count=0;
    int rem_pnt = (2*axis_num+2)*pvt_num;
    while (rem_pnt > 0)
    {
        int test = ReadModbus_Long(PVT_START_INDEX);
        int test1 = ReadModbus_Short(ERROR_FLAG);
        while (test != 0)
        {
            ROS_INFO("WriteModbus_PVT Marker: wait for the modbus_read process in GMAS. pvt_start_index[%d], Error_flag[%d].",
                     test, test1);
            ros::Duration(wait_time).sleep();
            test = ReadModbus_Long(PVT_START_INDEX);
        }

        int reg_num = rem_pnt>112 ? 112 : rem_pnt;
        uint16_t reg_table[reg_num+2]; // = {0}

        InsertLongVarToShortArr(&reg_table[0], (long)(reg_num/(2*axis_num+2)));
        for ( int i = 1; i < (2+reg_num)/2; ++i )
            InsertLongVarToShortArr(&reg_table[2*i], (long)pvt[56*count+i-1]);
        int regs = modbus_write_registers(mb, reg_pos, reg_num+2, reg_table);

        if (regs == -1)
        {
            ROS_WARN("ERROR:%s: Write Modbus Fail.", __func__);
            return -111111;
        }
        count++;
        rem_pnt = rem_pnt - reg_num;
    }
    return 1;
}


int main ( int argc, char **argv )
{
    ROS_INFO ( "initializing ROS" );
    ros::init ( argc, argv, "mobot6_interface" );
    ros::NodeHandle NODE;
    ROS_INFO ( "initializing ROS succeed" );
    bool    p_RReset = false;
    ros::Duration(5.0).sleep();

    while ( ros::ok() )
    {
        ROS_INFO ( "mian-loop begins" );
        //
        ROS_INFO ( "setting parameters" );
        //
        // set the loop time, IP address and port address
        const double            p_GMAS_TIME              = GMAS_LOOP;
        const double            p_LOOP_TIME              = INTERFACE_LOOP;
        const double            p_FEEDBACK_TIME          = STATE_LOOP;

        const char              p_ROBOT_IP[]             = "192.168.1.3";
        int                     p_ROBOT_PORT             = 502;

        TrajectoryProcessor     TRAJ_PROCESSOR ( p_GMAS_TIME );
        TRAJ_PROCESSOR.AddAxis ( JOINTS_ROS_AXIS[MAP_ROS2GMAS[0]], JOINTS_GMAS_AXIS[0] );
        TRAJ_PROCESSOR.AddAxis ( JOINTS_ROS_AXIS[MAP_ROS2GMAS[1]], JOINTS_GMAS_AXIS[1] );
        TRAJ_PROCESSOR.AddAxis ( JOINTS_ROS_AXIS[MAP_ROS2GMAS[2]], JOINTS_GMAS_AXIS[2] );
        TRAJ_PROCESSOR.AddAxis ( JOINTS_ROS_AXIS[MAP_ROS2GMAS[3]], JOINTS_GMAS_AXIS[3] );
        TRAJ_PROCESSOR.AddAxis ( JOINTS_ROS_AXIS[MAP_ROS2GMAS[4]], JOINTS_GMAS_AXIS[4] );
        TRAJ_PROCESSOR.AddAxis ( JOINTS_ROS_AXIS[MAP_ROS2GMAS[5]], JOINTS_GMAS_AXIS[5] );
        //
        const double            p_MinimalInputTime     = 1.0;
        const size_t            p_MinimalStartSize     = 7;
        const size_t            p_MinimalSendSize      = 7;
        //
        ROS_INFO ( "setting parameters succeed" );
        //
        // init the subscribers and publishers
        ROS_INFO ( "initializing subscriber and publisher" );
        JointTrajectoryHandller TRAJECTORY_HANDLLER;
        StringHandller          COMMAND_HANDLLER;
        StringHandller          GRIP_COMMAND_HANDLLER;

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

        ros::Publisher  pFeedbackStates =
                NODE.advertise<control_msgs::FollowJointTrajectoryFeedback> (
                    INTERFACE_FEEDBACK_STATES, 1024 );
        ros::Publisher  pJointStates =
                NODE.advertise<sensor_msgs::JointState> (
                    INTERFACE_JOINT_STATES_ARM, 1024 );
        ros::Publisher  pInterfaceStates =
                NODE.advertise<std_msgs::String> (
                    INTERFACE_STATES, 1024 );
        ROS_INFO ( "initializing subscriber and publisher succeed" );
        //
        ROS_INFO ( "connecting to GMAS" );

        mb = modbus_new_tcp(p_ROBOT_IP, p_ROBOT_PORT);  
        modbus_set_slave(mb, 1);  

        uint32_t    tv_sec = 0;
        uint32_t    tv_usec = 500000; 

        modbus_set_response_timeout(mb, tv_sec, tv_usec);

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
        //---excpetions NO.2 is thrown - begin ------------------------------------------
        try
        {
            int iCnt = 0;
            if (stop_flag)
            {
                int unload_resp = ReadModbus_Long(BUF_REM);
                if (unload_resp != 0)
                {
                    // conduct the GMAS_unloaded and Buffer_cleared sub-function (no.34)
                    int resp = WriteModbus_Short (COMMAND_POS, 34); // Refer to StateFunction_34
                    if (resp > 0)
                        ROS_INFO("Write the GMAS_Unload process successfully.");
                    else
                        ROS_ERROR("Error: Write the GMAS_Unload process failed.");

                    bool check_34 = false;
                    int regs = ReadModbus_Short(BUF_REM);
                    if((regs!=0) && (regs!=1))
                    {
                        check_34 = true;
                    }
                    iCnt = 0;
                    while( check_34 )
                    {
                        int regs = ReadModbus_Short(BUF_REM);
                        if((regs!=0) && (regs!=1))
                            check_34 = true;
                        else
                            check_34 = false;
                        ROS_WARN("Error: GMAS Buffer has not been cleared (#34). BUF_NUM[%d].", regs);
                        ros::Duration( 0.2 ).sleep();
                        iCnt++;
                        if (iCnt >= 50)
                        {
                            ROS_ERROR("Error: Break the loop without clearing the buffer (#34). BUF_NUM[%d].", regs);
                            break;
                        }
                    }
                }
                int regs = ReadModbus_Short (STATE_GROUP);
                if ( (regs==1) || (regs==2) || (regs==3) || (regs==5) || (regs==6) )
                {
                    ROS_ERROR("[#34-#31] current cVector state(*40) is [%d]", regs);
                    int resp = WriteModbus_Short (COMMAND_POS, 31); // Refer to StateFunction_31
                    if (resp > 0)
                        ROS_INFO("Write the stop_flag process successfully.");
                    else
                    {
                        ROS_WARN("Error: modbus write for Motor_Init process failed.");
                        for ( size_t i = 3; i > 0; --i )
                        {
                            ROS_INFO ( "reset the motors in %lu seconds.", i );
                            ros::Duration(1.0).sleep();
                        }
                        p_RReset = true;
                        stop_flag = 1;
                    }
                }
                else
                {
                    ROS_ERROR("[#31]Error: the current group status is not available for stopping.");
                    ROS_ERROR("[#34-#31] current cVector state(*40) is [%d]", regs);
                }

                iCnt = 0;
                if( !p_RReset )
                {
                    bool check_31 = false;
                    if ((ReadModbus_Short(COMMAND_POS)!=0) || (ReadModbus_Short(FUNC_STATE)!=2))
                    {
                        check_31 = true;
                    }
                    while( check_31 ) 
                    {
                        int reg_cmd_pos = ReadModbus_Short(COMMAND_POS);
                        int reg_func_state = ReadModbus_Short(FUNC_STATE);
                        int reg_pro_step = ReadModbus_Short(PROCESS_STEP);
                        int reg_sub_step = ReadModbus_Short(PROCESS_SUB_STEP);
                        if ((ReadModbus_Short(COMMAND_POS)!=0) || (ReadModbus_Short(FUNC_STATE)!=2))
                            check_31 = true;
                        else
                            check_31 = false;
                        ROS_INFO("Wait for state-function 31. COMMAND_POS[%d], FUNC_STATE[%d].", reg_cmd_pos, reg_func_state);
                        ROS_WARN("Wait for state-function 31. PROCESS_STEP[%d], SUB_STEP[%d].", reg_pro_step, reg_sub_step);
                        ros::Duration( 0.2 ).sleep();
                        //
                        iCnt++;
                        if (iCnt >= 50)
                        {
                            p_RReset = true;
                            iCnt = 0;
                            break;
                        }
                    }
                    // determine wether the state_function has been blocked
                    if( p_RReset )
                    {
                        stop_flag = 1;
                        ROS_ERROR("The state_function 31 has been blocked, re-reset the vehicle.");
                    }
                    else if( !p_RReset )
                    {
                        stop_flag = 0;
                        ROS_INFO("motor killed ..., stop_flag[%d].", stop_flag);
                    }
                }
            }
            //
            iCnt = 0;
            int check_count = 0;
            for (size_t i = 0; i < TRAJ_PROCESSOR.NumOfAxis()+1; ++i) // total
            {
                int regs1 = ReadModbus_Short(Map_OpMode[i]);
                int regs2 = ReadModbus_Short(Map_Status[i]);
                while (((regs1 != 1) || (regs2 != 1)) && (!p_RReset))
                {
                    regs1 = ReadModbus_Short(Map_OpMode[i]);
                    regs2 = ReadModbus_Short(Map_Status[i]);
                    ROS_ERROR("Axis [a%02d] is in OPMode[%d] and in Status[%d].\n", i+1, regs1, regs2);
                    ros::Duration(0.2).sleep();
                    // skip out the blocked cyclication
                    check_count++;
                    if(check_count >= 50)
                    {
                        p_RReset = true;
                        check_count = 0;
                        break;
                    }
                }
                // the normal condition that everything is ok
                if( !p_RReset )
                {
                    ROS_INFO("Axis [a%02d] is in SYNC_POSITION Op-Mode[%d] and in STAND_STILL status[%d].\n", i+1, regs1, regs2);
                    iCnt++;
                }
                // skip out the blocked cyclication
                else if( p_RReset )
                {
                    break;
                }
            }
            if (iCnt == TRAJ_PROCESSOR.NumOfAxis()+1) // total
            {
                ROS_INFO("Initializing GMAS succeed.");
            }
            //
            int s_MODE  = MODE_INIT;
            if( p_RReset )
            {
                s_MODE = MODE_STOP;
                p_RReset = false;
            }

            ROS_INFO ( "starting control-loop" );
            // trajectory
            PTTrajectory            s_TRAJECTORY             = TRAJ_PROCESSOR.MakeTrajectory();
            double                  s_LAST_INPUT_TIME        = 0.0;
            // feecback
            double                  s_LAST_FEEDBACK_TIME     = 0.0;
            std::vector<double>     s_LAST_FEEDBACK ( TRAJ_PROCESSOR.NumOfAxis()+1, 0.0 );
            // runtime estimation
            int                     s_REMAINING_COMMAND_SIZE = 0;
            double                  s_EXPECTED_FINISH_TIME   = ros::Time::now ().toSec ();
            bool                    s_MOVING                 = false;

            bool                    s_GRIP_MOVING            = false;
            int                     s_CURRENT_GRIP_CMD       = 0;
            int                     s_LAST_GRIP_CMD          = 0;
            int                     s_REFRESH_COUNT          = 0;
            // homing
            bool                    s_HOME_CHECKMODE         = false;
            int                     s_HOME_COUNT             = 0;
            int                     s_HOME_LAST_AXIS         = 0;
            bool                    s_HOME_LAST_AXIS_FLAG    = true;
            // interface buffer control
            int                     s_TrajDelayThreshold     = 5;
            int                     s_NOTRAJ_COUNT           = 0;
            int                     s_DELAY_COUNT            = 0;
            bool                    s_DELAY_FLAG             = false;
            // Actions and Status string
            std::string             strActions;
            std::string             strStatus;
            strActions.reserve ( 256 );
            strStatus.reserve ( 256 );
            // init the ros2trj mapping
            std::vector<size_t> ros2trj(TRAJ_PROCESSOR.NumOfAxis(), 0);
            for (size_t i = 0; i < TRAJ_PROCESSOR.NumOfAxis(); ++i)
                ros2trj[i] = i;
            //--------------------------------------------------------------------------
            ROS_INFO ( "control loop started" );
            while( ros::ok() && s_MODE != MODE_STOP )
            {
                strActions.clear();
                double start = ros::Time::now().toSec();
                // limitation flag of gripper
                bool f_grip_limitation;
                bool r_grip_limitaion;
                int iCnt;
                //
                s_HOME_LAST_AXIS_FLAG = true;

                //---excpetions NO.1 is thrown------------------------------------------
                try
                {
                    bool communicated = false;
                    //---spin-----------------------------------------------------------
                    ros::spinOnce();
                    //
                    double start_2 = ros::Time::now().toSec();
                    if (s_MODE == MODE_HOME)
                    {
                        if (s_HOME_COUNT <= 20)
                        {
                            iCnt = 0;
                            for (size_t i = 0; i < TRAJ_PROCESSOR.NumOfAxis()+1; ++i) // total
                            {
                                int resp = ReadModbus_Short(Map_Status[i]);
                                if ( resp == 1 ) // "STAND_STILL"
                                    iCnt++;
                            }

                            // if all axes are in STAND_STILL, add one to s_HOME_COUNT, otherwise clear s_HOME_COUNT
                            if ( iCnt == TRAJ_PROCESSOR.NumOfAxis()+1 ) // total
                                s_HOME_COUNT++;
                            else
                                s_HOME_COUNT = 0;
                        }

                        // if the s_HOME_COUNT is more than 20, enter the Mode_Switch process
                        else
                        {
                            s_HOME_COUNT = 0;
                            int resp = WriteModbus_Long (COMMAND_POS, 35);  // #35 is similar to #22 in vehicle
                            if (resp > 0)
                                ROS_INFO("[Important] Write the Mode_Switch(NO.35) process successfully.");
                            else
                                ROS_ERROR("Error: modbus write for Mode_Switch(NO.35) process failed.");

                            int count = 0;
                            bool check_35 = false;
                            if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                            {
                                check_35 = true;
                            }
                            while( check_35 )
                            {
                                count++;
                                if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                    check_35 = true;
                                else
                                    check_35 = false;
                                ros::Duration( 0.2 ).sleep();

                                if (count == 50)
				{
                                    int reg_cmd_pos = ReadModbus_Short(COMMAND_POS);
                                    int reg_func_state = ReadModbus_Short(FUNC_STATE);
                                    int reg_proc_step = ReadModbus_Short(PROCESS_STEP);
                                    ROS_ERROR("[#35] Error: cannot loop out. CMD_POS[%d], FUNC_STATE[%d], PROCESS_STEP[%d]", reg_cmd_pos, reg_func_state, reg_proc_step);
                                    throw Result ( ERROR_INVALID_STATE, "invalid home mode switch (#35)" );
				}
                            }
                            s_HOME_CHECKMODE = true;
                        }

                        // check the operation mode of each axis, then determine the interface mode
                        if ( s_HOME_CHECKMODE )
                        {
                            iCnt = 0;
                            for (size_t i = 0; i < TRAJ_PROCESSOR.NumOfAxis()+1; ++i) // total
                            {
                                int resp = ReadModbus_Short(Map_OpMode[i]);
                                if ( resp == 1 ) // "CYCLIC POSITION"
                                    iCnt++;
                            }
                            if (iCnt == TRAJ_PROCESSOR.NumOfAxis()+1) // total
                            {
                                s_MODE = MODE_INIT;
                                s_HOME_LAST_AXIS = 0;
                                s_HOME_CHECKMODE = false;
                            }
                        }
                    }
                    double cost_2 = ros::Time::now().toSec() - start_2;
                    ROS_DEBUG("2. MODE_SWITCH process end. cost_time: %lf ms.", cost_2*1000);
                    //
                    double start_3 = ros::Time::now().toSec();

                    if ( COMMAND_HANDLLER.newMsg() )
                    {
                        std_msgs::String input_ = COMMAND_HANDLLER.fetchMsg();
                        std::string input = input_.data;
                        boost::to_upper ( input );
                        std::cout << "get input " <<input << std::endl;

                        std::vector<std::string> result;
                        boost::split( result, input, boost::is_any_of(" "), boost::algorithm::token_compress_on );
                        std::string COMMAND = result[0];
                        std::string HOMEAXIS = "";
                        if ((COMMAND == INTERFACE_CMD_HOME) && (result.size() == 2))
                            HOMEAXIS = result[1];

                        StringAppend ( strActions, "1. %s\t", input.c_str () );
                        //
                        if ( s_MODE == MODE_MOVE )
                        {
                            if (COMMAND == INTERFACE_CMD_STOP)
                            {
                                s_MODE = MODE_STOP;
                                stop_flag = 1;
                            }
                            else if (COMMAND==INTERFACE_CMD_SPLINE && !s_MOVING)
                            {
                                s_MODE = MODE_SPLINE;
                                s_TRAJECTORY.clear();
                                TRAJECTORY_HANDLLER.clear();
                            }
                            else if (COMMAND == INTERFACE_CMD_CLEAR)
                            {
                                s_TRAJECTORY.clear();
                            }
                        }
                        //
                        else if ( s_MODE == MODE_SPLINE )
                        {
                            if (COMMAND == INTERFACE_CMD_STOP)
                            {
                                s_MODE = MODE_STOP;
                                stop_flag = 1;
                            }
                            else if (COMMAND==INTERFACE_CMD_MOVE && !s_MOVING)
                            {
                                s_MODE = MODE_MOVE;
                                s_TRAJECTORY.clear();
                                TRAJECTORY_HANDLLER.clear();
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
                                stop_flag = 1;
                                ROS_WARN("INIT STATE: STOP COMMAND");
                            }
                            else if ( COMMAND==INTERFACE_CMD_MOVE || COMMAND==INTERFACE_CMD_SPLINE )
                            {
                                s_TRAJECTORY.clear();
                                TRAJECTORY_HANDLLER.clear();

                                int resp = WriteModbus_Short (COMMAND_POS, 32);
                                if (resp > 0)
                                    ROS_INFO("Write the Move_Init process successfully.");
                                else
                                    ROS_ERROR("Error: modbus write for Move_Init process failed.");

                                // wait until the StateFunction has been conducted
                                iCnt = 0;
                                bool check_32 = false;
                                if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                {
                                    check_32 = true;
                                }
                                while( check_32 )
                                {
                                    int regs1 = ReadModbus_Short(COMMAND_POS);
                                    int regs2 = ReadModbus_Short(FUNC_STATE);
                                    if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                        check_32 = true;
                                    else
                                        check_32 = false;
                                    ROS_INFO("Wait for the end of the sub-function 32. COMMAND_POS[%d], FUNC_STATE[%d].", regs1, regs2);
                                    ros::Duration( 0.2 ).sleep();
                                }
                                // check the motors
                                iCnt = 0;
                                while(ReadModbus_Short(STATE_GROUP) != 1)   // Group Status: STANDBY (no.40)
                                {
                                    iCnt++;
                                    ROS_ERROR("Error: Group Status is not available for MOVE MODE.");
                                    ros::Duration( 0.2 ).sleep();
                                    if (iCnt == 25)
                                        throw Result ( ERROR_INVALID_STATE,
                                                       "invalid group state for move mode switch (#32)" );
                                }

                                iCnt = 0;
                                for (size_t i = 0; i < TRAJ_PROCESSOR.NumOfAxis()+1; ++i) // total
                                {
                                    iCnt = 0;
                                    while(ReadModbus_Short(Map_Status[i]) != 1) // Axis Status: STAND_STILL (no.52 & no.62)
                                    {
                                        iCnt++;
                                        ROS_ERROR("Error: Axis Status [a%02d] is not available for MOVE MODE.", i+1);
                                        if (iCnt == 50)
                                            throw Result ( ERROR_INVALID_STATE,
                                                           "invalid axis state for move mode switch (#32)" );
                                    }
                                    iCnt = 0;
                                    while(ReadModbus_Short(Map_OpMode[i]) != 1) // Axis OpMode: SYNC_POSITION (no.51 & no.61)
                                    {
                                        iCnt++;
                                        ROS_ERROR("Error: Axis OpMode [a%02d] is not available for MOVE MODE.", i+1);
                                        if (iCnt == 50)
                                            throw Result ( ERROR_INVALID_STATE,
                                                           "invalid operation mode for move mode switch (#32)" );
                                    }
                                }
                                ROS_INFO("enable motor succeed");

                                // set the "s_MODE" to "MODE_MOVE" or "MODE_SPLINE"
                                if (COMMAND == INTERFACE_CMD_MOVE)
                                    s_MODE = MODE_MOVE;
                                else if (COMMAND == INTERFACE_CMD_SPLINE)
                                    s_MODE = MODE_SPLINE;
                                else
                                    ROS_ERROR("the command is neither MOVE nor SPLINE");
                            }
                            else if ( COMMAND == INTERFACE_CMD_HOME )
                            {
                                // conduct the Motor_Home process
                                s_HOME_COUNT = 0;
                                // for each axis condition && all axes condition, write the home_communicated bit
                                for(size_t i = 0; i <= TRAJ_PROCESSOR.NumOfAxis()+1; ++i) // total
                                {
                                    if ((HOMEAXIS == HOME_AXIS[i]) && (i != 0))
                                    {
                                        s_HOME_LAST_AXIS = MAP_GMAS2ROS[i-1];
                                        int resp = WriteModbus_Short (HOME_COMM, MAP_GMAS2ROS[i-1]);
                                        ROS_ERROR("[Home_Process #36] responding axis: %d.", MAP_GMAS2ROS[i-1]);
                                        if (resp > 0)
                                            ROS_INFO("Write the home_communicated bit successfully.");
                                        else
                                            ROS_WARN("Error: modbus write for home_communicated bit failed.");
                                    }
                                    else if((HOMEAXIS == HOME_AXIS[i]) && (i == 0))
                                    {
                                        s_HOME_LAST_AXIS = 11;
                                        int resp = WriteModbus_Short (HOME_COMM, 11);   // if the all axes condition, write 11 to modbus
                                        if (resp > 0)
                                            ROS_INFO("Write the home_communicated bit successfully.");
                                        else
                                            ROS_WARN("Error: modbus write for home_communicated bit failed.");
                                    }
                                }
                                //
                                int resp = WriteModbus_Long (COMMAND_POS, 36); 
                                if (resp > 0)
                                    ROS_INFO("[Important] Write the Motor_Home(NO.36) process successfully.[%d]", resp);
                                else
                                    ROS_ERROR("Error: modbus write for Motor_Home process failed.");

                                // wait until the StateFunction_36 has been conducted
                                iCnt = 0;
                                bool check_36 = false;
                                if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2) || (ReadModbus_Short(HOME_COMM) != 0))
                                {
                                    check_36 = true;
                                }
                                while( check_36 )
                                {
                                    int regs1 = ReadModbus_Short(COMMAND_POS);
                                    int regs2 = ReadModbus_Short(HOME_COMM);
                                    int regs3 = ReadModbus_Short(FUNC_STATE);
                                    int regs4 = ReadModbus_Short(PROCESS_STEP);
                                    int regs5 = ReadModbus_Short(PROCESS_SUB_STEP);
                                    if ((regs1 != 0) || (regs2 != 0) || (regs3 != 2))
                                        check_36 = true;
                                    else
                                        check_36 = false;
                                    ROS_INFO("[aaaaa] Wait for state_function 36. cmd_pos[%d], home_cmd[%d], fun_state[%d]", regs1, regs2, regs3);
                                    ROS_WARN("[aaaaa] Wait for state_function 36. process_step[%d], process_sub_step[%d]", regs4, regs5);
                                    ros::Duration( 0.2 ).sleep();
                                    iCnt++;
                                    if (iCnt >= 50)
                                        break;
                                }
                                if (iCnt >= 50)
                                {
                                    s_MODE = MODE_STOP;
                                    stop_flag = 1;
                                }
                                else
                                {
                                    s_MODE = MODE_HOME;
                                }
                            }
                        }
                        //
                        else if ( s_MODE == MODE_HOME )
                        {
                            if ( COMMAND == INTERFACE_CMD_HOMESTOP )
                            {
                                int resp = WriteModbus_Long (COMMAND_POS, 37); 
                                if (resp > 0)
                                    ROS_INFO("[Important] Write the Home_Stop(NO.37) process successfully.");
                                else
                                    ROS_ERROR("Error: modbus write for Home_Stop process failed.");

                                // wait until the StateFunction_37 has been conducted
                                iCnt = 0;
                                bool check_37 = false;
                                if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                {
                                    check_37 = true;
                                }
                                while( check_37 )
                                {
                                    int regs1 = ReadModbus_Short(COMMAND_POS);
                                    int regs2 = ReadModbus_Short(FUNC_STATE);
                                    if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                        check_37 = true;
                                    else
                                        check_37 = false;
                                    ROS_INFO("Wait for the end of the state_function 37. cmd_pos[%d], fun_state[%d]", regs1, regs2);
                                    ros::Duration( 0.2 ).sleep();
                                }
                            }
                            else if ( COMMAND == INTERFACE_CMD_HOME )
                            {
                                // conduct the Motor_Home process
                                s_HOME_COUNT = 0;
                                // for each axis condition && all axes condition, write the home_communicated bit
                                for(size_t i = 0; i <= TRAJ_PROCESSOR.NumOfAxis()+1; ++i) // total
                                {
                                    if ((HOMEAXIS == HOME_AXIS[i]) && (i != 0))
                                    {
                                        if (s_HOME_LAST_AXIS != MAP_GMAS2ROS[i-1])
                                        {
                                            s_HOME_LAST_AXIS = MAP_GMAS2ROS[i-1];
                                            ROS_ERROR("[Home_Process #36] responding axis: %d.", MAP_GMAS2ROS[i-1]);
                                            int resp = WriteModbus_Short (HOME_COMM, MAP_GMAS2ROS[i-1]);
                                            if (resp > 0)
                                                ROS_INFO("Write the home_communicated bit successfully.");
                                            else
                                                ROS_WARN("Error: modbus write for home_communicated bit failed.");
                                        }
                                        else
                                        {
                                            s_HOME_LAST_AXIS_FLAG = false;
                                            ROS_ERROR("[Home_Process] too fast homing command [%d].", s_HOME_LAST_AXIS);
                                        }
                                    }
                                    else if((HOMEAXIS == HOME_AXIS[i]) && (i == 0))
                                    {
                                        if (s_HOME_LAST_AXIS != 11)
                                        {
                                            s_HOME_LAST_AXIS = 11;
                                            int resp = WriteModbus_Short (HOME_COMM, 11);   // if the all axes condition, write 11 to modbus
                                            if (resp > 0)
                                                ROS_INFO("Write the home_communicated bit successfully.");
                                            else
                                                ROS_WARN("Error: modbus write for home_communicated bit failed.");
                                        }
                                        else
                                        {
                                            s_HOME_LAST_AXIS_FLAG = false;
                                            ROS_ERROR("[Home_Process] too fast homing command [%d].", s_HOME_LAST_AXIS);
                                        }
                                    }
                                }
                                //
                                if( s_HOME_LAST_AXIS_FLAG )
                                {
                                    int resp = WriteModbus_Long (COMMAND_POS, 36); 
                                    if (resp > 0)
                                        ROS_INFO("[Important] Write the Motor_Home(NO.36) process successfully.");
                                    else
                                        ROS_ERROR("Error: modbus write for Motor_Home process failed.");

                                    // wait until the StateFunction_36 has been conducted
                                    iCnt = 0;
                                    bool check_36_1 = false;
                                    if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2) || (ReadModbus_Short(HOME_COMM) != 0))
                                    {
                                        check_36_1 = true;
                                    }
                                    while( check_36_1 )
                                    {
                                        int regs1 = ReadModbus_Short(COMMAND_POS);
                                        int regs2 = ReadModbus_Short(HOME_COMM);
                                        int regs3 = ReadModbus_Short(FUNC_STATE);
                                        if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2) || (ReadModbus_Short(HOME_COMM) != 0))
                                            check_36_1 = true;
                                        else
                                            check_36_1 = false;
                                        ROS_INFO("[bbbbb] Wait for the end of the state_function 36. cmd_pos[%d], home_cmd[%d], fun_state[%d]", regs1, regs2, regs3);
                                        ros::Duration( 0.2 ).sleep();
                                    }
                                }
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
                        if ( s_MODE==MODE_MOVE || s_MODE==MODE_SPLINE )
                        {
                            PTTrajectory output = TRAJ_PROCESSOR.MakeTrajectory();
                            if ( s_MODE == MODE_MOVE )
                            {
                                Result ec = TRAJ_PROCESSOR.TranslateDirect ( input, output, ros2trj );
                                if ( ec.is_failed() )
                                    throw ec;
                            }
                            else if ( s_MODE == MODE_SPLINE )
                            {
                                Result ec = TRAJ_PROCESSOR.TranslateDirect ( input, output, ros2trj );
                                if ( ec.is_failed() )
                                    throw ec;
                                if ( s_TRAJECTORY.NumOfPoints () > 0 )
                                    throw Result( ERROR_UNHANDLEABLE_INPUT,
                                                  "invalid trajectory for auto model: platform is moving" );
                                if ( output.NumOfPoints() < p_MinimalStartSize )
                                    throw Result( ERROR_UNHANDLEABLE_INPUT,
                                                  "invalid trajectory for auto model: not enough points" );
                            }
//-------------------------------------------------------------------------------------------------------------------------------
                            std::vector<double> output_test = output.GetPoint(0);
                            std::cout << "output test:" << std::endl;
                            for (int i=0; i<output_test.size(); ++i)
                            {
                                std::cout << output_test[i] << " ";
                            }
                            std::cout << std::endl;
//-------------------------------------------------------------------------------------------------------------------------------

                            s_TRAJECTORY.Append ( output );
                            s_LAST_INPUT_TIME = ros::Time::now().toSec();
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

                    if ( s_MODE != MODE_STOP )
                    {
                        if ( ros::Time::now().toSec() + p_LOOP_TIME - s_LAST_FEEDBACK_TIME - p_FEEDBACK_TIME >= 0 )
                        {
                            StringAppend ( strActions, "3. new_trajectory\t" );
                            communicated = true;

                            std::vector<int> feedback_reply ( TRAJ_PROCESSOR.NumOfAxis()+1 ); //total
                            std::vector<double> feedback ( TRAJ_PROCESSOR.NumOfAxis()+1 ); //total
                            std::vector<std::string> feedback_name ( TRAJ_PROCESSOR.NumOfAxis()+1 ); //total
                            std::vector<double> velocity ( TRAJ_PROCESSOR.NumOfAxis()+1 ); //total

                            // read the position feedback from modbus, in the GMAS sequence
                            for (size_t i = 0; i < TRAJ_PROCESSOR.NumOfAxis()+1; ++i) // total
                            {
                                feedback_reply[i] = ReadModbus_Long(Map_Position[i]);
                                if (feedback_reply[i] == -111111)
                                {
                                    ROS_ERROR("Error: Axis feedback position [a%02d] is not available now.", i+1);
                                    throw Result ( ERROR_INVALID_FEEDBACK, "can not get feedback data" );
                                }
                            }
                            // fill in the feedback vector: feedback[], feedback_name[] and velocity[]
                            for (size_t i = 0; i < TRAJ_PROCESSOR.NumOfAxis()+1; ++i) // total
                            {
                                try
                                {
                                    feedback[MAP_ROS2GMAS[i]] = ((double)feedback_reply[i])/10000.0/180.0*3.1415926;  // 100.0
                                    feedback_name[MAP_ROS2GMAS[i]] = JOINTS_ROS_AXIS[MAP_ROS2GMAS[i]];
                                    velocity[MAP_ROS2GMAS[i]] = (feedback[MAP_ROS2GMAS[i]] - s_LAST_FEEDBACK[MAP_ROS2GMAS[i]])/
                                            (ros::Time::now().toSec() - s_LAST_FEEDBACK_TIME);
                                }
                                catch ( ... )
                                {
                                    throw Result ( ERROR_INVALID_FEEDBACK, "can not retrieve feedback data" );
                                }
                            }
                            //
                            sensor_msgs::JointState jointState;
                            jointState.header.stamp = ros::Time::now();
                            jointState.name = feedback_name;
                            jointState.position = feedback;
                            jointState.velocity = velocity;
                            pJointStates.publish ( jointState );
                            //
                            // publish feedback ("/feedback_states")
                            control_msgs::FollowJointTrajectoryFeedback follow_joint_trajectory_feedback;
                            follow_joint_trajectory_feedback.header.stamp = ros::Time::now();
                            follow_joint_trajectory_feedback.joint_names = feedback_name;
                            follow_joint_trajectory_feedback.actual.positions = feedback;
                            pFeedbackStates.publish ( follow_joint_trajectory_feedback );
                            //
                            s_LAST_FEEDBACK_TIME = ros::Time::now().toSec();
                            s_LAST_FEEDBACK = feedback;
                            //
                            if( feedback[6] >= LIMIT_GRIPPER[0] ) // the forward limitaion of gripper
                            {
                                f_grip_limitation = true;
                                r_grip_limitaion  = false;
                            }
                            else if( feedback[6] <= LIMIT_GRIPPER[1] ) // the reverse limitation of gripper
                            {
                                f_grip_limitation = false;
                                r_grip_limitaion  = true;
                            }
                            else if((feedback[6]<LIMIT_GRIPPER[0]) && (feedback[6]>LIMIT_GRIPPER[1]))
                            {
                                f_grip_limitation = false;
                                r_grip_limitaion  = false;
                            }
                            // ROS_INFO("forward_limit[%d], reverse_limit[%d]", f_grip_limitation, r_grip_limitaion);
                        }
                    }
                    //  if the current state has been changed to "MODE_STOP"
                    else
                    {
                        s_LAST_FEEDBACK_TIME = 0.0;
                        std::fill ( s_LAST_FEEDBACK.begin (), s_LAST_FEEDBACK.end (), 0.0 );
                    }

                    double cost_5 = ros::Time::now().toSec() - start_5;
                    ROS_DEBUG("5. TRAJECTORY_FEEDBACK end. cost_time: %lf ms.", cost_5*1000);
                    //
                    double start_6 = ros::Time::now().toSec();

                    if (s_MODE==MODE_MOVE || s_MODE==MODE_SPLINE)
                    {
                        if (s_DELAY_COUNT >= 2)
                        {
                            if (s_MODE == MODE_MOVE)
                            {
                                ROS_ERROR("[Trajectory Clear] buffer_num[%zu].", s_TRAJECTORY.NumOfPoints());
                                s_TRAJECTORY.clear();
                                s_DELAY_FLAG = true;
                                s_DELAY_COUNT = 0;
				s_EXPECTED_FINISH_TIME = 0.0;
                            }
                            else if (s_MODE == MODE_SPLINE)
                            {
                                s_DELAY_COUNT = 0;
                            }
                        }
                        // else if the trajectory_input is not delayed trajectory, send pvt points to GMAS
                        else
                        {
                            if ( (!s_MOVING) && (s_TRAJECTORY.NumOfPoints() >= p_MinimalStartSize) && (!s_GRIP_MOVING) )
                            {
                                s_MOVING = true;
                            }
                            if ( (!s_MOVING) && (s_REMAINING_COMMAND_SIZE == 0 ))
                            {
                                s_DELAY_FLAG = false;
                            }

                            if ( s_MOVING  && !communicated )
                            {
                                // set the communicated flag
                                communicated = true;
                                // check the rotary PVT table and send trajectory points
                                if ( s_EXPECTED_FINISH_TIME - ros::Time::now().toSec() < p_GMAS_TIME * p_MinimalSendSize )
                                {
                                    StringAppend ( strActions, "4. send_trajectory\t" );
                                    // get the RemainingPoints of PVT Table, and assign it to variable s_REMAINING_COMMAND_SIZE
                                    s_REMAINING_COMMAND_SIZE = ReadModbus_Long(REM_POINTS);
                                    while (s_REMAINING_COMMAND_SIZE == -111111)
                                    {
                                        s_REMAINING_COMMAND_SIZE = ReadModbus_Long(REM_POINTS);
                                        ROS_ERROR("Error: PVT Remaining Points number is not available now.");
                                        throw Result ( ERROR_INVALID_FEEDBACK, "can not retrieve remaining points data" );
                                    }
                                    ROS_INFO("interface_buf_num[%zu].", s_TRAJECTORY.NumOfPoints());

                                    // check the state of s_TRAJECTORY (class PTTrajectory), determine wether to add points to GMAS
                                    if ((s_TRAJECTORY.NumOfPoints()>0) && (s_TRAJECTORY.NumOfPoints()+s_REMAINING_COMMAND_SIZE>3))
                                    {
                                        int SendSize = s_TRAJECTORY.NumOfPoints()>7 ? 7 : s_TRAJECTORY.NumOfPoints(); // 10
                                        std::vector<int> pvt;
                                        pvt.resize( SendSize*(s_TRAJECTORY.NumOfAxis()+1), 0.0 );
                                        for (int i = 0; i < SendSize; ++i)
                                        {
                                            pvt[(s_TRAJECTORY.NumOfAxis()+1)*i] = p_GMAS_TIME * 10000;
                                            // the a07 corresponds to linear actuated joint
                                            std::vector<int> aline = s_TRAJECTORY.PopFront(10000.0*180.0/3.1415926);
                                            for (int j = 0; j < s_TRAJECTORY.NumOfAxis(); ++j)
                                                pvt[(s_TRAJECTORY.NumOfAxis()+1)*i+j+1] = aline[j];
                                        }

    //-------------------------------------------------------test1----------------------------------------------------------------
                                        ROS_WARN("WriteModbus_PVT: GMAS_RemPnts[%d], SendSize[%d], BufferNum[%zu], PVTStep[%d].", s_REMAINING_COMMAND_SIZE,
                                                 SendSize, s_TRAJECTORY.NumOfPoints(), pvt.size()/(s_TRAJECTORY.NumOfAxis()+1));
    //-------------------------------------------------------test1----------------------------------------------------------------

                                        int resp = WriteModbus_PVT (PVT_START_INDEX,
                                                                    pvt,
                                                                    pvt.size()/(s_TRAJECTORY.NumOfAxis()+1),
                                                                    s_TRAJECTORY.NumOfAxis() );
                                        if (resp > 0)
                                            s_REMAINING_COMMAND_SIZE = s_REMAINING_COMMAND_SIZE + SendSize;
                                        else
                                            ROS_ERROR ("Error: modbus write for pvt Append_Points process failed.");
                                    }

                                    s_EXPECTED_FINISH_TIME = static_cast<double>(s_REMAINING_COMMAND_SIZE)
                                            * p_GMAS_TIME + ros::Time::now().toSec();

    //-------------------------------------------------------test3----------------------------------------------------------------
                                    ROS_WARN("After decision: RemPnts[%d], BufferNum[%d], finish_time[%lf]",
                                             s_REMAINING_COMMAND_SIZE, s_TRAJECTORY.NumOfPoints(), s_EXPECTED_FINISH_TIME*1000);
    //-------------------------------------------------------test3----------------------------------------------------------------

                                    // reset the s_MOVING flag wether s_TRAJECTORY has been cleared
                                    if ( s_REMAINING_COMMAND_SIZE == 0 )
                                    {
                                        s_MOVING = false;
                                        s_DELAY_FLAG = false;
                                        ROS_INFO ( "->  buffer empty:　stopped  <-");
                                    }
                                }
                                // else if the s_EXPECTED_FINISH_TIME is larger than (p_GMAS_TIME*p_MinimalSendSize),
                                // estimate the "s_REMAINING_COMMAND_SIZE" directly
                                else
                                {
                                    s_REMAINING_COMMAND_SIZE = ( s_EXPECTED_FINISH_TIME - ros::Time::now().toSec() ) / p_GMAS_TIME;
                                }
                            }
                            else if ( !s_MOVING && !communicated )
                            {
                                if ( GRIP_COMMAND_HANDLLER.newMsg() )
                                {
                                    std_msgs::String input_ = GRIP_COMMAND_HANDLLER.fetchMsg();
                                    std::string input = input_.data;
                                    if ((!f_grip_limitation) && (!r_grip_limitaion))
                                    {
                                        if (input == GRIP_FORWARD)
                                            s_CURRENT_GRIP_CMD = GRIP_GMAS_FORWARD;
                                        else if (input == GRIP_REVERSE)
                                            s_CURRENT_GRIP_CMD = GRIP_GMAS_REVERSE;
                                        else if (input == GRIP_STOP)
                                            s_CURRENT_GRIP_CMD = GRIP_GMAS_STOP;
                                    }
                                    else if ((f_grip_limitation) && (!r_grip_limitaion))
                                    {
                                        if (input == GRIP_FORWARD)
                                        {
                                            s_CURRENT_GRIP_CMD = GRIP_GMAS_STOP;
                                            ROS_WARN("The forward limitation of gripper.");
                                        }
                                        else if (input == GRIP_REVERSE)
                                            s_CURRENT_GRIP_CMD = GRIP_GMAS_REVERSE;
                                        else if (input == GRIP_STOP)
                                            s_CURRENT_GRIP_CMD = GRIP_GMAS_STOP;
                                    }
                                    else if ((!f_grip_limitation) && (r_grip_limitaion))
                                    {
                                        if (input == GRIP_FORWARD)
                                            s_CURRENT_GRIP_CMD = GRIP_GMAS_FORWARD;
                                        else if (input == GRIP_REVERSE)
                                        {
                                            s_CURRENT_GRIP_CMD = GRIP_GMAS_STOP;
                                            ROS_WARN("The reverse limitation of gripper.");
                                        }
                                        else if (input == GRIP_STOP)
                                            s_CURRENT_GRIP_CMD = GRIP_GMAS_STOP;
                                    }
                                    //
                                    // conduct the vibrated machine
                                    bool refresh_flag = false;
                                    if (s_CURRENT_GRIP_CMD != s_LAST_GRIP_CMD)
                                        refresh_flag = true;

                                    if (refresh_flag)
                                        s_REFRESH_COUNT++;
                                    else
                                        s_REFRESH_COUNT = 0;

                                    if (refresh_flag && s_REFRESH_COUNT>=3)
                                        s_LAST_GRIP_CMD = s_CURRENT_GRIP_CMD;
                                    //
                                    // determine wether GMAS read previous commands, then update the command state #38
                                    iCnt = 0;
                                    while( ReadModbus_Short(GRIP_CMD) != 0 )
                                    {
                                        ROS_ERROR("[ERROR] Refresh Command: Wait for reading gripper command.");
                                        ros::Duration(0.2).sleep();
                                    }

                                    if (refresh_flag && s_REFRESH_COUNT>=3)
                                    {
                                        std::cout << "New gripper command:" << s_CURRENT_GRIP_CMD << std::endl;

                                        int resp1 = WriteModbus_Short (GRIP_CMD, s_CURRENT_GRIP_CMD);
                                        if (resp1 < 0)
                                            ROS_ERROR("Error: modbus write for gripper velocity parameter(NO.38) failed.");

                                        int resp2 = WriteModbus_Short (COMMAND_POS, 38);
                                        if (resp2 > 0)
                                            ROS_INFO("Write the gripper command(NO.38) successfully.");
                                        else
                                            ROS_ERROR("Error: modbus write for gripper command(NO.38) failed.");

                                        // write the command index to modbus
                                        iCnt = 0;
                                        bool check_38 = false;
                                        if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                        {
                                            check_38 = true;
                                        }
                                        while( check_38 )
                                        {
                                            int regs1 = ReadModbus_Short(COMMAND_POS);
                                            int regs2 = ReadModbus_Short(FUNC_STATE);
                                            if ((ReadModbus_Short(COMMAND_POS) != 0) || (ReadModbus_Short(FUNC_STATE) != 2))
                                                check_38 = true;
                                            else
                                                check_38 = false;
                                            ROS_INFO("Wait for the end of the state_function 38. cmd_pos[%d], fun_state[%d]", regs1, regs2);
                                            ros::Duration( 0.2 ).sleep();
                                        }
                                        //refresh the variables: s_GRIP_MOVING and s_REFRESH_COUNT
                                        if ((s_CURRENT_GRIP_CMD == GRIP_GMAS_FORWARD) || (s_CURRENT_GRIP_CMD == GRIP_GMAS_REVERSE))
                                            s_GRIP_MOVING = true;
                                        else if (s_CURRENT_GRIP_CMD == GRIP_GMAS_STOP)
                                            s_GRIP_MOVING = false;

                                        s_REFRESH_COUNT = 0;
                                    }
                                }
                            }
                        }
                    }
                    //
                    else
                    {
                        s_MOVING = false;
                        s_REMAINING_COMMAND_SIZE = 0;
                        s_EXPECTED_FINISH_TIME = 0.0;
                    }

                    double cost_6 = ros::Time::now().toSec() - start_6;
                    ROS_DEBUG("6. GMAS_COMMUNICATE end. cost_time: %lf ms.", cost_6*1000);
                    //
                    double start_7 = ros::Time::now().toSec();

                    strStatus.clear();
                    //
                    if ( s_MODE == MODE_MOVE )
                        StringAppend ( strStatus, INTERFACE_STATE_MOVE INTERFACE_STATE_SPARATER );
                    else if ( s_MODE == MODE_SPLINE )
                        StringAppend ( strStatus, INTERFACE_STATE_SPLINE INTERFACE_STATE_SPARATER );
                    else if ( s_MODE == MODE_INIT )
                        StringAppend ( strStatus, INTERFACE_STATE_INIT INTERFACE_STATE_SPARATER );
                    else if ( s_MODE == MODE_STOP )
                        StringAppend ( strStatus, INTERFACE_STATE_STOP INTERFACE_STATE_SPARATER );
                    else if ( s_MODE == MODE_HOME )
                        StringAppend ( strStatus, INTERFACE_STATE_HOME INTERFACE_STATE_SPARATER );
                    //
                    if ( s_MOVING )
                        StringAppend ( strStatus, INTERFACE_STATE_MOVING INTERFACE_STATE_SPARATER );
                    else
                        StringAppend ( strStatus, INTERFACE_STATE_STABLE INTERFACE_STATE_SPARATER );
                    //
                    if ( s_DELAY_FLAG )
                    {
                        StringAppend ( strStatus, INTERFACE_STATE_DELAYED INTERFACE_STATE_SPARATER );
                        ROS_ERROR("buffer_TEST[%zu] GMAS_REM_SIZE[%d].", s_TRAJECTORY.NumOfPoints(), s_REMAINING_COMMAND_SIZE);
                    }
                    else
                        StringAppend ( strStatus, INTERFACE_STATE_NORMAL INTERFACE_STATE_SPARATER );
                    //
                    StringAppend ( strStatus, "%4ld %8.3lf\t", s_REMAINING_COMMAND_SIZE, s_EXPECTED_FINISH_TIME );
                    ROS_DEBUG ( "%s" INTERFACE_STATE_SPARATER "%s", strStatus.c_str (), strActions.c_str () );
                    //
                    std_msgs::String output;
                    output.data = strStatus;
                    pInterfaceStates.publish(output);
                    //
                    int resp = ReadModbus_Short (ERROR_FLAG);
                    ROS_DEBUG("[Important] Error Flag: [%d], s_MODE: [%d]", resp, s_MODE);

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
                        stop_flag = 1;
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

            // conduct the GMAS_unloaded and Buffer_cleared sub-function (no.34)
            int resp = WriteModbus_Short (COMMAND_POS, 34); // Refer to StateFunction_34
            if (resp > 0)
                ROS_INFO("Write the GMAS_Unload process successfully.");
            else
                ROS_ERROR("Error: Write the GMAS_Unload process failed.");

            // check the pvt_table state and buffer_steps

            bool check_34_1 = false;
            if((ReadModbus_Short(BUF_REM)!=0) && (ReadModbus_Short(BUF_REM)!=1) && (ReadModbus_Short(PVT_INIT)==1))
            {
                check_34_1 = true;
            }
            while( check_34_1 )
            {
                int reg_buf_rem = ReadModbus_Short(BUF_REM);
                int reg_pvt_init = ReadModbus_Short(PVT_INIT);
                if((reg_buf_rem!=0) && (reg_buf_rem!=1) && (reg_pvt_init==1))
                    check_34_1 = true;
                else
                    check_34_1 = false;
                ROS_ERROR("Error: GMAS Buffer or PVT Table has not been cleared (#34_1). BUF_NUM[%d], PVT_INIT[%d].", reg_buf_rem, reg_pvt_init);
                ros::Duration( 0.2 ).sleep();
            }
            ROS_INFO("initialize PVT and GMAS_Buffer succeed");
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
        pFeedbackStates.shutdown();
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




