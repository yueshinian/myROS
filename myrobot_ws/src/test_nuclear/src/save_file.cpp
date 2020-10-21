#include "ros/ros.h"
#include <fstream>
#include <time.h>

#define     SAVE_PATH               "/home/kangyuan45/text.txt"
#define     STREAM_TEST_SEGMENT     "########################################"
#define     STREAM_SEQ_SEGMENT      "----------------------------------------"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_file");
    ros::NodeHandle nh;
    std::ofstream write;

    //
    time_t time_seconds = time(0);
    struct tm* now_time = localtime(&time_seconds);
    write.open( SAVE_PATH, std::ofstream::out | std::ofstream::app );
    write << std::endl << std::endl << STREAM_TEST_SEGMENT << std::endl;
    write << "TEST started: " << now_time->tm_hour << ":"
          << now_time->tm_min << ":"
          << now_time->tm_sec << std::endl;
    write << STREAM_SEQ_SEGMENT << std::endl;
    write.close();
    ros::Duration(20).sleep();
    //
    time_seconds = time(0);
    now_time = localtime(&time_seconds);
    int reserve_seq = 2;
    double  reserve_rotation_max = 30.0;
    double  reserve_intensity_max = 102.3;
    write.open( SAVE_PATH, std::ofstream::out | std::ofstream::app );
    write << "time:\t" << now_time->tm_hour << ":"
          << now_time->tm_min << ":"
          << now_time->tm_sec << std::endl;
    write << "sequence:\t" << reserve_seq << std::endl;
    write << "odom_pose:\t" << 234.52 << 12.53 << 1.43 << std::endl;
    write << "rotation_max:\t" << reserve_rotation_max << std::endl;
    write << "intensity_max:\t" << reserve_intensity_max << std::endl;
    write.close();
    while(1);
    //
    return 0;
}
