// ROS
#include "ros/ros.h"
// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[])
{
    // ROS
    ros::init(argc, argv, "RTSP_Streamer");
    ros::NodeHandle node_handle;

    std::cout<<"RTSP Server Start"<<std::endl;

    std::cout<<"RTSP Server Exit"<<std::endl;
    return 0;
}