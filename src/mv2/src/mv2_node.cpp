#include <ros/ros.h>
#include "mv2.h"

int main(int argc, char * argv[])
{
    ROS_INFO_STREAM("ros2-Mv2_20230528");
    
    // ROS
    ros::init(argc, argv, "mv2_driver");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    // ROS2
    // rclcpp::init(argc, argv);

    // start the driver
    mv2_driver::Mv2Driver dvr(node, private_nh);

    while(ros::ok() /*&& !ros_shutdown*/)
    {
        // poll device until end of file
        // bool polled_ = dvr.poll();
        // if (!polled_)
        //     ROS_ERROR_THROTTLE(1.0, "Velodyne - Failed to poll device.");

        ros::spinOnce();
    }

    // ros::shutdown();
    // rclcpp::shutdown();

    return 0;
}
