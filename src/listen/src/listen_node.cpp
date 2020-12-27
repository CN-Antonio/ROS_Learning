#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pkg_msgs/pkgMsg.h"

void CallBack(const std_msgs::String:Constptr* msg){
    ROS_INFO("I Heard %s",msg->data.c_str());
}

int main(int argc, char ** argv){
    ros::init(argc,argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("package1_topic",10,CallBack);
    ros::spin();
    return 0;
}