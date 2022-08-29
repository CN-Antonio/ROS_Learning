#include "ros/ros.h"
#include "std_msgs/String.h"

void CallBack(const std_msgs::String::ConstPtr &msg){
    ROS_INFO("eBUS info: %s",msg->data.c_str());
}

int main(int argc, char ** argv){
    ros::init(argc,argv, "eBUS_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("eBUS_topic",10,CallBack); //(接收来自topic,队列长度，回调函数)
    ros::spin(); //反复查看队列，处理、清空队列；反复调用当前可触发的回调函数
    return 0;
}