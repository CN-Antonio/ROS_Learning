#include "ros/ros.h"
#include "std_msgs/String.h"
// #include "msgs/Msg.h"

void CallBack(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I Heard %s",msg->data.c_str());
}

int main(int argc, char ** argv){
    ros::init(argc,argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("say_topic",10,CallBack);
    ros::spin();
    return 0;
}