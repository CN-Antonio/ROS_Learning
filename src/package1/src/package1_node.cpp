#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"

int main(int argc, char ** argv){
    ros::init(argc,argv, "package1_talker");
    ros::NodeHandle n;
    ros::Publisher package1_pub = n.advertise<std_msgs::String>("package1_topic",10);
    ros::Rate loop_rate(10);
    int count=0;

    while(ros::ok()){
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello package1" << count;
        count++;
        msg.data = ss.str();
        package1_pub.publish(msg);
        loop_rate.sleep();
    }
}