#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"
#include "msgs/Msg.h"

int main(int argc, char ** argv){
    ros::init(argc,argv, "say_talker");
    ros::NodeHandle n;
    ros::Publisher say_pub = n.advertise<std_msgs::String>("say_topic",10);
    ros::Publisher say_pub_new = n.advertise<msgs::Msg>("say_topic_new",10);
    ros::Rate loop_rate(10);
    int count=0;

    while(ros::ok()){
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Hello say" << count;
        count++;
        msg.data = ss.str();
        say_pub.publish(msg);

        //参数中心
        std::string param_string;
        n.param<std::string>("myparam",param_string,"Hi");

        // Msg.h
        msgs::Msg Mymsg;
        Mymsg.id = count;
        // Mymsg.detail = "hello world";
        Mymsg.detail = param_string;    //参数中心传递publish的数据
        say_pub_new.publish(Mymsg);

        loop_rate.sleep();
    }
}