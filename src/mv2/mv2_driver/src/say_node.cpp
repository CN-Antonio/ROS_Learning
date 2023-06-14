#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"

#include "mv2_msgs/control.h"

int main(int argc, char ** argv){
    ros::init(argc,argv, "say_talker"); //解析参数，命名当前node
    ros::NodeHandle n;  //创建句柄，实例化node
    ros::Publisher say_pub = n.advertise<mv2_msgs::control>("mv2/control", 10); //(发送的目标topic，消息队列长度)
    ros::Rate loop_rate(1);    //控制rate=16.0Hz
    int count = 0;

    while(ros::ok()){
        mv2_msgs::control ctl;
        std::stringstream ss;
        ss << "Hello say " << count;
        count++;
        ctl.str = ss.str();
        say_pub.publish(ctl);
        // say_pub_new.publish(msg);

        //参数中心：传递外界读入数据
        // std::string param_string;
        // n.param<std::string>("myparam",param_string,"Hi");
        //bash：rosparam set myparam "OK"

        /*/ Msg.h
        msgs::Msg Mymsg;
        Mymsg.id = count;
        // Mymsg.detail = "hello world";
        Mymsg.detail = param_string;    //参数中心传递publish的数据
        say_pub_new.publish(Mymsg);
        //*/

        loop_rate.sleep();
    }
}