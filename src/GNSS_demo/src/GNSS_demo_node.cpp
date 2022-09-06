//Publish GNSS messages
#include"ros/ros.h"
#include"std_msgs/String.h"
#include"sstream"

int main(int argc, char ** argv){
    ros::init(argc,argv,"GNSS");
    ros::NodeHandle GNSS;
    ros::Publisher GNSS_pub = GNSS.advertise<std_msgs::String>("GNSS_topic",1);
    ros::Rate loop_rate(1); //1Hz
    int count=0;

    while(ros::ok()){
        std_msgs::String msg;
        std::stringstream ss;
        ss <<"GNSS Hello" << count;
        count++;
        msg.data = ss.str();
        GNSS_pub.publish(msg);


        loop_rate.sleep();
    }
}