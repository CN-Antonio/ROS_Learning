#include <ros/ros.h>
#include "mv2.h"

void joyMsgCb(const mv2_msgs::control &msg);

int main(int argc, char ** argv)
{
    // ROS
    ros::init(argc, argv, "joy ctrl to mv2");
    ros::NodeHandle node;
    ros::Subscriber joyRecv;
    ros::Publisher crtlPub;
    ros::Rate loop_rate(10);

    // handler
    joyRecv = node.subscribe("joy", 10, joyMsgCb);
    crtlPub = node.advertise<mv2_msgs::control>("mv2/control", 10);

    while(ros::ok()){
        // ctrlPub.publish()

        loop_rate.sleep();
    }

    return 0;
}

void joyMsgCb(const mv2_msgs::control &msg)
{

}