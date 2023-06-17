#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "mv2.h"

class joy2mv2
{
public:
    joy2mv2(ros::NodeHandle node);
    ~joy2mv2() {}
    void init();
    void pub();
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    void joyMsgCb(const sensor_msgs::Joy::ConstPtr &msg);

    mv2_msgs::control ctrlMsg;
};

joy2mv2::joy2mv2(ros::NodeHandle node)
{
    sub_ = node.subscribe<sensor_msgs::Joy>("joy", 10, boost::bind(&joy2mv2::joyMsgCb, this, _1));
    pub_ = node.advertise<mv2_msgs::control>("mv2/control", 10);
}

void joy2mv2::init()
{
    // init mode
    ctrlMsg.Drive.Mode     = MV2_DRIVE_MODE_PROGRAM;
    ctrlMsg.Drive.cMode    = MV2_DRIVE_CMODE_STROKE;
    ctrlMsg.Drive.Servo    = (MV2_SERVO)SERVO_ON;
    ctrlMsg.Steering.Mode  = MV2_STEER_MODE_PROGRAM;
    ctrlMsg.Steering.cMode = MV2_STEER_CMODE_ANGLE;
    ctrlMsg.Steering.Servo = (MV2_SERVO)SERVO_ON;

    // init value
    ctrlMsg.Drive.Accel = 0;
    ctrlMsg.Drive.Brake = 0;
}

void joy2mv2::pub()
{
    pub_.publish(ctrlMsg);
}

int main(int argc, char ** argv)
{
    // ROS
    ros::init(argc, argv, "joy_ctrl_to_mv2");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

    joy2mv2 Joy(node);
    Joy.init();

    // handler
    // joyRecv = node.subscribe("joy", 10, joyMsgCb);
    // ctrlPub = node.advertise<mv2_msgs::control>("mv2/control", 10);

    while(ros::ok()){
        Joy.pub();
        ros::spinOnce();
        loop_rate.sleep();        
    }

    return 0;
}

void joy2mv2::joyMsgCb(const sensor_msgs::Joy::ConstPtr &msg)
{
    // ROS_INFO_STREAM("joyMsgCb");
    // ROS_INFO("brk %f", msg->axes[2]);
    ctrlMsg.Steering.Angle = -(int)(msg->axes[0] * 4000);
    ctrlMsg.Drive.Accel = (int)((-msg->axes[5]+1) * 1000);
    ctrlMsg.Drive.Brake = (int)((-msg->axes[2]+1) * 2000);
}