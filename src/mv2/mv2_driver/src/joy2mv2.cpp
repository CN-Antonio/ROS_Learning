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
    ctrlMsg.Drive.mode     = (MV2_MODE)MODE_MANUAL;
    ctrlMsg.Steering.mode  = (MV2_MODE)MODE_MANUAL;

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
    /* mode */
    if(msg->buttons[0] == 1) // A => Gear-D
    {
        ctrlMsg.Drive.Shift= (SHIFT_POSITION)SHIFT_POS_D;
    }
    if(msg->buttons[1] == 1) // B => Gear-R
    {
        ctrlMsg.Drive.Shift= (SHIFT_POSITION)SHIFT_POS_R;
    }
    if(msg->buttons[3] == 1) // X => Program(PC)
    {
        ROS_INFO_STREAM("Program Mode");
        ctrlMsg.Drive.mode     = (MV2_MODE)MODE_PROGRAM;
        ctrlMsg.Drive.cmode    = (DRIVE_CONTROL_MODE)CONT_MODE_STROKE;
        ctrlMsg.Drive.servo    = (MV2_SERVO)SERVO_ON;
        ctrlMsg.Steering.mode  = (MV2_MODE)MODE_PROGRAM;
        ctrlMsg.Steering.cmode = (STEER_CONTROL_MODE)CONT_MODE_ANGLE;
        ctrlMsg.Steering.servo = (MV2_SERVO)SERVO_ON;
    }
    if(msg->buttons[4] == 1) // Y => Manual
    {
        ROS_INFO_STREAM("Manual Mode");
        ctrlMsg.Drive.mode     = (MV2_MODE)MODE_MANUAL;
        ctrlMsg.Steering.mode  = (MV2_MODE)MODE_MANUAL;
    }

    /* value */
    ctrlMsg.Steering.tAngle = -(int)(msg->axes[0] * 4000);
    ctrlMsg.Drive.Accel = (int)((-msg->axes[4]+1) * 1000);
    ctrlMsg.Drive.Brake = (int)((-msg->axes[5]+1) * 2000);
}