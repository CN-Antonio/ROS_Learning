#ifndef STREAMER_H
#define STREAMER_H

// std
#include <semaphore.h>

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/assign/list_of.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/Image.h>

#define WIDTH  640
#define HEIGHT 480

// communication between threads
struct shared_use_st
{
    int Index;
    char Buffer[WIDTH * HEIGHT * 3];
    sem_t sem;
};

class cvBridge
{
public:
    cvBridge();
    cvBridge(struct shared_use_st *shared);
    ~cvBridge();
private:
void *imageCb(void *arg);
};


namespace rtsps {
class RtspStreamer {
public:
    RtspStreamer(ros::NodeHandle &, std::string, std::string, int, int);
    virtual ~RtspStreamer();

    void registerPublishers();

    sensor_msgs::CameraInfo get_default_camera_info_from_image(sensor_msgs::ImagePtr);

    void publishStream(const std::string);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::CameraPublisher rtsp_pub_;
    cv_bridge::CvImage bridge_;

    std::string camera_name_;
    std::string camera_info_url_;
    int frame_width_;
    int frame_height_;

};
}

#endif //STREAMER_H
