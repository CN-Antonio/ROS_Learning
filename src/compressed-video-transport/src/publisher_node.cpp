#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <ros/ros.h>
// #include "std_msgs/String.h"

#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_cam_node");
    ros::NodeHandle nh;
    ROS_INFO("Built with OpenCV %s", CV_VERSION);

    cv::String pipeline;

    switch(*argv[1])
    {
        case 48:
            pipeline = "v4l2src device=/dev/video0 \
! videoconvert \
! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)RGB, framerate=(fraction)30/1 \
! videorate \
! videoconvert \
! appsink";
            // pipeline = "v4l2src device=/dev/video0 ! videoconvert ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)RGB, framerate=(fraction)30/1 ! videorate ! videoconvert ! appsink";
            break;
        case 49:
            pipeline = "v4l2src device=/dev/video0 io-mode=2 \
! \"image/jpeg,width=1920,height=1080,framerate=30/1,format=MJPG\" \
! nvjpegdec \
! video/x-raw ! nvvidconv \
! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, format=(string)I420, framerate=(fraction)30/1 \
! videoconvert \
! appsink";
            break;
        case 50:
            pipeline = "v4l2src device=/dev/video0 io-mode=2 \
! image/jpeg, witdh=1920 \
! v4l2jpegdec ! video/x-raw \
! imxvideoconvert_g2d ! video/x-raw, format=BGRx ! queue \
! videoconvert ! video/x-raw,format=BGR \
! appsink";
            break;
        default:
            pipeline = "videotestsrc \
! videoconvert \
! videoscale \
! appsink";
            break;
    }
    cv::VideoCapture capture(pipeline, cv::CAP_GSTREAMER);
    // capture.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    // capture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    // capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    
    // cv::VideoWriter video;
    // video.open(testpipeline, 0, (double)30, cv::Size(1280, 720), true);

    if (!(capture).isOpened()) {
        ROS_ERROR_STREAM("Failed to open video device\n");
        ros::shutdown();
    }

    //image_transport will publish the video that can be compressed
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_color = it.advertise("/color", 1);

    cv::Mat image = cv::Mat(1280, 720, CV_8UC3);

    while (ros::ok()) {
        // video.write(image);
        capture >> image; //load
        if (image.empty()) {
            ROS_ERROR_STREAM("Failed to capture image!");
            ros::shutdown();
        }
        pub_color.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg());
    }

    ros::spin();
}