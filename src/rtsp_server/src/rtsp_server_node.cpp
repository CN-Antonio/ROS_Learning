// std
#include <sys/shm.h>
#include <pthread.h>
#include <semaphore.h>
#include "bits/stdc++.h"  // TODO:remove
// ROS
#include <ros/ros.h>
// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>       // opencv GUI
// #include <opencv2/imgproc/types_c.h>
// #include <opencv2/imgproc/imgproc.hpp>    // 图像处理
// cv_bridge
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// Gstreamer
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
// Project
#include <streamer.h>

#define WINDOW_NAME  "gst_camera"
#define DEFAULT_RTSP_PORT "8554"
// #define WIDTH  1920 // moved to stream.h
// #define HEIGHT 1080
bool USE_GST_RTSP    = true; 
bool USE_GST_SHOW    = true;
bool USE_OPENCV_SHOW = false;

static char *port = (char *) DEFAULT_RTSP_PORT;
bool start_gst = false;
volatile int stop_rc = 0;

/* moved to stream.h
struct shared_use_st
{
    int Index;
    char Buffer[WIDTH*HEIGHT*3];
    sem_t sem;
};*/

void *shm = NULL;
struct shared_use_st *shared = NULL;
int shmid;

static void signal_handler(int signo)
{
    printf("SIGINT: Closing accessory\n");
    stop_rc = 1;
    usleep(100000);
    exit(-1);
}

int shm_init(void)
{
    int i = 0;
    shmid = shmget((key_t)123, sizeof(struct shared_use_st), 0666|IPC_CREAT);
    if(shmid == -1)
    {
        exit(EXIT_FAILURE);
    }
    shm = shmat(shmid, (void*)0, 0);
    if(shm == (void*)-1)
    {
        exit(EXIT_FAILURE);
    }
    printf("Memory attached at %ld\n", (intptr_t)shm);
    shared = (struct shared_use_st*)shm;
    shared->Index = 0;
    sem_init(&(shared->sem),1,1);
    return 1;
}

void *captureImage(void *arg)
{
    cv::Mat img_input;
    unsigned int capwidth = 0;
    unsigned int capheight = 0;
    unsigned int framerate = 0;
    unsigned int frame_num = 0;
    cv::VideoCapture capture;
    //camera source support: usb camera, csi, rtsp
    //usb cam: MJPG
    //camera source support: usb camera, cv_bridge, csi, rtsp
    //usb cam: UYVY
    //capture.open("v4l2src device=/dev/video0 ! video/x-raw, width=1920, height=1080, format=UYVY, framerate=30/1 ! videoconvert ! video/x-raw, format=(string)BGR, width=1920,height=1080 ! appsink max-buffers=1 drop=false sync=false", cv::CAP_GSTREAMER);

    //csi
    
    // cv_bridge

    //capture.open("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1920,height=1080, format=(string)NV12 ! nvvidconv flip-method=2 ! video/x-raw,width=1920,height=1080,format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);

    //rtsp
    //capture.open("rtspsrc location=rtsp://admin:hyzn1234@192.168.1.64:554/h264/ch1/main/av_stream latency=0 ! rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv flip-method=2 ! video/x-raw, width=(int)1920, height=(int)1080, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink sync=false", cv::CAP_GSTREAMER);

    if(!capture.isOpened()){
        std::cout<<"VideoCapture or VideoWriter not opened"<<std::endl;
        exit(-1);
    }
    std::cout<<"VideoCapture or VideoWriter opened"<<std::endl;

    int key = -1;
    if(USE_OPENCV_SHOW){
        cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
        cv::resizeWindow(WINDOW_NAME,960,540);
        cv::moveWindow(WINDOW_NAME, 0, 0);
        cv::startWindowThread();
    }
    capwidth = capture.get(cv::CAP_PROP_FRAME_WIDTH);
    capheight = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
    framerate = capture.get(cv::CAP_PROP_FPS);
    std::cout << "width : " << capwidth << " height : " << capheight << " framerate : " << framerate << std::endl;
    char *image;

    auto start = std::chrono::system_clock::now();
    while(!stop_rc)
    {
        capture >> img_input;
        start_gst = true;
        //auto start = std::chrono::system_clock::now();
        if(USE_OPENCV_SHOW){
            cv::startWindowThread();
            cv::imshow(WINDOW_NAME, img_input);
            key = cv::waitKey(10);
            if (key == 'q'){
                break;
            }
        }
        //process image
        image = (char*)img_input.data;
        //save image
        if(sem_wait(&(shared->sem)) == -1)
        {
            printf("P ERROR!\n");
            exit(EXIT_FAILURE);
        }
        memcpy(shared->Buffer, image, WIDTH*HEIGHT*3);
        sem_post(&shared->sem);
        // if(USE_GST_RTSP){
        //     if(rtspqueue.getSize() < 5){
        //         msg *mp_rtsp = new msg;
        //         mp_rtsp->len = WIDTH*HEIGHT*3;
        //         mp_rtsp->data = new uint8_t[WIDTH*HEIGHT*3];
        //         memcpy(mp_rtsp->data, image, WIDTH*HEIGHT*3);
        //         pthread_mutex_lock(&rtspMutex);
        //         rtspqueue.push(mp_rtsp);
        //         pthread_mutex_unlock(&rtspMutex);
        //         pthread_cond_signal(&rtspCond);
        //     }
        // }

        // if(USE_GST_SHOW){
        //     msg *mp_gst = new msg;
        //     mp_gst->len = WIDTH*HEIGHT*3;
        //     mp_gst->data = new uint8_t[WIDTH*HEIGHT*3];
        //     memcpy(mp_gst->data, image, WIDTH*HEIGHT*3);
        //     pthread_mutex_lock(&gstMutex);
        //     gstqueue.push(mp_gst);
        //     pthread_mutex_unlock(&gstMutex);
        //     pthread_cond_signal(&gstCond);
        // }
        auto end = std::chrono::system_clock::now();
        int fps = 1000.0/std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        //std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    }

    if(USE_OPENCV_SHOW){
        capture.release();
        cv::destroyAllWindows();
    }
}

void *gst_rtsp(void *arg)
{
    // while(!start_gst)
    // {
    while(!start_gst)
    {
        usleep(100);
    }
    GMainLoop *loop;
    GstRTSPServer *server;
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory;
    // loop = g_main_loop_new (NULL, FALSE);

    // pthread_t m_rtsp;
    // pthread_create(&m_rtsp, NULL, multirtsp, loop);
    // server = gst_rtsp_server_new ();
    // g_object_set (server, "service", port, NULL); 
    
    // mounts = gst_rtsp_server_get_mount_points (server);
    // factory = gst_rtsp_media_factory_new ();

    // //SW encode
    // //gst_rtsp_media_factory_set_launch (factory,
    // //        "( appsrc name=videosrc is-live=true ! videoconvert ! video/x-raw, format=I420 ! x264enc bitrate=4000000 ! rtph264pay config-interval=10 name=pay0 pt=96 )");

    // //HW encode
    // //Profile Description 0 Baseline profile 2 Main profile 4 High profile
    // //Hardware Preset Level 0 DisablePreset 1 UltraFastPreset 2 FastPreset
    // gst_rtsp_media_factory_set_launch (factory,
    //         "( appsrc name=videosrc is-live=true ! videoconvert ! nvvidconv ! video/x-raw(memory:NVMM), format=I420 ! nvv4l2h264enc maxperf-enable=1 control-rate=0 bitrate=4000000 preset-level=2 profile=2 ! rtph264pay name=pay0 pt=96 sync=false )");

    // gst_rtsp_media_factory_set_shared (factory, TRUE); 
    // g_signal_connect (factory, "media-configure", (GCallback) media_configure, NULL);
    // gst_rtsp_mount_points_add_factory (mounts, "/test", factory);
    // g_object_unref (mounts);
    // gst_rtsp_server_attach (server, NULL);
    // /* start serving */
    // g_print ("stream ready at rtsp://127.0.0.1:8554/test\n");
    // pthread_join(m_rtsp, NULL);
}

int main(int argc, char *argv[])
{
    // ROS
    ros::init(argc, argv, "RTSP_Streamer");
    ros::NodeHandle node_handle;
    int frame_width;
    int frame_height;
    std::string camera_name;
    std::string camera_info_url;
    std::string user;
    std::string pw;
    std::string ip;
    std::string port;

    if(node_handle.getParam("camera_name", camera_name))
        ROS_INFO("camera_name is %s", &camera_name);
    else
        ROS_WARN("didn't find parameter camera_name");
    node_handle.getParam("camera_info_url", camera_info_url);
    node_handle.getParam("/ros_streamer/width", frame_width);
    node_handle.getParam("/ros_streamer/height", frame_height);
    node_handle.getParam("/user", user);
    node_handle.getParam("/pw", pw);
    node_handle.getParam("/ip", ip);
    node_handle.getParam("/port", port);

    // thread
    std::cout<<"RTSP Server Start"<<std::endl;
    signal(SIGINT, signal_handler);
    shm_init();

    /* thread entry */
    // cv_bridge recv
    pthread_t thread_capture;
    pthread_create(&thread_capture, NULL, captureImage, NULL);

    // RTSP server
    if(USE_GST_RTSP){
        pthread_t thread_gst_rtsp;
        pthread_create(&thread_gst_rtsp, NULL, gst_rtsp, NULL);
    }

    while(!stop_rc){
        if(sem_wait(&(shared->sem)) == -1)
        {
            printf("P ERROR!\n");
            exit(EXIT_FAILURE);
        }
        // memcpy(BGR_IMG, shared->Buffer, WIDTH*HEIGHT*3);
        // sem_post(&shared->sem);
        // frame = Mat(HEIGHT, WIDTH, CV_8UC3, BGR_IMG);
        //http server
        //cv::resize(frame, frame, Size(1280, 720), 0, 0, cv::INTER_LINEAR);
        //send_mjpeg(frame, 8090, 400000, 10);
        //usleep(10);

        //save pic
        //cv::imwrite("test.jpg", frame);
        usleep(10000);
    }
    std::cout<<"RTSP Server Exit"<<std::endl;
    return 0;
}