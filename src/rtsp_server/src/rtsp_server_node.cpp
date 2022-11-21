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
#include "loopqueue.hpp"

#define WINDOW_NAME  "gst_camera"
#define DEFAULT_RTSP_PORT "8554"
// #define WIDTH  1920 // moved to stream.h
// #define HEIGHT 1080
bool USE_GST_RTSP    = true;
bool USE_GST_SHOW    = false;
bool USE_OPENCV_SHOW = false;


/* Private macro -------------------------------------------------------------*/
/* thread BEGIN PM */
/* moved to stream.h
struct shared_use_st
{
    int Index;
    char Buffer[WIDTH*HEIGHT*3];
    sem_t sem;
};*/
/* thread END PM */

/* GST BEGIN PM */
typedef struct Msg
{
    int len;
    uint8_t* data;
} msg;
/* GST END PM */

/* ffmpeg BEGIN PM */
//ffmpeg  -i rtsp://192.168.1.6:8554/test -vcodec  copy  -t 60  -y test.mp4
typedef struct
{
    gboolean white;
    GstClockTime timestamp;
} MyContext;
/* ffmpeg END PM */

/* Private variables ---------------------------------------------------------*/
/* GST BEGIN PV */
static char *port = (char *) DEFAULT_RTSP_PORT;
bool start_gst = false; // flag

LoopQueue<msg*> gstqueue(32);
LoopQueue<msg*> rtspqueue(32);

pthread_mutex_t gstMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  gstCond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t rtspMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  rtspCond = PTHREAD_COND_INITIALIZER;
int count_t = 0;
/* GST END PV */

volatile int stop_rc = 0; // global run flag

/* Private function prototypes -----------------------------------------------*/
/* called when we need to give data to appsrc */
static void need_data (GstElement * appsrc, guint unused, MyContext * ctx);
static void need_data (GstElement * appsrc, guint unused, MyContext * ctx)
{
    GstBuffer *buf;
    guint buffersize;
    GstFlowReturn ret;
    GstMapInfo map;
    GstClockTime pts, dts;
    count_t++;

    pthread_mutex_lock(&rtspMutex);
    while (0 == rtspqueue.getSize())
    {
        pthread_cond_wait(&rtspCond, &rtspMutex);
        usleep(100);
    }
    buf = gst_buffer_new_allocate(NULL, rtspqueue.top()->len, NULL);
    if (!buf){
        pthread_mutex_unlock(&rtspMutex);
        return;
    }
    if(!gst_buffer_map(buf, &map, GST_MAP_WRITE)){
        gst_buffer_unref(buf);
        pthread_mutex_unlock(&rtspMutex);
        return;
    }else{
        memcpy((guchar *)map.data, (guchar *)(rtspqueue.top()->data), rtspqueue.top()->len);
    }
    pthread_mutex_unlock(&rtspMutex);

    GST_BUFFER_PTS(buf) = ctx->timestamp;
    GST_BUFFER_DURATION(buf) = gst_util_uint64_scale_int(1, GST_SECOND, 30);
    ctx->timestamp += GST_BUFFER_DURATION(buf);
    g_signal_emit_by_name(appsrc, "push-buffer", buf, &ret);
    if(buf){
        gst_buffer_unmap (buf, &map);
        gst_buffer_unref(buf);
    }
    if (ret != 0)
    {
        printf("g_main_loop_quit\n");
    }
    delete rtspqueue.top()->data;
    delete rtspqueue.top();
    rtspqueue.pop();
}
// config RTSP param
static void media_configure (GstRTSPMediaFactory * factory, GstRTSPMedia * media, gpointer user_data);
static void media_configure (GstRTSPMediaFactory * factory, GstRTSPMedia * media, gpointer user_data)
{
    printf("media_configure\n");
    GstElement *element, *appsrc;
    MyContext *ctx;

    /* get the element used for providing the streams of the media */
    element = gst_rtsp_media_get_element (media);

    /* get our appsrc, we named it 'videosrc' with the name property */
    appsrc = gst_bin_get_by_name_recurse_up (GST_BIN (element), "videosrc");
    g_object_set (G_OBJECT (appsrc), 
        "stream-type" , 0 , //rtsp
        "format" , GST_FORMAT_TIME , NULL);

    g_object_set (G_OBJECT (appsrc), "caps",
        gst_caps_new_simple ("video/x-raw",
            "format", G_TYPE_STRING, "BGR",
            "width", G_TYPE_INT, WIDTH,
            "height", G_TYPE_INT, HEIGHT,
            "framerate", GST_TYPE_FRACTION, 30, 1,
            "pixel-aspect-ratio", GST_TYPE_FRACTION, 1, 1, NULL), NULL);

    ctx = g_new0 (MyContext, 1);
    ctx->white = FALSE;
    ctx->timestamp = 0;
    /* make sure ther datais freed when the media is gone */
    g_object_set_data_full (G_OBJECT (media), "my-extra-data", ctx,
        (GDestroyNotify) g_free);

    /* install the callback that will be called when a buffer is needed */
    g_signal_connect (appsrc, "need-data", (GCallback) need_data, ctx);
    gst_object_unref (appsrc);
    gst_object_unref (element);
}
// for loop
void* multirtsp(void *args);
void* multirtsp(void *args)
{
    GMainLoop *loop = (GMainLoop *) args;
    g_main_loop_run (loop);
}


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
    //camera source support: usb camera, cv_bridge, csi, rtsp
    //usb cam: MJPG
    capture.open("v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! nvv4l2decoder mjpeg=1 ! nvvidconv flip-method=0 ! video/x-raw,width=1920,height=1080,format=BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);
    //usb cam: UYVY
    //capture.open("v4l2src device=/dev/video0 ! video/x-raw, width=1920, height=1080, format=UYVY, framerate=30/1 ! videoconvert ! video/x-raw, format=(string)BGR, width=1920,height=1080 ! appsink max-buffers=1 drop=false sync=false", cv::CAP_GSTREAMER);
    
    // cv_bridge

    //csi
    //capture.open("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1920,height=1080 ! nvvidconv flip-method=2 ! video/x-raw,width=1920,height=1080,format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);
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
        // RTSP stream
        if(USE_GST_RTSP){
            if(rtspqueue.getSize() < 5){
                msg *mp_rtsp = new msg;
                mp_rtsp->len = WIDTH*HEIGHT*3;
                mp_rtsp->data = new uint8_t[WIDTH*HEIGHT*3];
                memcpy(mp_rtsp->data, image, WIDTH*HEIGHT*3);
                pthread_mutex_lock(&rtspMutex);
                rtspqueue.push(mp_rtsp);
                pthread_mutex_unlock(&rtspMutex);
                pthread_cond_signal(&rtspCond);
            }
        }
        // GST display
        if(USE_GST_SHOW){
            msg *mp_gst = new msg;
            mp_gst->len = WIDTH*HEIGHT*3;
            mp_gst->data = new uint8_t[WIDTH*HEIGHT*3];
            memcpy(mp_gst->data, image, WIDTH*HEIGHT*3);
            pthread_mutex_lock(&gstMutex);
            gstqueue.push(mp_gst);
            pthread_mutex_unlock(&gstMutex);
            pthread_cond_signal(&gstCond);
        }
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
    while(!start_gst)
    {
        usleep(100);
    }
    GMainLoop *loop;
    GstRTSPServer *server;
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory;

    gst_init (NULL, NULL);
    loop = g_main_loop_new (NULL, FALSE);

    pthread_t m_rtsp;
    pthread_create(&m_rtsp, NULL, multirtsp, loop);
    server = gst_rtsp_server_new ();
    g_object_set (server, "service", port, NULL); 
    
    mounts = gst_rtsp_server_get_mount_points (server);
    factory = gst_rtsp_media_factory_new ();

    //SW encode
    //gst_rtsp_media_factory_set_launch (factory,
    //        "( appsrc name=videosrc is-live=true ! videoconvert ! video/x-raw, format=I420 ! x264enc bitrate=4000000 ! rtph264pay config-interval=10 name=pay0 pt=96 )");

    //HW encode
    //Profile Description 0 Baseline profile 2 Main profile 4 High profile
    //Hardware Preset Level 0 DisablePreset 1 UltraFastPreset 2 FastPreset
    gst_rtsp_media_factory_set_launch (factory,
            "( appsrc name=videosrc is-live=true ! videoconvert ! nvvidconv ! video/x-raw(memory:NVMM), format=I420 ! nvv4l2h264enc maxperf-enable=1 control-rate=0 bitrate=4000000 preset-level=2 profile=2 ! rtph264pay name=pay0 pt=96 sync=false )");

    gst_rtsp_media_factory_set_shared (factory, TRUE); 
    g_signal_connect (factory, "media-configure", (GCallback) media_configure, NULL);
    gst_rtsp_mount_points_add_factory (mounts, "/test", factory);
    g_object_unref (mounts);
    gst_rtsp_server_attach (server, NULL);
    /* start serving */
    g_print ("stream ready at rtsp://127.0.0.1:8554/test\n");
    pthread_join(m_rtsp, NULL);
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
    // GST show
    // if(USE_GST_SHOW){
    //     pthread_t thread_gst_show;
    //     pthread_create(&thread_gst_show, NULL, gst_show, NULL);
    // }

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