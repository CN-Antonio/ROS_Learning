//
#include "ros/ros.h"
#include "bits/stdc++.h"
// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/types_c.h>
// #include <opencv2/imgproc/imgproc.hpp>
// ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// Gstreamer
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#define DEFAULT_RTSP_PORT "7551"

// OpenCV-bridge
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
 
    // // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
 
    // Update GUI Window
    cv::imshow("OPENCV_WINDOW", cv_ptr->image);
    cv::waitKey(3);
}

// RTSP
static char *port = (char *) DEFAULT_RTSP_PORT;
static GOptionEntry entries[] = {
  {"port", 'p', 0, G_OPTION_ARG_STRING, &port,
      "Port to listen on (default: " DEFAULT_RTSP_PORT ")", "PORT"},
  {NULL}
};

int main(int argc, char *argv[])
{
  // declare object
  GMainLoop *loop;
  GstRTSPServer *server;
  GstRTSPMountPoints *mounts;
  GstRTSPMediaFactory *factory;
  GOptionContext *optctx;
  GError *error = NULL;
  gchar *str;
  // ROS
  ros::init(argc, argv, "RTSP_Streamer");
  ros::NodeHandle node_handle;
  // cv_bridge
  image_transport::ImageTransport image_transport(node_handle);
  image_transport::Subscriber image_sub = image_transport.subscribe("/camera/image_color_RAW", 1, imageCb);

  std::cout << "RTSP_Streamer" << std::endl << std::endl;

  optctx = g_option_context_new ("<launch line> - Test RTSP Server, Launch\n\n"
      "Example: \"( videotestsrc ! x264enc ! rtph264pay name=pay0 pt=96 )\"");
  g_option_context_add_main_entries (optctx, entries, NULL);
  g_option_context_add_group (optctx, gst_init_get_option_group ());
  if (!g_option_context_parse (optctx, &argc, &argv, &error)) {
    g_printerr ("Error parsing options: %s\n", error->message);
    g_option_context_free (optctx);
    g_clear_error (&error);
    return -1;
  }
  g_option_context_free (optctx);

  loop = g_main_loop_new (NULL, FALSE);

  /* create a server instance */
  server = gst_rtsp_server_new ();
  g_object_set (server, "service", port, NULL);

  /* get the mount points for this server, every server has a default object
   * that be used to map uri mount points to media factories */
  mounts = gst_rtsp_server_get_mount_points (server);

  str = g_strdup_printf("("
    " v4l2src device=/dev/video0 io-mode=2 "
    " ! image/jpeg,width=1920,height=1080,framerate=30/1,format=MJPG "
    " ! nvjpegdec "
    " ! clockoverlay halignment=right valignment=bottom text=\"JYB1034\" shaded-background=true font-desc=\"Sans, 18\" "
    " ! video/x-raw ! nvvidconv "
    " ! video/x-raw(memory:NVMM),format=I420,width=1920,height=1080,framerate=30/1 "
    " ! nvv4l2h264enc MeasureEncoderLatency=1 maxperf-enable=1 profile=4 preset-level=1 iframeinterval=500 control-rate=1 bitrate=2000000 "
    " ! rtph264pay name=pay0 pt=96 "
    " )");

  /* make a media factory for a test stream. The default media factory can use
   * gst-launch syntax to create pipelines.
   * any launch line works as long as it contains elements named pay%d. Each
   * element with pay%d names will be a stream */
  factory = gst_rtsp_media_factory_new ();
  gst_rtsp_media_factory_set_launch (factory, str);

  /* attach the test factory to the /test url */
  gst_rtsp_mount_points_add_factory (mounts, "/test", factory);

  /* don't need the ref to the mapper anymore */
  g_object_unref (mounts);

  /* attach the server to the default maincontext */
  gst_rtsp_server_attach (server, NULL);

  /* start serving */
  g_print ("stream ready at rtsp://127.0.0.1:%s/test\n", port);
  g_main_loop_run (loop);

  return 0;
}