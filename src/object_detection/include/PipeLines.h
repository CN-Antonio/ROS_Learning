#ifndef _PIPELINE_H
#define _PIPELINE_H

// sink
#define TEST_DEMO_SINK
// #define TEST_VIDEO
// #define TEST_RTSP
// #define TEST_CAMERA

// src
#define DISP_CLOCK
#define TEST_IMAGESINK_SRC
// #define TEST_RTSP_SRC

#ifdef TEST_DEMO_SINK
static std::string CreateAppSinkPipeline()
{
    std::stringstream pipelineString;

    pipelineString
        << "videotestsrc"
        << LINK
        << "videoconvert"
        << LINK
        << "video/x-raw, format=(string)BGR, width=1280, height=720, framerate=30/1"
        << LINK
        << "appsink caps=video/x-raw,format=BGR name="
        << APPSINK_NAME;

    return pipelineString.str();

}
#elif defined TEST_CAMERA
static std::string CreateAppSinkPipeline()
{
    std::stringstream pipelineString;

    pipelineString
        << "v4l2src device=/dev/video0"
        << LINK
        << "image/jpeg,width=1920,height=1080,framerate=30/1,format=MJPG"
        << LINK
        << "nvjpegdec"
        << LINK
        << "video/x-raw,format=(string)BGR,width=(int)1920,height=(int)1080,framerate=(fraction)30/1"
        << LINK
        << "appsink caps=video/x-raw,format=BGR name="
        << APPSINK_NAME;

    return pipelineString.str();
}
#elif defined TEST_VIDEO
static std::string CreateAppSinkPipeline()
{
    std::stringstream pipelineString;

    pipelineString
        << "filesrc location=/home/antonio/Downloads/309254773-1-208.mp4"
        << LINK
        << "qtdemux"
        << "h264parse"
        << LINK
        << "nvv4l2decoder enable-max-performance=1 drop-frame-interval=1"
        << LINK
        << "nvvidconv"
        << LINK
        << "video/x-raw,format=I420,width=1920,height=1080"
        << LINK
        << "video/x-raw,format=BGR,width=(int)1920,height=(int)1080"
        << LINK
        << "videoconvert"
        << LINK
        << "appsink caps=video/x-raw,format=BGR name="
        << APPSINK_NAME;

    return pipelineString.str();
}
#elif defined TEST_RTSP
static std::string CreateAppSinkPipeline()
{
    std::stringstream pipelineString;

    pipelineString
        << "rtsp://192.168.31.163:554/"
        << LINK
        << "h264parse"
        << LINK
        << "mppvideodec"
        << LINK
        << "video/x-raw,format=(string)NV12"
        << LINK
        << "rgaconvert output-io-mode=dmabuf-import capture-io-mode=dmabuf vpu-stride=true"
        << LINK
        << "video/x-raw,format=BGR,width=(int)1920,height=(int)1080"
        << LINK
        << "appsink caps=video/x-raw,format=BGR name="
        << APPSINK_NAME;

    return pipelineString.str();
}
#endif

#ifdef TEST_IMAGESINK_SRC
static std::string CreateAppSrcPipeline()
{
    std::stringstream pipelineString;

    pipelineString
        << "appsrc caps=video/x-raw,format=(string)BGR,width=(int)1280,height=(int)720,framerate=(fraction)30/1 "
           "block=true name="
        << APPSRC_NAME
        << LINK
#ifdef DISP_CLOCK
        << "clockoverlay halignment=right valignment=bottom text=\"JYB1034\" shaded-background=true font-desc=\"Sans, 18\" "
        << LINK
#endif
        << "videoconvert"
        << LINK
        << "video/x-raw,format=I420,width=1280,height=720"
        << LINK
        << "xvimagesink sync=false";

    return pipelineString.str();
}
#elif defined TEST_RTSP_SRC
static std::string CreateAppSrcPipeline()
{
    std::stringstream pipelineString;

    pipelineString
        << "appsrc caps=video/x-raw,format=(string)BGR,width=(int)1280,height=(int)720,framerate=(fraction)30/1 "
           "block=true name="
        << APPSRC_NAME
        << LINK
        << "videoconvert"
        << LINK
        << "video/x-raw,format=I420,width=1280,height=720"
        << LINK
        << "video/x-raw(memory:NVMM),format=I420,width=1920,height=1080,framerate=30/1"
        << LINK
        << "nvv4l2h264enc MeasureEncoderLatency=1 maxperf-enable=1 profile=4 preset-level=1 iframeinterval=500 control-rate=1 bitrate=2000000"
        << LINK
        << "rtph264pay name=pay0 pt=96";

    return pipelineString.str();
}
#endif

#endif
