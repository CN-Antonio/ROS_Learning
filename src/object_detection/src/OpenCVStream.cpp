/**
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd
 * Author: Jacob Chen <jacob-chen@iotwrt.com>
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "OpenCVStream.h"
#include "PipeLines.h"

#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>

OpenCVStream::OpenCVStream()
{
    is_streaming__ = false;

    OpenCVFaceDect* face_dect = new OpenCVFaceDect();
    face_dect->Initialize("../blobs/haarcascade_frontalface_alt.xml");

    effect_lists.push_back(face_dect);
}

OpenCVStream::~OpenCVStream()
{
    if (is_streaming__) {
        StreamOFF();
    }
}

void OpenCVStream::StreamON()
{
    boost::lock_guard<boost::mutex> guard(state_mutex__);

    if (is_streaming__) {
        return;
    }

    // sink input from rtsp/cam/file
    sink_pipeline__ = new GstAppSinkPipeline();
    sink_pipeline__->Initialize(CreateAppSinkPipeline());
    sink_pipeline__->set_is_streaming(true);
    sink_pipeline__->SetPipelineState(GST_STATE_PLAYING);

    // src output to *
    src_pipeline__ = new GstAppSrcPipeline();
    src_pipeline__->Initialize(CreateAppSrcPipeline());
    src_pipeline__->SetPipelineState(GST_STATE_PLAYING);

    process_thread__ = boost::thread(&OpenCVStream::Process, this);
    is_streaming__ = true;
}

void OpenCVStream::StreamOFF()
{
    boost::lock_guard<boost::mutex> guard(state_mutex__);

    if (!is_streaming__) {
        return;
    }

    is_streaming__ = false;
    process_thread__.join();
    src_pipeline__->SetPipelineState(GST_STATE_PAUSED);
    sink_pipeline__->SetPipelineState(GST_STATE_PAUSED);
    sink_pipeline__->set_is_streaming(false);
    delete src_pipeline__;
    delete sink_pipeline__;
}

void OpenCVStream::Process()
{
    GstSample* sample;
    GstMapInfo map;
    GstStructure* s;
    GstBuffer* buffer;
    GstCaps* caps;
    GstMemory* mem;
    // RTSP
    GstRTSPServer *server;
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory;

    int height, width, size;
    int fd;
    void* map_data;

    while (is_streaming__) {
        if (sink_pipeline__->GetIsNewFrameAvailable()) {
            sink_pipeline__->GetLatestSample(&sample);
            
            // 这里都是为了拿到buffer
            caps = gst_sample_get_caps(sample);
            buffer = gst_sample_get_buffer(sample);
            s = gst_caps_get_structure(caps, 0);
            gst_structure_get_int(s, "height", &height);
            gst_structure_get_int(s, "width", &width);

            size = gst_buffer_get_size(buffer); // WxHxC
            /* Since gstreamer don't support map buffer to write,
             * we have to use mmap directly
             */
            mem = gst_buffer_peek_memory(buffer, 0);
            
            // 注意这里拿到dmabuf的fd啦！！！！！很重要
            // fd = gst_dmabuf_memory_get_fd(mem);
            // TODO: gst_is_dmabuf_memory (mem)检查mem错误，检查前步骤mem赋值
            
            // 为什么不直接用gstreamer里已经mmap过的地址？因为gstreamer有权限问题，有可能mmap成只读的了
            // 这里拿到buffer可读的地址了！！！！！！！fd就是这么转vaddr的
            // map_data = mmap64(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

            std::list<OpenCVEffect*>::iterator itor = effect_lists.begin();
            while (itor != effect_lists.end()) {
                /* assume BGR */
                (*itor)->Process1((void*)buffer, width, height);

                itor++;
            }

            // stream begin
            // /* create a server instance */
            // server = gst_rtsp_server_new ();
            // g_object_set (server, "service", "7551", NULL);

            // /* get the mount points for this server, every server has a default object
            // * that be used to map uri mount points to media factories */
            // mounts = gst_rtsp_server_get_mount_points (server);

            // //
            // /* make a media factory for a test stream. The default media factory can use
            // * gst-launch syntax to create pipelines.
            // * any launch line works as long as it contains elements named pay%d. Each
            // * element with pay%d names will be a stream */
            // factory = gst_rtsp_media_factory_new ();
            // gst_rtsp_media_factory_set_launch (factory, str);

            // /* attach the test factory to the /test url */
            // gst_rtsp_mount_points_add_factory (mounts, "/test", factory);

            // stream end

            // post process
            // munmap(map_data, size);
            /* will auto released */
            gst_buffer_ref(buffer);
            src_pipeline__->SendBUF(buffer);
            sink_pipeline__->ReleaseFrameBuffer();
            /* g_print("%s\n", gst_caps_to_string(caps)); */
        }
    }
}

//gst_sample_ref(&sample);
//gst_sample_unref(&sample);