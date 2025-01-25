#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include <rclcpp/rclcpp.hpp>
#include <gz/msgs/image.pb.h>
#include <msp_vision/msp_vision.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

void msp::MSPVisionNode::setup_pipeline()
{

  video_mainloop_start();
  rtsp_server = rtsp_server_create(port, local_only);

  appsrc = NULL;

  // Setup the pipeline
  pipeline_tail = "key-int-max=30 ! video/x-h264, profile=baseline ! rtph264pay name=pay0 pt=96 )";
  if (camera == false)
  {
    pipeline_head =
        "( appsrc name=imagesrc do-timestamp=true min-latency=0 max-latency=0 max-bytes=1000 is-live=true ! "
        "videoconvert ! videoscale ! ";
    pipeline =
        pipeline_head + caps_1 + framerate + caps_2 + " ! x264enc tune=zerolatency bitrate=" + bitrate + pipeline_tail;
    rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(), (GstElement **)&(appsrc));
  }
  else
  {
    pipeline = "( " + source + " ! videoconvert ! videoscale ! " + caps_1 + framerate + caps_2 +
               " ! x264enc tune=zerolatency bitrate=" + bitrate + pipeline_tail;
    rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(), NULL);
  }
}

static void *msp::mainloop(void *arg)
{
  GMainLoop *loop = g_main_loop_new(NULL, FALSE);
  g_main_loop_run(loop);
  g_main_destroy(loop);
  return NULL;
}

void msp::MSPVisionNode::video_mainloop_start()
{
  pthread_t tloop;
  gst_init(NULL, NULL);
  pthread_create(&tloop, NULL, &mainloop, NULL);
}

GstRTSPServer *msp::MSPVisionNode::rtsp_server_create(const std::string &port, const bool local_only)
{
  GstRTSPServer *server;

  /* create a server instance */
  server = gst_rtsp_server_new();
  // char *port = (char *) port;
  g_object_set(server, "service", port.c_str(), NULL);
  /* attach the server to the default maincontext */
  if (local_only)
  {
    g_object_set(server, "address", "127.0.0.1", NULL);
  }
  gst_rtsp_server_attach(server, NULL);
  /* add a timeout for the session cleanup */
  g_timeout_add_seconds(1, (GSourceFunc)session_cleanup, this);
  return server;
}

void msp::MSPVisionNode::rtsp_server_add_url(const char *url, const char *sPipeline, GstElement **appsrc)
{
  GstRTSPMountPoints *mounts;
  GstRTSPMediaFactory *factory;

  /* get the mount points for this server, every server has a default object
   * that be used to map uri mount points to media factories */
  mounts = gst_rtsp_server_get_mount_points(rtsp_server);

  /* make a media factory for a test stream. The default media factory can use
   * gst-launch syntax to create pipelines.
   * any launch line works as long as it contains elements named pay%d. Each
   * element with pay%d names will be a stream */
  factory = gst_rtsp_media_factory_new();
  gst_rtsp_media_factory_set_launch(factory, sPipeline);

  /* notify when our media is ready, This is called whenever someone asks for
   * the media and a new pipeline is created */
  g_signal_connect(factory, "media-configure", (GCallback)media_configure, appsrc);

  gst_rtsp_media_factory_set_shared(factory, TRUE);

  /* attach the factory to the url */
  gst_rtsp_mount_points_add_factory(mounts, url, factory);

  /* don't need the ref to the mounts anymore */
  g_object_unref(mounts);
}

static void msp::media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, GstElement **appsrc)
{
  if (appsrc)
  {
    GstElement *pipeline = gst_rtsp_media_get_element(media);

    *appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "imagesrc");

    /* this instructs appsrc that we will be dealing with timed buffer */
    gst_util_set_object_arg(G_OBJECT(*appsrc), "format", "time");

    gst_object_unref(pipeline);
  }
  else
  {
    guint i, n_streams;
    n_streams = gst_rtsp_media_n_streams(media);

    for (i = 0; i < n_streams; i++)
    {
      GstRTSPAddressPool *pool;
      GstRTSPStream *stream;
      gchar *min, *max;

      stream = gst_rtsp_media_get_stream(media, i);

      /* make a new address pool */
      pool = gst_rtsp_address_pool_new();

      min = g_strdup_printf("224.3.0.%d", (2 * i) + 1);
      max = g_strdup_printf("224.3.0.%d", (2 * i) + 2);
      gst_rtsp_address_pool_add_range(pool, min, max, 5000 + (10 * i), 5010 + (10 * i), 1);
      g_free(min);
      g_free(max);
      gst_rtsp_stream_set_address_pool(stream, pool);
      g_object_unref(pool);
    }
  }
}

void msp::MSPVisionNode::push_mat_to_gst(const cv::Mat image)
{
  GstBuffer *buf;
  GstCaps *caps;

  if (appsrc != NULL)
  {
    int size = image.total() * image.elemSize();
    caps = msp::MSPVisionNode::gst_caps_new_from_image(image);
    gst_app_src_set_caps(appsrc, caps);
    buf = gst_buffer_new_allocate(nullptr, size, nullptr);
    gst_buffer_fill(buf, 0, image.data, size);
    GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);
    gst_app_src_push_buffer(appsrc, buf);
  }
}

GstCaps *msp::MSPVisionNode::gst_caps_new_from_image(const cv::Mat image)
{
  std::string format;

  switch (image.type())
  {
  case CV_8UC1:
    format = "GREY8";
    break;
  case CV_16UC1:
    format = "GRAY16_LE";
    break;
  case CV_8UC3:
    format = "RGB";
    break;
  default:
    format = "RGB";
  }

  return gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, format.c_str(), "width", G_TYPE_INT, image.cols,
                             "height", G_TYPE_INT, image.rows, "framerate", GST_TYPE_FRACTION, stoi(framerate), 1,
                             "stream-format", G_TYPE_STRING, "byte-stream", nullptr);
}

static gboolean msp::session_cleanup(msp::MSPVisionNode *node, rclcpp::Logger logger, gboolean ignored)
{
  GstRTSPServer *server = node->rtsp_server;
  GstRTSPSessionPool *pool;
  int num;

  pool = gst_rtsp_server_get_session_pool(server);
  num = gst_rtsp_session_pool_cleanup(pool);
  g_object_unref(pool);

  if (num > 0)
  {
    char s[32];
    snprintf(s, 32, (char *)"Sessions cleaned: %d", num);
    RCLCPP_INFO(node->get_logger(), s);
  }
  return TRUE;
}
