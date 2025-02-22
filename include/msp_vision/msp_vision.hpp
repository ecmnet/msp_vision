
#ifndef MSPVISION_IMAGE2RTSP_HPP
#define MSPVISION_IMAGE2RTSP_HPP

#include <msp_controller/msp_px4.h>

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

#include <msp_controller/msp_node_base.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/log_message.hpp>

namespace msp
{

  class MSPVisionNode : public msp::MSPNodeBase // public rclcpp::Node
  {
  public:
    explicit MSPVisionNode() : msp::MSPNodeBase("MSPVision", MSP_COMP_VISION)
    {

      setup_fonts();

      gz_node = std::make_unique<gz::transport::Node>();

      auto qos = this->getQos();

      battery_subscription_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
          "/msp/out/battery_status", qos, [this](const px4_msgs::msg::BatteryStatus::UniquePtr msg)
          {
            voltage_v = msg->voltage_v;
          });

      local_pos_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "msp/out/vehicle_local_position", qos, [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
          { altrel_m = msg->dist_bottom; });

      message_px4_subscription_ = this->create_subscription<px4_msgs::msg::LogMessage>(
          "msp/out/log_message", qos, [this](const px4_msgs::msg::LogMessage::UniquePtr msg)
          { 
            log_message_s.text = std::string(reinterpret_cast<const char*>(msg->text.data()));
            log_message_s.severity = msg->severity; 
            log_message_s.tms = this->get_clock()->now().nanoseconds() / 1000L;
            
         });

      message_msp_subscription_ = this->create_subscription<px4_msgs::msg::LogMessage>(
          "/msp/in/log_message", qos, [this](const px4_msgs::msg::LogMessage::UniquePtr msg)
          {
            log_message_s.text = std::string(reinterpret_cast<const char*>(msg->text.data()));
            log_message_s.severity = msg->severity; 
            log_message_s.tms = this->get_clock()->now().nanoseconds() / 1000L;
           });
     
      initialize();
    }

    ~MSPVisionNode()
    {
      RCLCPP_INFO(this->get_logger(), "Stream closed");
      GstRTSPSessionPool *pool;
      pool = gst_rtsp_server_get_session_pool(this->rtsp_server);
      g_object_unref(pool);
    }

    void initialize();
    void onStreamingStarted();
    void onStreamingStopped();

    void receive_msp_command(const std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> request,
                             std::shared_ptr<px4_msgs::srv::VehicleCommand::Response> response) override;

    GstRTSPServer *rtsp_server;

  private:
    struct message
    {
      std::string text;
      uint8_t severity;
      uint64 tms;
    };

    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_subscription_;
    rclcpp::Subscription<px4_msgs::msg::LogMessage>::SharedPtr message_px4_subscription_;
    rclcpp::Subscription<px4_msgs::msg::LogMessage>::SharedPtr message_msp_subscription_;

    std::unique_ptr<gz::transport::Node> gz_node;

    std::string source = "v4l2src device=/dev/video0";
    std::string mountpoint = "/stream";
    std::string bitrate = "400";
    std::string framerate = "30";
    std::string caps_1 = "video/x-raw, framerate =";
    std::string caps_2 = "/1,width=1280,height=760, alignment=au, udpsrc buffer-size=2048, rtsp_transport=udp, speed-preset=ultrafast,config-interval=1, latency=0";
    std::string port = "1051";
    std::string pipeline;
    std::string pipeline_head;
    std::string pipeline_tail;

    bool local_only = false;
    bool camera = false;
    bool is_streaming = true;

    float voltage_v = 0;
    float altrel_m = 0;

    message log_message_s;

    uint8_t stream = 0;

    const cv::Scalar color = cv::Scalar(255, 255, 255);

    cv::Ptr<cv::freetype::FreeType2> ft2;

    GstAppSrc *appsrc;

    void process_video_overlay(cv::Mat &image);
    void addOverlayItem(cv::Mat &roiImage, std::string header, std::string subtitle, int horizontal_pos);

    void setup_pipeline();
    void setup_fonts();

    void video_mainloop_start();
    void rtsp_server_add_url(const char *url, const char *sPipeline, GstElement **appsrc);
    void onImageReceived(const gz::msgs::Image &msg);
    void onDepthReceived(const gz::msgs::Image &msg);

    GstRTSPServer *rtsp_server_create(const std::string &port, const bool local_only);
    GstCaps *gst_caps_new_from_image(const cv::Mat image);
    void push_mat_to_gst(const cv::Mat image);
  };

  static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, GstElement **appsrc);
  static void media_teardown(GstRTSPMedia *media, gpointer user_data);
  static void *mainloop(void *arg);
  static gboolean session_cleanup(MSPVisionNode *node, rclcpp::Logger logger, gboolean ignored);

  class Formatter
  {
  public:
    // Static method to format time
    static std::string formatTime(float seconds)
    {
      const int minutes = static_cast<int>(seconds) / 60;
      const int wholeSeconds = static_cast<int>(seconds) % 60;
      const int firstMillisecondDigit = static_cast<int>((seconds - static_cast<int>(seconds)) * 10); // Extract only the first digit

      // Create a formatted string
      std::ostringstream oss;
      oss << std::setfill('0')
          << std::setw(2) << minutes << ":"
          << std::setw(2) << wholeSeconds << "."
          << firstMillisecondDigit;

      return oss.str();
    }

    static std::string formatDecimals(float value, int decimals, std::string unit)
    {
      std::ostringstream oss;
      oss.precision(decimals);
      oss << std::fixed << value << unit;
      return oss.str();
    }
  };

} // namespace msp

#endif