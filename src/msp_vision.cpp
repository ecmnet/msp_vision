

#include <msp_vision/msp_vision.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/battery_status.hpp>

#define MAX_DEPTH_M 20.0;

using namespace msp;

void MSPVisionNode::start()
{
  
  setup_pipeline();

  RCLCPP_INFO(this->get_logger(), "Stream available at rtsp://%s:%s%s", gst_rtsp_server_get_address(rtsp_server),
              port.c_str(), mountpoint.c_str());

  gz_node->Subscribe("/depth_camera", &MSPVisionNode::onDepthReceived, this);
  gz_node->Subscribe("/camera", &MSPVisionNode::onImageReceived, this);
 
}

void MSPVisionNode::receive_msp_command(const std::shared_ptr<px4_msgs::srv::VehicleCommand::Request> request,
                                        std::shared_ptr<px4_msgs::srv::VehicleCommand::Response> response)
{
  switch (request->request.command)
  {
  case MSP_CMD::SELECT_VIDEO_STREAM:
    stream = int(request->request.param1);
    break;
  }
}

void MSPVisionNode::onStreamingStarted() {
  RCLCPP_INFO(this->get_logger(), "Streaming started");
  is_streaming = true;
}

void MSPVisionNode::onStreamingStopped() {
  RCLCPP_INFO(this->get_logger(), "Streaming stopped");
  is_streaming = false;
}

void MSPVisionNode::onDepthReceived(const gz::msgs::Image &msg)
{
  if (stream != 1 || !is_streaming) 
    return;

  cv::Mat image(msg.height(), msg.width(), CV_32FC1, (void *)msg.data().data());
  cv::Mat depthNormalized;

  double maxVal = 255 / MAX_DEPTH_M;
  image.convertTo(depthNormalized, CV_8UC1, maxVal, 0);
  cv::Mat bgrImage;
  cv::applyColorMap(depthNormalized, bgrImage, cv::COLORMAP_VIRIDIS);
  process_video_overlay(bgrImage);

  push_mat_to_gst(bgrImage);
}

void MSPVisionNode::onImageReceived(const gz::msgs::Image &msg)
{
  if (stream != 0 || !is_streaming)
    return;

  cv::Mat image(msg.height(), msg.width(), CV_8UC3, (void *)msg.data().data());
  process_video_overlay(image);

  push_mat_to_gst(image);
}


int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<msp::MSPVisionNode>());
  rclcpp::shutdown();
  return 0;
}