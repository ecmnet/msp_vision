
#include <msp_vision/msp_vision.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>
#include <sstream>

using namespace msp;

void MSPVisionNode::setup_fonts()
{
    try
    {
        ft2 = cv::freetype::createFreeType2();
        ft2->loadFontData("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 0); // Load a TTF font
    }
    catch (cv::Exception &e)
    {
        std::cerr << "Failed to load FreeType: " << e.what() << std::endl;
    }
}

void MSPVisionNode::process_video_overlay(cv::Mat &image)
{
    cv::Rect roi;
    if (image.rows < 500)
        roi = cv::Rect(0, image.rows - 50, image.cols, 50);
    else
        roi = cv::Rect(0, image.rows - 80, image.cols, 80);

    cv::Mat roiImage = image(roi);
    cv::GaussianBlur(roiImage, roiImage, cv::Size(15, 15), 0);

    if (this->armed_time_ > 0)
        addOverlayItem(roiImage,
                       msp::Formatter::formatTime((this->get_clock()->now().nanoseconds() / 1000L - this->armed_time_) / 1e6f),
                       "armed", 0);
    else
        addOverlayItem(roiImage, "00:00.0", "disarmed", 0);

    addOverlayItem(roiImage, msp::Formatter::formatDecimals(voltage_v, 1, "V"), "battery", 150);
    addOverlayItem(roiImage, msp::Formatter::formatDecimals(altrel_m, 1, "m"), "alt.rel", 270);
    
    if (log_message_s.tms > this->get_clock()->now().nanoseconds() / 1000L - 3000000)
        addOverlayItem(roiImage, log_message_s.text, "info", 390);
    else
        addOverlayItem(roiImage, "", "info", 390);
}

void MSPVisionNode::addOverlayItem(cv::Mat &roiImage, std::string header, std::string subtitle, int item_index)
{

    const int horizontal_pos = item_index * roiImage.cols / 1280 + 7;

    if (roiImage.rows < 80)
    {

        ft2->putText(roiImage, header, cv::Point(horizontal_pos + 6, 25), 15, color, -1, cv::LINE_AA, true);
        ft2->putText(roiImage, subtitle, cv::Point(horizontal_pos + 6, 40), 12, color, -1, cv::LINE_AA, true);
        cv::line(roiImage, cv::Point(horizontal_pos, 12), cv::Point(horizontal_pos, roiImage.rows - 10), color, 1, cv::LINE_AA);
    }
    else
    {

        ft2->putText(roiImage, header, cv::Point(horizontal_pos + 10, 40), 35, color, -1, cv::LINE_AA, true);
        ft2->putText(roiImage, subtitle, cv::Point(horizontal_pos + 10, 65), 22, color, -1, cv::LINE_AA, true);
        cv::line(roiImage, cv::Point(horizontal_pos, 15), cv::Point(horizontal_pos, roiImage.rows - 15), color, 1, cv::LINE_AA);
    }
}
