#include <algorithm>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "api/withrobot_camera.hpp" /* withrobot camera API */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

bool find_v4l_device_path(std::string &devPath) {
    if (!std::filesystem::exists("/dev/v4l/by-id")) return false;

    // Withrobot camera id would be like
    // "usb-WITHROBOT_Inc._oCam-1CGN-U-T_SN_35E27013-video-index0"
    std::vector<std::string> paths;
    for (const auto &entry : std::filesystem::directory_iterator("/dev/v4l/by-id")) {
        if (entry.is_character_file() && (entry.path().filename().string().find("1CGN-U-T") != std::string::npos)) {
            auto path = entry.path().parent_path();
            path /= std::filesystem::read_symlink(entry.path());
            path = std::filesystem::canonical(path);
            paths.push_back(path);
        }
    }

    if (paths.empty()) return false;

    // Singel camera can contain two video pahts, normally ealier one gives
    // image
    std::sort(paths.begin(), paths.end());
    devPath = paths.front();
    return true;
}

class ImagePublisher : public rclcpp::Node {
   public:
    ImagePublisher() : Node("oCam_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 1);
    }

    void initialize_config() {
        declare_parameter("device_path", "");  // Blank for auto detection

        /*
         * oCam-1CGN supported image formats
         * USB 3.0
         * 	[0] "8-bit Greyscale 1280 x 720 60 fps"
         *	[1] "8-bit Greyscale 1280 x 960 45 fps"
         *	[2] "8-bit Greyscale 320 x 240 160 fps"
         * 	[3] "8-bit Greyscale 640 x 480 80 fps"
         *
         * USB 2.0
         * 	[0] "8-bit Greyscale 1280 x 720 30 fps"
         *	[1] "8-bit Greyscale 1280 x 960 22.5 fps"
         *	[2] "8-bit Greyscale 320 x 240 160 fps"
         * 	[3] "8-bit Greyscale 640 x 480 80 fps"
         */
        // native_downsample: 1280x960 GRBG -> 640x480 rgb8.
        // hardware_binned: 640x480 GRBG -> 640x480 rgb8.
        declare_parameter("capture_mode", "native_downsample");
        declare_parameter("OutputWidth", 640);
        declare_parameter("OutputHeight", 480);
        declare_parameter("FPS", 30);
        declare_parameter("frame_timeout_sec", 5.0);
        declare_parameter("frame_id", "ocam_optical_frame");

        // See v4l2-ctl -d[#] --all to check control options
        // "Gain" (range reported by this camera: 0 to 255)
        declare_parameter("Brightness", 128);
        // "Exposure (Absolute)", (default[min, step, max]) : 39(39 [1, 1, 625])
        declare_parameter("Exposure", 128);
        declare_parameter("Auto exposure mode",
                          1);  // 1 - Manual mode, 3 - Apeture priority mode

        frame_id_ = get_parameter("frame_id").as_string();
    }

    void publish_image(const cv::Mat &img) {
        sensor_msgs::msg::Image msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;
        msg.height = img.rows;
        msg.width = img.cols;
        msg.step = img.cols * img.elemSize();
        msg.encoding = "rgb8";

        const uint32_t size = img.total() * img.elemSize();
        msg.data.assign(img.data, img.data + size);

        publisher_->publish(msg);
    }

   private:
    std::string frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};
