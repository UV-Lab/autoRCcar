#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ekf.h"
#include "ekf_ros_wrapper.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_FATAL(rclcpp::get_logger("ins_gnss"),
                     "No config file provided. Usage: ros2 run autorccar_ins_gnss ins_gnss <config_file>");
        rclcpp::shutdown();
        return 1;
    }

    // Create EKF based navigation system.
    std::string config = argv[1];

    EKF nav(config);

    auto node = std::make_shared<EKFWrapper>(&nav);
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}