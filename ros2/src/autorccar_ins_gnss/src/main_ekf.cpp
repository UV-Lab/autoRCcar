#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ekf.h"
#include "ekf_ros_wrapper.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create EKF based navigation system.
    std::string config = argv[1];

    EKF nav(config);

    auto node = std::make_shared<EKFWrapper>(&nav);
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}