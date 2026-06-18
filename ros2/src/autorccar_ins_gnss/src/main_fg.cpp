#include <memory>

#include "ins_gnss_fg.h"
#include "ins_gnss_fg_ros.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("ins_gnss_fg"),
                     "Usage: ins_gnss_fg <config_file>");
        return -1;
    }

    std::string config_file(argv[1]);

    auto fg   = std::make_unique<InsGnssFg>(config_file);
    auto node = std::make_shared<InsGnssFgRos>(fg.get());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
