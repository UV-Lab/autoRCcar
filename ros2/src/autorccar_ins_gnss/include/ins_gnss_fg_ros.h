#ifndef AUTORCCAR_INS_GNSS__INS_GNSS_FG_ROS_H
#define AUTORCCAR_INS_GNSS__INS_GNSS_FG_ROS_H

#include <memory>

#include "autorccar_interfaces/msg/gnss.hpp"
#include "autorccar_interfaces/msg/imu.hpp"
#include "autorccar_interfaces/msg/nav_state.hpp"
#include "ins_gnss_fg.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class InsGnssFgRos : public rclcpp::Node {
   public:
    explicit InsGnssFgRos(InsGnssFg* pFg);

   private:
    rclcpp::Subscription<autorccar_interfaces::msg::Imu>::SharedPtr  imu_sub_;
    rclcpp::Subscription<autorccar_interfaces::msg::Gnss>::SharedPtr gnss_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr          yaw_sub_;
    rclcpp::Publisher<autorccar_interfaces::msg::NavState>::SharedPtr nav_pub_;

    InsGnssFg* mpFg_;
    NavSol*    mpSol_;

    void ImuCallback(const autorccar_interfaces::msg::Imu& msg);
    void GnssCallback(const autorccar_interfaces::msg::Gnss& msg);
    void YawCallback(const std_msgs::msg::Float32& msg);
    void PublishNavSol();
};

#endif
