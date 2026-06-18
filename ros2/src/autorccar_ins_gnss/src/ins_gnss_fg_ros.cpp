#include "ins_gnss_fg_ros.h"

#include <eigen3/Eigen/Dense>

#include "ins_toolbox.h"
#include "rclcpp/rclcpp.hpp"

InsGnssFgRos::InsGnssFgRos(InsGnssFg* pFg)
    : Node("ins_gnss_fg"), mpFg_(pFg), mpSol_(nullptr) {
    imu_sub_ = this->create_subscription<autorccar_interfaces::msg::Imu>(
        "IMU", 10,
        std::bind(&InsGnssFgRos::ImuCallback, this, std::placeholders::_1));

    gnss_sub_ = this->create_subscription<autorccar_interfaces::msg::Gnss>(
        "GNSS", 10,
        std::bind(&InsGnssFgRos::GnssCallback, this, std::placeholders::_1));

    yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "setyaw_topic", 10,
        std::bind(&InsGnssFgRos::YawCallback, this, std::placeholders::_1));

    nav_pub_ = this->create_publisher<autorccar_interfaces::msg::NavState>(
        "nav_topic", 10);
}

void InsGnssFgRos::ImuCallback(const autorccar_interfaces::msg::Imu& msg) {
    double t   = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9;
    Eigen::Vector3d acc(msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z);
    Eigen::Vector3d gyro(msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z);

    if (!mpFg_->is_time_set) {
        mpFg_->SetTimeInit(t);
    }

    mpSol_ = mpFg_->UpdateImu(t, acc, gyro);

    if (mpSol_ != nullptr) {
        PublishNavSol();
    }
}

void InsGnssFgRos::GnssCallback(const autorccar_interfaces::msg::Gnss& msg) {
    double t = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9;
    Eigen::Vector3d pos_ecef(msg.position_ecef.x,
                              msg.position_ecef.y,
                              msg.position_ecef.z);
    Eigen::Vector3d vel_ecef(msg.velocity_ecef.x,
                              msg.velocity_ecef.y,
                              msg.velocity_ecef.z);

    mpSol_ = mpFg_->UpdateGnss(t, pos_ecef, vel_ecef);

    if (mpSol_ != nullptr) {
        PublishNavSol();
    }
}

void InsGnssFgRos::YawCallback(const std_msgs::msg::Float32& msg) {
    mpFg_->SetYawInit(msg.data);
}

void InsGnssFgRos::PublishNavSol() {
    if (mpSol_ == nullptr) return;

    Eigen::Matrix<double, 20, 1> x = mpSol_->GetNavRosMsg();

    // Convert NED → ENU if requested
    if (mpFg_->GetNavFrame() == "ENU") {
        static const Eigen::Matrix3d Tn2e =
            (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();
        x.block<3, 1>(4, 0) = Tn2e * x.block<3, 1>(4, 0);  // position
        x.block<3, 1>(7, 0) = Tn2e * x.block<3, 1>(7, 0);  // velocity

        Eigen::Quaterniond att_ned;
        att_ned.w() = x(10);
        att_ned.x() = x(11);
        att_ned.y() = x(12);
        att_ned.z() = x(13);
        // NED → ENU quaternion conversion: swap x/y, negate z
        x(10) = att_ned.w();
        x(11) = att_ned.y();
        x(12) = att_ned.x();
        x(13) = -att_ned.z();
    }

    auto msg_out = autorccar_interfaces::msg::NavState();
    msg_out.timestamp.sec     = static_cast<int>(x(0));
    msg_out.timestamp.nanosec = static_cast<uint32_t>((x(0) - static_cast<int>(x(0))) * 1e9);
    msg_out.origin.x          = x(1);
    msg_out.origin.y          = x(2);
    msg_out.origin.z          = x(3);
    msg_out.position.x        = x(4);
    msg_out.position.y        = x(5);
    msg_out.position.z        = x(6);
    msg_out.velocity.x        = x(7);
    msg_out.velocity.y        = x(8);
    msg_out.velocity.z        = x(9);
    msg_out.quaternion.w      = x(10);
    msg_out.quaternion.x      = x(11);
    msg_out.quaternion.y      = x(12);
    msg_out.quaternion.z      = x(13);
    msg_out.acceleration.x    = x(14);
    msg_out.acceleration.y    = x(15);
    msg_out.acceleration.z    = x(16);
    msg_out.angular_velocity.x = x(17);
    msg_out.angular_velocity.y = x(18);
    msg_out.angular_velocity.z = x(19);

    nav_pub_->publish(msg_out);
}
