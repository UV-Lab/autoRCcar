#ifndef AUTOCAR_HARDWARE_CONTROL_HARDWARE_CONTROL_H_
#define AUTOCAR_HARDWARE_CONTROL_HARDWARE_CONTROL_H_

#include <memory>

#include "autorccar_interfaces/msg/control_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_port.h"

namespace autorccar {
namespace hardware_control {

constexpr int kEscPwmMin{3277};
constexpr int kEscPwmN{5039};
constexpr int kEscPwmMax{6553};
constexpr int kSteerPwmMin{10};   // 0 (margin 10)
constexpr int kSteerPwmN{87};     // Actual 90
constexpr int kSteerPwmMax{170};  // 180 (margin 10)

constexpr double kEscPwmSpeedGain{192.25};  // PWM per (m/s)
constexpr double kSteerPwmGain{3.18};       // PWM per degree

enum class DriveCommand { kStop = 0, kAuto, kManual };

struct ControlCommand {
    double speed{0.0};
    double steering_angle{0.0};
};

struct Pwm {
    int speed{0};
    int steering{0};
};

struct ControlResult {
    ControlCommand command;
    Pwm pwm;
};

struct Parameters {
    double max_speed{0.0};
    double max_steering_angle{0.0};
    std::string serial_port_name;
    int serial_baudrate{0};
    bool use_dummy_hardware{false};
};

class HardwareControl {
   public:
    HardwareControl(const HardwareControl&) = delete;
    HardwareControl& operator=(const HardwareControl&) = delete;
    HardwareControl(HardwareControl&&) = delete;
    HardwareControl& operator=(HardwareControl&&) = delete;

    explicit HardwareControl(const Parameters& parameters);

    void SetDriveCommand(const DriveCommand& drive_command);
    ControlResult SendControlCommand(ControlCommand& control_command);

   private:
    bool GotStopCommand() const;
    Pwm ConvertCommandToPwm(const ControlCommand& control_command) const;
    int SerializeAndSendMessage(DriveCommand cmd, const Pwm& pwm) const;
    void SendStopMessage() const;

    Parameters parameters_;
    std::unique_ptr<SerialPort> serial_;
    DriveCommand drive_command_{DriveCommand::kStop};
};

}  // namespace hardware_control
}  // namespace autorccar

#endif  // AUTOCAR_HARDWARE_CONTROL_HARDWARE_CONTROL_H_
