#ifndef AUTOCAR_HARDWARE_SERIAL_PORT_H_
#define AUTOCAR_HARDWARE_SERIAL_PORT_H_

#include <termios.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace autorccar {
namespace hardware_control {

class SerialPort {
   public:
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    explicit SerialPort(const std::string& port_name, int baudrate);
    ~SerialPort();

    SerialPort(SerialPort&& other) noexcept;
    SerialPort& operator=(SerialPort&& other) noexcept;

    ssize_t Write(const uint8_t* data, size_t size) const;

   private:
    speed_t GetBaudrate(int baudrate);

    int fd_{-1};
};

}  // namespace hardware_control
}  // namespace autorccar

#endif  // AUTOCAR_HARDWARE_SERIAL_PORT_H_
