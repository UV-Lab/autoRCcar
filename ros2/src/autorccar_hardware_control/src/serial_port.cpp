#include "serial_port.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <stdexcept>
#include <string>

namespace autorccar {
namespace hardware_control {

SerialPort::SerialPort(const std::string& port_name, int baudrate) {
    int fd{open(port_name.c_str(), O_RDWR | O_NOCTTY)};
    if (fd < 0) {
        throw std::runtime_error("Failed to open serial port");
    }

    // Get current serial port settings
    struct termios options{};
    if (tcgetattr(fd, &options) != 0) {
        close(fd);
        throw std::runtime_error("tcgetattr failed");
    }

    // Set baud both ways
    cfsetispeed(&options, GetBaudrate(baudrate));
    cfsetospeed(&options, GetBaudrate(baudrate));

    // 8 bits, no parity, no stop bits
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // Canonical mode
    options.c_lflag |= ICANON;

    // Commit the serial port settings
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        close(fd);
        throw std::runtime_error("tcsetattr failed");
    }

    fd_ = fd;

    std::cout << "[" << port_name << "] opened as " << fd_ << std::endl;
}

SerialPort::~SerialPort() {
    if (fd_ >= 0) {
        close(fd_);
    }
}

SerialPort::SerialPort(SerialPort&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; }

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
    if (this != &other) {
        if (fd_ >= 0) {
            close(fd_);
        }
        fd_ = other.fd_;
        other.fd_ = -1;
    }
    return *this;
}

ssize_t SerialPort::Write(const uint8_t* data, size_t size) const {
    ssize_t ret{write(fd_, data, size)};
    if (ret < 0) {
        throw std::runtime_error("write failed");
    }
    return ret;
}

speed_t SerialPort::GetBaudrate(int baudrate) {
    switch (baudrate) {
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        default:
            throw std::runtime_error("Unsupported baudrate");
    }
}

}  // namespace hardware_control
}  // namespace autorccar
