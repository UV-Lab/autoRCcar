#include "api/withrobot_camera.hpp"

#include <chrono>
#include <fstream>
#include <stdexcept>
#include <thread>

namespace {
std::string scenario() {
    const char* value = std::getenv("OCAM_TEST_SCENARIO");
    return value ? value : "frames";
}

void event(const char* name) {
    std::ofstream(std::getenv("OCAM_TEST_EVENTS"), std::ios::app) << name << '\n';
}
int starts = 0;
int frames = 0;
}

namespace Withrobot {
Camera::Camera(std::string name, camera_format*, const char*, unsigned char)
    : dev_name(std::move(name)), v4l2_s{} {}
Camera::~Camera() { event("closed"); }

bool Camera::set_format(unsigned int width, unsigned int height, unsigned int format,
                        unsigned int numerator, unsigned int denominator) {
    config.width = width;
    config.height = height;
    config.image_size = width * height;
    config.pixformat = format;
    config.rate_numerator = numerator;
    config.rate_denominator = denominator;
    return true;
}
bool Camera::get_current_format(camera_format& format) { format = config; return true; }
bool Camera::set_control(const char*, int) { return true; }
int Camera::get_control(const char* name) {
    return std::string(name) == "Auto Exposure" ? 1 : 128;
}
std::string Camera::get_serial_number() { return "test-camera"; }

bool Camera::start() {
    event("start");
    ++starts;
    return scenario() != "start_failure" && !(scenario() == "restart_failure" && starts > 1);
}
bool Camera::stop() {
    event("stop");
    return scenario() != "stop_failure" && scenario() != "recovery_stop_failure";
}

int Camera::get_frame(unsigned char* output, unsigned int size, unsigned int) {
    event("read");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ++frames;
    if (scenario() == "exception") throw std::runtime_error("test frame failure");
    if (scenario() == "partial") return size / 2;
    if (scenario() == "frames" || scenario() == "stop_failure" ||
        (scenario() == "frames_then_timeout" && frames <= 20) ||
        (scenario() == "transient" && frames % 3 == 0)) {
        std::memset(output, 128, size);
        event("frame");
        return size;
    }
    errno = scenario() == "interrupted" ? EINTR : ETIMEDOUT;
    return -1;
}
}
