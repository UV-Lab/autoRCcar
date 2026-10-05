#pragma once

#include <optional>
#include <string_view>

namespace ocam {

struct CaptureConfiguration {
    int width;
    int height;
    bool hardware_binned;
};

inline std::optional<CaptureConfiguration> capture_config_for_mode(std::string_view mode) {
    if (mode == "native_downsample") {
        return CaptureConfiguration{1280, 960, false};
    }
    if (mode == "hardware_binned") {
        return CaptureConfiguration{640, 480, true};
    }
    return std::nullopt;
}

}  // namespace ocam
