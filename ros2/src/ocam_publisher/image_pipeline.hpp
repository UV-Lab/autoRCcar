#pragma once

#include <opencv2/imgproc.hpp>

namespace ocam {

inline void demosaic_and_resize(const cv::Mat& bayer, cv::Mat& output, int conversion_code,
                                const cv::Size output_size) {
    if (bayer.size() == output_size) {
        cv::cvtColor(bayer, output, conversion_code);
        return;
    }

    cv::Mat native_color;
    cv::cvtColor(bayer, native_color, conversion_code);
    cv::resize(native_color, output, output_size, 0.0, 0.0, cv::INTER_AREA);
}

}  // namespace ocam
