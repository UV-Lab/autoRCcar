#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "image_pipeline.hpp"

namespace {

cv::Mat make_solid_grbg(const cv::Size size, const cv::Vec3b& rgb) {
    cv::Mat bayer(size, CV_8UC1);
    for (int y = 0; y < size.height; ++y) {
        for (int x = 0; x < size.width; ++x) {
            const bool even_row = (y % 2) == 0;
            const bool even_column = (x % 2) == 0;
            const int channel = even_row ? (even_column ? 1 : 0) : (even_column ? 2 : 1);
            bayer.at<uint8_t>(y, x) = rgb[channel];
        }
    }
    return bayer;
}

TEST(ImagePipeline, PreservesRedInRgbOutputForGrbgInput) {
    const cv::Mat bayer = make_solid_grbg({16, 16}, {255, 0, 0});
    cv::Mat rgb;

    ocam::demosaic_and_resize(bayer, rgb, cv::COLOR_BayerGB2RGB, bayer.size());

    const cv::Vec3b pixel = rgb.at<cv::Vec3b>(8, 8);
    EXPECT_EQ(pixel[0], 255);
    EXPECT_EQ(pixel[1], 0);
    EXPECT_EQ(pixel[2], 0);
}

TEST(ImagePipeline, PreservesGreenInRgbOutputForGrbgInput) {
    const cv::Mat bayer = make_solid_grbg({16, 16}, {0, 255, 0});
    cv::Mat rgb;

    ocam::demosaic_and_resize(bayer, rgb, cv::COLOR_BayerGB2RGB, bayer.size());

    const cv::Vec3b pixel = rgb.at<cv::Vec3b>(8, 8);
    EXPECT_EQ(pixel[0], 0);
    EXPECT_EQ(pixel[1], 255);
    EXPECT_EQ(pixel[2], 0);
}

TEST(ImagePipeline, PreservesBlueInRgbOutputForGrbgInput) {
    const cv::Mat bayer = make_solid_grbg({16, 16}, {0, 0, 255});
    cv::Mat rgb;

    ocam::demosaic_and_resize(bayer, rgb, cv::COLOR_BayerGB2RGB, bayer.size());

    const cv::Vec3b pixel = rgb.at<cv::Vec3b>(8, 8);
    EXPECT_EQ(pixel[0], 0);
    EXPECT_EQ(pixel[1], 0);
    EXPECT_EQ(pixel[2], 255);
}

TEST(ImagePipeline, PreservesRedInBgrOutputForGrbgInput) {
    const cv::Mat bayer = make_solid_grbg({16, 16}, {255, 0, 0});
    cv::Mat bgr;

    ocam::demosaic_and_resize(bayer, bgr, cv::COLOR_BayerGB2BGR, bayer.size());

    const cv::Vec3b pixel = bgr.at<cv::Vec3b>(8, 8);
    EXPECT_EQ(pixel[0], 0);
    EXPECT_EQ(pixel[1], 0);
    EXPECT_EQ(pixel[2], 255);
}

TEST(ImagePipeline, PreservesColorAtRequestedDownsampledSize) {
    const cv::Mat bayer = make_solid_grbg({1280, 960}, {100, 150, 200});
    cv::Mat rgb;

    ocam::demosaic_and_resize(bayer, rgb, cv::COLOR_BayerGB2RGB, {640, 480});

    EXPECT_EQ(rgb.type(), CV_8UC3);
    EXPECT_EQ(rgb.cols, 640);
    EXPECT_EQ(rgb.rows, 480);
    EXPECT_EQ(rgb.at<cv::Vec3b>(240, 320), cv::Vec3b(100, 150, 200));
}

}  // namespace
