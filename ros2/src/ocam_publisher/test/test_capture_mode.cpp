#include <gtest/gtest.h>

#include "capture_mode.hpp"

TEST(CaptureMode, NativeDownsampleRequestsNativeResolution) {
    const auto configuration = ocam::capture_config_for_mode("native_downsample");

    ASSERT_TRUE(configuration.has_value());
    EXPECT_EQ(configuration->width, 1280);
    EXPECT_EQ(configuration->height, 960);
    EXPECT_FALSE(configuration->hardware_binned);
}

TEST(CaptureMode, HardwareBinnedRequestsBinnedResolution) {
    const auto configuration = ocam::capture_config_for_mode("hardware_binned");

    ASSERT_TRUE(configuration.has_value());
    EXPECT_EQ(configuration->width, 640);
    EXPECT_EQ(configuration->height, 480);
    EXPECT_TRUE(configuration->hardware_binned);
}

TEST(CaptureMode, RejectsUnknownMode) {
    EXPECT_FALSE(ocam::capture_config_for_mode("native").has_value());
}
