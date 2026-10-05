#include "publisher.hpp"

#include "capture_mode.hpp"
#include "image_pipeline.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    node->initialize_config();

    const std::string capture_mode = node->get_parameter("capture_mode").as_string();
    const auto capture_configuration = ocam::capture_config_for_mode(capture_mode);
    if (!capture_configuration.has_value()) {
        RCLCPP_ERROR(node->get_logger(),
                     "Unsupported capture_mode '%s'; expected 'native_downsample' or 'hardware_binned'",
                     capture_mode.c_str());
        rclcpp::shutdown();
        return -1;
    }

    const int capture_width = capture_configuration->width;
    const int capture_height = capture_configuration->height;
    const int output_width = node->get_parameter("OutputWidth").as_int();
    const int output_height = node->get_parameter("OutputHeight").as_int();
    const int fps = node->get_parameter("FPS").as_int();
    const double frame_timeout_sec = node->get_parameter("frame_timeout_sec").as_double();
    if (!std::isfinite(frame_timeout_sec) || frame_timeout_sec <= 0) {
        RCLCPP_ERROR(node->get_logger(), "frame_timeout_sec must be finite and positive");
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
    if (output_width != 640 || output_height != 480 || fps <= 0) {
        RCLCPP_ERROR(node->get_logger(),
                     "The publisher requires OutputWidth=640, OutputHeight=480, and a positive FPS; got %dx%d at %d fps",
                     output_width, output_height, fps);
        rclcpp::shutdown();
        return -1;
    }

    std::string devPath = node->get_parameter("device_path").as_string();
    // Use designated port when given
    if (!devPath.empty()) {
        if (!std::filesystem::exists(devPath)) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not found camera device: %s", devPath.c_str());
            return -1;
        }
    } else {
        if (!find_v4l_device_path(devPath)) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "oCam-1MGN-U-T is not found in default path: /dev/v4l/by-id");
            return -1;
        }
    }

    const unsigned int grbg_format = Withrobot::fourcc_to_pixformat('G', 'R', 'B', 'G');

    Withrobot::Camera camera(devPath);
    if (!camera.set_format(capture_width, capture_height, grbg_format, 1, fps)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to request %dx%d GRBG at %d fps", capture_width,
                     capture_height, fps);
        rclcpp::shutdown();
        return -1;
    }

    const int requested_auto_exposure = node->get_parameter("Auto exposure mode").as_int();
    const int requested_exposure = node->get_parameter("Exposure").as_int();
    const int requested_gain = node->get_parameter("Brightness").as_int();
    if (!camera.set_control("Auto Exposure", requested_auto_exposure) ||
        !camera.set_control("Exposure Time, Absolute", requested_exposure) ||
        !camera.set_control("Gain", requested_gain)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to configure one or more camera controls");
        rclcpp::shutdown();
        return -1;
    }

    // Print infomations
    Withrobot::camera_format camFormat;
    if (!camera.get_current_format(camFormat)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to read the negotiated camera format");
        rclcpp::shutdown();
        return -1;
    }
    if (camFormat.width != static_cast<unsigned int>(capture_width) ||
        camFormat.height != static_cast<unsigned int>(capture_height) || camFormat.pixformat != grbg_format) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Camera negotiated %dx%d format 0x%08x; expected %dx%d GRBG (0x%08x)", camFormat.width,
                     camFormat.height, camFormat.pixformat, capture_width, capture_height, grbg_format);
        rclcpp::shutdown();
        return -1;
    }
    if (camFormat.rate_numerator != 1 || camFormat.rate_denominator != static_cast<unsigned int>(fps)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Camera negotiated %u/%u fps; expected 1/%d fps",
                     camFormat.rate_numerator, camFormat.rate_denominator, fps);
        rclcpp::shutdown();
        return -1;
    }

    const int effective_auto_exposure = camera.get_control("Auto Exposure");
    const int effective_exposure = camera.get_control("Exposure Time, Absolute");
    const int effective_gain = camera.get_control("Gain");
    if (effective_auto_exposure != requested_auto_exposure || effective_exposure != requested_exposure ||
        effective_gain != requested_gain) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Camera controls differ from request: auto exposure %d/%d, exposure %d/%d, gain %d/%d",
                     effective_auto_exposure, requested_auto_exposure, effective_exposure, requested_exposure,
                     effective_gain, requested_gain);
        rclcpp::shutdown();
        return -1;
    }

    const std::string camName = camera.get_dev_name();
    const std::string camSerialNumber = camera.get_serial_number();

    printf("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
    printf(
        "----------------- Current format informations "
        "-----------------\n");
    camFormat.print();
    printf(
        "------------------------------------------------------------"
        "---\n");

    std::cout << "Current Gain: " << effective_gain << std::endl;
    std::cout << "Current Exposure Time: " << effective_exposure << std::endl;
    std::cout << "Current Auto Exposure Mode: " << effective_auto_exposure << std::endl;
    RCLCPP_INFO(node->get_logger(), "capture_mode=%s: %dx%d GRBG -> %dx%d rgb8 at %d fps", capture_mode.c_str(),
                capture_width, capture_height, output_width, output_height, fps);

    if (!camera.start()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to start camera streaming");
        camera.stop();
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    cv::Mat colorImg;
    const cv::Size output_size(output_width, output_height);

    int exit_status = EXIT_SUCCESS;
    auto last_frame = std::chrono::steady_clock::now();
    try {
        while (rclcpp::ok()) {
            errno = 0;
            const int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);
            const int frame_error = errno;
            if (!rclcpp::ok()) break;

            if (size != static_cast<int>(camFormat.image_size)) {
                const double missing_sec = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - last_frame).count();
                if (missing_sec >= frame_timeout_sec) {
                    RCLCPP_ERROR(node->get_logger(), "No complete frame for %.2f seconds (limit %.2f)",
                                 missing_sec, frame_timeout_sec);
                    exit_status = EXIT_FAILURE;
                    break;
                }
                if (frame_error == EINTR) continue;

                RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                                     "Incomplete camera frame (%d bytes, errno %d); attempting restart",
                                     size, frame_error);
                if (!camera.stop()) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to stop camera for recovery");
                    exit_status = EXIT_FAILURE;
                    break;
                }
                if (!camera.start()) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to restart camera streaming");
                    exit_status = EXIT_FAILURE;
                    break;
                }
                continue;
            }

            last_frame = std::chrono::steady_clock::now();
            ocam::demosaic_and_resize(srcImg, colorImg, cv::COLOR_BayerGB2RGB, output_size);
            node->publish_image(colorImg);
            rclcpp::spin_some(node);
        }
    } catch (const rclcpp::exceptions::RCLError& error) {
        // SIGINT can invalidate the ROS context between the loop guard and publish.
        if (rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "ROS capture error: %s", error.what());
            exit_status = EXIT_FAILURE;
        }
    } catch (const std::exception& error) {
        RCLCPP_ERROR(node->get_logger(), "Image processing failed: %s", error.what());
        exit_status = EXIT_FAILURE;
    }
    if (!camera.stop()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to stop camera streaming");
        exit_status = EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return exit_status;
}
