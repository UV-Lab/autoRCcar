cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME ocam_publisher)
project(${PROJECT_NAME} LANGUAGES CXX)

# Find dependencies
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_subdirectory(api)

set(INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    ${OpenCV_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    )

set(LINK_LIBS
    LIB_OCAM
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    )

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Release by default" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_compile_options(
  "$<$<CONFIG:Debug>:-O0;-g>"
  "$<$<CONFIG:Release>:-O3>"
  "$<$<CONFIG:None>:-O3>"
  -std=c++17
  -Wfatal-errors
  )

add_executable(ocam_node ros2/publisher.cpp)
target_link_libraries (ocam_node PRIVATE ${LINK_LIBS})
target_include_directories(ocam_node PRIVATE ${INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(image_pipeline_test test/test_image_pipeline.cpp)
  target_link_libraries(image_pipeline_test ${OpenCV_LIBS})
  target_include_directories(image_pipeline_test PRIVATE ${INCLUDE_DIRS})

  ament_add_gtest(capture_mode_test test/test_capture_mode.cpp)
  target_include_directories(capture_mode_test PRIVATE ${INCLUDE_DIRS})

  ament_add_gtest(frame_wait_test test/test_frame_wait.cpp)
  target_include_directories(frame_wait_test PRIVATE ${INCLUDE_DIRS})

  find_package(ament_cmake_pytest REQUIRED)
  add_executable(ocam_node_fake_camera ros2/publisher.cpp test/fake_camera.cpp api/withrobot_utility.cpp)
  target_link_libraries(ocam_node_fake_camera PRIVATE
    ${OpenCV_LIBS} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES} ${sensor_msgs_LIBRARIES} udev)
  target_include_directories(ocam_node_fake_camera PRIVATE ${INCLUDE_DIRS})
  ament_add_pytest_test(publisher_process_test test/test_publisher_process.py
    TIMEOUT 60
    ENV OCAM_TEST_PUBLISHER=$<TARGET_FILE:ocam_node_fake_camera>)
  ament_add_pytest_test(publisher_launch_test test/test_publisher_launch.py
    TIMEOUT 60
    ENV OCAM_TEST_PUBLISHER=$<TARGET_FILE:ocam_node_fake_camera>)
endif()

install(TARGETS ocam_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
