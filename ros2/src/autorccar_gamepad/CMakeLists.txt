cmake_minimum_required(VERSION 3.8)
project(autorccar_gamepad)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autorccar_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ── 실행 파일 빌드 ──────────────────────────────────────────────────
add_executable(gamepad_control_node
  src/main.cpp
  src/gamepad_control_node.cpp
)
 
target_include_directories(gamepad_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
 
ament_target_dependencies(gamepad_control_node
  rclcpp
  sensor_msgs
  std_msgs
  autorccar_interfaces
)

# ── 설치 ────────────────────────────────────────────────────────────
install(TARGETS gamepad_control_node
  DESTINATION lib/${PROJECT_NAME}
)
 
install(DIRECTORY include/
  DESTINATION include
)
 
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
