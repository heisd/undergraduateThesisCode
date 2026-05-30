cmake_minimum_required(VERSION 3.8)
project(trajectory_bridge)

# ── 编译选项 ──────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── 依赖 ──────────────────────────────────────
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(control_msgs  REQUIRED)

# ── 可执行目标 ────────────────────────────────
add_executable(trajectory_bridge_node
  src/trajectory_bridge_node.cpp
)

target_include_directories(trajectory_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(trajectory_bridge_node
  rclcpp
  rclcpp_action
  std_msgs
  control_msgs
)

# ── 安装 ─────────────────────────────────────
install(TARGETS trajectory_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── 测试（可选） ──────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()