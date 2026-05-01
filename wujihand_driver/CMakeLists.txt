cmake_minimum_required(VERSION 3.8)
project(wujihand_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(wujihand_msgs REQUIRED)

# wujihandcpp is consumed via its installed Config package. Two ways to point
# CMake at it (local dev vs. system install):
#
#   * Local dev — build the SDK once, then point CMAKE_PREFIX_PATH at the
#     SDK build tree (no install step needed; the SDK exports a build-tree
#     Targets file):
#         cmake --build /path/to/wujihandpy/build
#         colcon build --cmake-args \
#             -DCMAKE_PREFIX_PATH=/path/to/wujihandpy/build/wujihandcpp
#
#   * System install — install the SDK into a standard prefix:
#         cmake --install /path/to/wujihandpy/build --prefix /usr/local
#         # then find_package picks it up automatically.
#
# wujihandcppConfig.cmake transitively pulls spdlog and Threads, so no
# manual link of those is needed below.
find_package(wujihandcpp CONFIG REQUIRED)

set(COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(COMMON_AMENT_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  wujihand_msgs
)

# --- wujihand_driver_node (joint control) ---
add_executable(wujihand_driver_node
  src/wujihand_driver_node.cpp
  src/wujihand_driver_main.cpp
)

target_include_directories(wujihand_driver_node PUBLIC ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(wujihand_driver_node ${COMMON_AMENT_DEPS})

# wujihandcpp::wujihandcpp brings include dirs, spdlog, Threads, and usb-1.0
# transitively (the latter as a bare `-lusb-1.0` flag).
target_link_libraries(wujihand_driver_node
  wujihandcpp::wujihandcpp
)

# Install
install(TARGETS wujihand_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
