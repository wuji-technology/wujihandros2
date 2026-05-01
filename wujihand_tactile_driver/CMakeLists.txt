cmake_minimum_required(VERSION 3.8)
project(wujihand_tactile_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(wujihand_tactile_msgs REQUIRED)

# wujihandcpp is consumed via its installed Config package. See the SDK
# repo (wujihandpy) for build instructions:
#
#   cmake --install /path/to/wujihandpy/build --prefix /path/to/install
#   colcon build --cmake-args -DCMAKE_PREFIX_PATH=/path/to/install
#
# The wujihandcpp::wujihandcpp imported target carries include dirs,
# spdlog, Threads, and usb-1.0 transitively.
find_package(wujihandcpp CONFIG REQUIRED)

add_executable(tactile_driver_node
  src/tactile_driver_node.cpp
  src/main.cpp
)

target_include_directories(tactile_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tactile_driver_node
  rclcpp
  sensor_msgs
  wujihand_tactile_msgs
)

target_link_libraries(tactile_driver_node
  wujihandcpp::wujihandcpp
)

# Install
install(TARGETS tactile_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
