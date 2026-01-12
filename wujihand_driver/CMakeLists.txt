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

# Find wujihandcpp
# Set WUJIHANDPY_SOURCE_DIR env var to use source build, otherwise uses system install
if(DEFINED ENV{WUJIHANDPY_SOURCE_DIR})
    set(WUJIHANDPY_DIR "$ENV{WUJIHANDPY_SOURCE_DIR}")
    set(WUJIHANDCPP_INCLUDE_DIR "${WUJIHANDPY_DIR}/wujihandcpp/include")
    set(WUJIHANDCPP_LIBRARY "${WUJIHANDPY_DIR}/build/wujihandcpp/libwujihandcpp.a")
    set(WUJIHANDCPP_SPDLOG_LIBRARY "${WUJIHANDPY_DIR}/build/_deps/spdlog-build/libspdlog.a")
    message(STATUS "Using wujihandcpp from source: ${WUJIHANDPY_DIR}")
else()
    find_library(WUJIHANDCPP_LIBRARY NAMES wujihandcpp PATHS /usr/lib)
    find_path(WUJIHANDCPP_INCLUDE_DIR NAMES wujihandcpp/device/hand.hpp PATHS /usr/include)
    find_library(WUJIHANDCPP_SPDLOG_LIBRARY NAMES spdlog PATHS /usr/lib)
    message(STATUS "Using system-installed wujihandcpp")
endif()

if(NOT WUJIHANDCPP_LIBRARY OR NOT WUJIHANDCPP_INCLUDE_DIR)
    message(FATAL_ERROR "wujihandcpp not found. Please install the wujihandcpp deb package or build wujihandpy first.")
endif()

if(NOT WUJIHANDCPP_SPDLOG_LIBRARY)
    message(FATAL_ERROR "spdlog not found. Please install libspdlog-dev or build wujihandpy first.")
endif()

find_package(Threads REQUIRED)

message(STATUS "Found wujihandcpp: ${WUJIHANDCPP_LIBRARY}")
message(STATUS "Found wujihandcpp include: ${WUJIHANDCPP_INCLUDE_DIR}")

# Build executable
add_executable(wujihand_driver_node
  src/wujihand_driver_node.cpp
  src/main.cpp
)

target_include_directories(wujihand_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${WUJIHANDCPP_INCLUDE_DIR}
)

ament_target_dependencies(wujihand_driver_node
  rclcpp
  std_msgs
  sensor_msgs
  wujihand_msgs
)

target_link_libraries(wujihand_driver_node
  ${WUJIHANDCPP_LIBRARY}
  ${WUJIHANDCPP_SPDLOG_LIBRARY}
  usb-1.0
  Threads::Threads
)

# Install
install(TARGETS wujihand_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
