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

# Find wujihandcpp (installed via deb package)
find_library(WUJIHANDCPP_LIBRARY NAMES wujihandcpp PATHS /usr/lib)
find_path(WUJIHANDCPP_INCLUDE_DIR NAMES wujihandcpp/device/hand.hpp PATHS /usr/include)

if(NOT WUJIHANDCPP_LIBRARY OR NOT WUJIHANDCPP_INCLUDE_DIR)
    message(FATAL_ERROR "wujihandcpp not found. Please install the wujihandcpp deb package.")
endif()

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
)

# Install
install(TARGETS wujihand_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
