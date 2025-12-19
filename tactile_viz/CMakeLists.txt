cmake_minimum_required(VERSION 3.8)
project(tactile_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Find wujihandcpp (installed via deb package or local build)
find_library(WUJIHANDCPP_LIBRARY NAMES wujihandcpp
  PATHS
    /usr/lib
    /home/wuji/Public/wujihandcpp/build
)
find_path(WUJIHANDCPP_INCLUDE_DIR NAMES wujihandcpp/protocol/handler.hpp
  PATHS
    /usr/include
    /home/wuji/Public/wujihandcpp/include
)

if(NOT WUJIHANDCPP_LIBRARY OR NOT WUJIHANDCPP_INCLUDE_DIR)
    message(FATAL_ERROR "wujihandcpp not found. Please install the wujihandcpp deb package or build from source.")
endif()

message(STATUS "Found wujihandcpp: ${WUJIHANDCPP_LIBRARY}")
message(STATUS "Found wujihandcpp include: ${WUJIHANDCPP_INCLUDE_DIR}")

# Build tactile_viz_node executable
add_executable(tactile_viz_node
  src/tactile_viz_node.cpp
)

target_include_directories(tactile_viz_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${WUJIHANDCPP_INCLUDE_DIR}
)

ament_target_dependencies(tactile_viz_node
  rclcpp
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2_ros
)

target_link_libraries(tactile_viz_node
  ${WUJIHANDCPP_LIBRARY}
)

# Build tactile_stats_node executable
add_executable(tactile_stats_node
  src/tactile_stats_node.cpp
)

target_include_directories(tactile_stats_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${WUJIHANDCPP_INCLUDE_DIR}
)

# Set default log directory to workspace/tactile_viz/log
target_compile_definitions(tactile_stats_node PRIVATE
  TACTILE_VIZ_LOG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/log"
)

ament_target_dependencies(tactile_stats_node
  rclcpp
)

target_link_libraries(tactile_stats_node
  ${WUJIHANDCPP_LIBRARY}
)

# Install executables
install(TARGETS tactile_viz_node tactile_stats_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
