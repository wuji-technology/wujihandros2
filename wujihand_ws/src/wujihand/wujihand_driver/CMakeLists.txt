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

# Use local wujihandcpp from third_party
set(WUJIHANDCPP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../third_party/wujihandpy/wujihandcpp")
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
# Provide LICENSE file to prevent CPack error (wujihandcpp uses CMAKE_SOURCE_DIR)
file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE" "MIT License - See third_party/wujihandpy/wujihandcpp/LICENSE")
add_subdirectory(${WUJIHANDCPP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/wujihandcpp EXCLUDE_FROM_ALL)

# Build executable
add_executable(wujihand_driver_node
  src/wujihand_driver_node.cpp
  src/main.cpp
)

target_include_directories(wujihand_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(wujihand_driver_node
  rclcpp
  std_msgs
  sensor_msgs
  wujihand_msgs
)

target_link_libraries(wujihand_driver_node
  wujihandcpp
)

# Install
install(TARGETS wujihand_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS wujihandcpp
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

# Add library path to LD_LIBRARY_PATH via environment hook
ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

ament_package()
