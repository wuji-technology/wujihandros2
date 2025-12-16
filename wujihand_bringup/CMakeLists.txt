cmake_minimum_required(VERSION 3.8)
project(wujihand_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/wave_demo.py
  scripts/mock_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
