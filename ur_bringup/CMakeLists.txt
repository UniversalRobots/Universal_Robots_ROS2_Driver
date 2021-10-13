cmake_minimum_required(VERSION 3.5)
project(ur_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
ament_python_install_module(ur_bringup/launch_common.py)

ament_package()
