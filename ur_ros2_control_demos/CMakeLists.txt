cmake_minimum_required(VERSION 3.5)
project(ur_ros2_control_demos)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(test_driver
  src/test_driver.cpp
)
ament_target_dependencies(test_driver rclcpp std_msgs sensor_msgs)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

install(TARGETS
        test_driver
        DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
