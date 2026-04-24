cmake_minimum_required(VERSION 3.16)
project(ur_rtde_client)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/publisher.cpp
  src/rtde_manager.cpp
  src/converter.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${example_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${builtin_interfaces_TARGETS}
  ${ur_dashboard_msgs_TARGETS}
  sensor_msgs::sensor_msgs_library
  ur_client_library::urcl
  yaml-cpp
)

add_executable(rtde_client_node
  src/rtde_client_node.cpp
)
target_link_libraries(rtde_client_node PUBLIC
  ${PROJECT_NAME}_lib
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
)

install(TARGETS ${PROJECT_NAME}_lib rtde_client_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()