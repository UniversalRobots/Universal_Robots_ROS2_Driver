cmake_minimum_required(VERSION 3.28.3)
project(ur_moveit_config)

find_package(ament_cmake REQUIRED)

find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(ur_moveit_preserve_continuous_joint_turns SHARED
  src/preserve_continuous_joint_turns.cpp
  src/preserve_continuous_joint_turns_adapter.cpp
)
target_compile_features(ur_moveit_preserve_continuous_joint_turns PUBLIC cxx_std_20)
ament_target_dependencies(ur_moveit_preserve_continuous_joint_turns moveit_core pluginlib rclcpp)
pluginlib_export_plugin_description_file(moveit_core preserve_continuous_joint_turns_plugin.xml)
install(TARGETS ur_moveit_preserve_continuous_joint_turns LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)
  ament_add_gtest(test_preserve_continuous_joint_turns
    test/test_preserve_continuous_joint_turns.cpp
  )
  target_link_libraries(test_preserve_continuous_joint_turns ur_moveit_preserve_continuous_joint_turns)
  ament_target_dependencies(test_preserve_continuous_joint_turns moveit_core srdfdom urdf)
endif()

ament_package()

install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
)
