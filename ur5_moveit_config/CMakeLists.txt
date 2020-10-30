cmake_minimum_required(VERSION 3.5)
project(ur5_moveit_config)

find_package(ament_cmake REQUIRED)

# if (BUILD_TESTING)
#   find_package(roslaunch REQUIRED)
#   roslaunch_add_file_check(tests/moveit_planning_execution.xml)
# endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
