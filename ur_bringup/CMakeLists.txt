cmake_minimum_required(VERSION 3.5)
project(ur_bringup)

find_package(ament_cmake REQUIRED)

# if (BUILD_TESTING)
#   find_package(roslaunch REQUIRED)
#   roslaunch_add_file_check(tests/roslaunch_test.xml)
# endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
