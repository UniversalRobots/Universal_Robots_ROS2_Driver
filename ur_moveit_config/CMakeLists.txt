cmake_minimum_required(VERSION 3.28.3)
project(ur_moveit_config)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_srdf_tf_prefix test/test_srdf_tf_prefix.py)
endif()

ament_package()

install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
)
