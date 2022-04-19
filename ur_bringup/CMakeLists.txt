cmake_minimum_required(VERSION 3.5)
project(ur_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/start_ursim.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
