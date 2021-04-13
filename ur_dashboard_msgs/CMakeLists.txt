cmake_minimum_required(VERSION 3.5)
project(ur_dashboard_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  msg/ProgramState.msg
  msg/RobotMode.msg
  msg/SafetyMode.msg
)

set(srv_files
  srv/AddToLog.srv
  srv/GetLoadedProgram.srv
  srv/GetProgramState.srv
  srv/GetRobotMode.srv
  srv/GetSafetyMode.srv
  srv/IsProgramRunning.srv
  srv/IsProgramSaved.srv
  srv/Load.srv
  srv/Popup.srv
  srv/RawRequest.srv
)

set(action_files
  action/SetMode.action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    action_msgs
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
