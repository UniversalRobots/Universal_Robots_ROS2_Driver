#pragma once

// TODO Redefinition of industrial_robot_status_interface
// Exchange as soon as it is released for Foxy

enum class TriState : int8_t
{
  UNKNOWN = -1,
  FALSE = 0,
  TRUE = 1,
};
enum class RobotMode : int8_t
{
  UNKNOWN = -1,
  MANUAL = 1,
  AUTO = 2,
};
typedef std::int32_t ErrorCode;
struct RobotStatus
{
  RobotMode mode;
  TriState e_stopped;
  TriState drives_powered;
  TriState motion_possible;
  TriState in_motion;
  TriState in_error;
  ErrorCode error_code;
};
