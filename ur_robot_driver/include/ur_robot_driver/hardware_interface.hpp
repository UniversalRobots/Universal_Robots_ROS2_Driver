// Copyright 2019 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \author  Andy Zelenak zelenak@picknik.ai
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2019-04-11
 *
 * \author  Mathias Fuhrer mathias.fuhrer@b-robotized.de
 * \date    2025-05-28 – Added support for usage with motion_primitives_controller
 *
 */
//----------------------------------------------------------------------
#ifndef UR_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_
#define UR_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_

// System
#include <cstdint>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <array>

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// UR stuff
#include "ur_client_library/ur/ur_driver.h"
#include "ur_client_library/ur/instruction_executor.h"
#include "ur_client_library/ur/robot_receive_timeout.h"
#include "ur_robot_driver/dashboard_client_ros.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"

// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "control_msgs/msg/motion_primitive.hpp"
#include <realtime_tools/lock_free_queue.hpp>

// Motion primitives controller
#include "motion_primitives_controllers/motion_primitives_forward_controller.hpp"

using MoprimMotionType = control_msgs::msg::MotionPrimitive;
using MoprimMotionHelperType = motion_primitives_controllers::MotionHelperType;

namespace ur_robot_driver
{
enum class PausingState
{
  PAUSED,
  RUNNING,
  RAMPUP
};

enum StoppingInterface
{
  NONE,
  STOP_POSITION,
  STOP_VELOCITY,
  STOP_PASSTHROUGH,
  STOP_FORCE_MODE,
  STOP_FREEDRIVE,
  STOP_TOOL_CONTACT,
  STOP_MOTION_PRIMITIVES,
  STOP_TORQUE,
  STOP_TWIST,
};

// We define our own quaternion to use it as a buffer, since we need to pass pointers to the state
// interfaces.
struct Quaternion
{
  Quaternion() : x(0), y(0), z(0), w(0)
  {
  }

  void set(const tf2::Quaternion& q)
  {
    x = q.x();
    y = q.y();
    z = q.z();
    w = q.w();
  }

  double x;
  double y;
  double z;
  double w;
};

struct RobotTypeWithSeries
{
  urcl::RobotType robot_type;
  urcl::RobotSeries robot_series;
};

RobotTypeWithSeries robotTypeFromString(const std::string& robot_type_str);

/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class URPositionHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(URPositionHardwareInterface)
  URPositionHardwareInterface();
  virtual ~URPositionHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) final;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) final;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) final;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() final;
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() final;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) final;

  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) final;

  /*!
   * \brief Callback to handle a change in the current state of the URCaps program running on the
   * robot. Executed only on the state change.
   *
   * \param program_running True when the URCap program is running on the robot.
   */
  void handleRobotProgramState(bool program_running);

  static constexpr double NO_NEW_CMD_ = std::numeric_limits<double>::quiet_NaN();

  void asyncThread();

protected:
  template <typename T>
  void readData(const std::unique_ptr<urcl::rtde_interface::DataPackage>& data_pkg, const std::string& var_name,
                T& data);
  template <typename T, size_t N>
  void readBitsetData(const std::unique_ptr<urcl::rtde_interface::DataPackage>& data_pkg, const std::string& var_name,
                      std::bitset<N>& data);

  // stop function used by on_shutdown and on_cleanup
  hardware_interface::CallbackReturn stop();

  // Interface handles, resolved once in on_export_*_interfaces()
  void resolve_state_interface_handles();
  void resolve_command_interface_handles();

  void initializeCommandInterfaces();
  void initAsyncIO();
  void checkAsyncIO();
  void updateNonDoubleValues();
  void extractToolPose();
  void transformForceTorque();
  void start_force_mode(const urcl::vector6d_t& task_frame, const urcl::vector6d_t& selection_vector,
                        const urcl::vector6d_t& wrench, double type, const urcl::vector6d_t& limits, double damping,
                        double gain_scaling);
  void stop_force_mode();
  // Reads the freedrive enable and abort commands and writes at most one freedrive control message.
  // Returns true when that write consumed this cycle. The caller sends a keepalive otherwise.
  bool commandFreedriveMode();
  void check_passthrough_trajectory_controller();
  void process_trajectory_result();
  void end_passthrough_cancel_wait();
  // Republish the joint and twist commands stored by perform_command_mode_switch().
  bool publish_seeded_motion_commands();
  void trajectory_done_callback(urcl::control::TrajectoryResult result);
  bool is_valid_joint_information(std::vector<std::array<double, 6>> data);
  void tool_contact_callback(urcl::control::ToolContactResult);
  void check_tool_contact_controller();

  urcl::vector6d_t urcl_position_commands_;
  urcl::vector6d_t urcl_velocity_commands_;
  urcl::vector6d_t urcl_torque_commands_;
  urcl::vector6d_t urcl_twist_commands_;
  urcl::vector6d_t urcl_joint_positions_;
  urcl::vector6d_t urcl_joint_velocities_;
  urcl::vector6d_t urcl_joint_efforts_;
  urcl::vector6d_t urcl_ft_sensor_measurements_;
  urcl::vector6d_t urcl_tcp_pose_;
  urcl::vector6d_t urcl_target_tcp_pose_;
  urcl::vector6d_t tcp_offset_;
  tf2::Quaternion tcp_rotation_quat_;
  Quaternion tcp_rotation_buffer;

  bool packet_read_;

  uint32_t runtime_state_;
  bool controllers_initialized_;

  std::bitset<18> actual_dig_out_bits_;
  std::bitset<18> actual_dig_in_bits_;
  std::array<double, 2> standard_analog_input_;
  std::array<double, 2> standard_analog_output_;
  std::bitset<4> analog_io_types_;
  uint32_t tool_mode_;
  std::bitset<2> tool_analog_input_types_;
  std::array<double, 2> tool_analog_input_;
  int32_t tool_output_voltage_;
  double tool_output_current_;
  double tool_temperature_;
  double speed_scaling_;
  double target_speed_fraction_;
  double speed_scaling_combined_;
  int32_t robot_mode_;
  int32_t safety_mode_;
  std::bitset<4> robot_status_bits_;
  std::bitset<11> safety_status_bits_;

  bool first_pass_;
  bool initialized_;
  bool command_snapshot_initialized_;
  double system_interface_initialized_;
  std::atomic_bool async_thread_shutdown_;
  urcl::VersionInformation version_info_;
  double get_robot_software_version_major_;
  double get_robot_software_version_minor_;
  double get_robot_software_version_bugfix_;
  double get_robot_software_version_build_;

  // Tool contact controller interface values
  double tool_contact_set_state_;
  double tool_contact_state_;
  std::atomic<double> tool_contact_result_;
  bool tool_contact_controller_running_;

  // Freedrive mode controller state. Enable and abort commands are read from their interfaces in write().
  // perform_command_mode_switch() sends FREEDRIVE_STOP. If that write fails, activated stays true and
  // write() retries the stop before any other control mode.
  std::atomic<bool> freedrive_activated_;
  std::atomic<bool> freedrive_mode_controller_running_;
  // Joint and twist commands stored by the mode switch have not reached their command interfaces.
  bool seeded_motion_commands_pending_ = false;
  // Motion-primitive command interfaces still hold pre-reset values.
  bool moprim_command_reset_pending_ = false;

  // Passthrough trajectory controller interface values
  uint8_t passthrough_trajectory_transfer_state_;
  uint8_t passthrough_trajectory_transfer_state_command_;
  uint32_t passthrough_trajectory_transfer_sequence_;
  uint32_t passthrough_trajectory_transfer_sequence_command_;
  bool passthrough_trajectory_abort_;
  bool passthrough_trajectory_abort_command_;
  bool passthrough_terminal_result_pending_;
  bool passthrough_trajectory_execution_active_;
  std::atomic_bool passthrough_cancel_result_pending_{ false };
  // The TRAJECTORY_CANCEL from stopping the passthrough controller still has to be sent.
  bool passthrough_cancel_unsent_ = false;
  uint32_t passthrough_trajectory_size_;
  bool passthrough_trajectory_controller_running_;
  urcl::vector6d_t passthrough_trajectory_positions_;
  urcl::vector6d_t passthrough_trajectory_velocities_;
  urcl::vector6d_t passthrough_trajectory_accelerations_;
  double passthrough_trajectory_time_from_start_;

  bool twist_controller_running_;

  // Payload reported by RTDE. Commanded payload is read from the command interfaces.
  double rtde_payload_mass_ = 0.0;
  urcl::vector3d_t rtde_payload_cog_{ 0.0, 0.0, 0.0 };
  urcl::vector6d_t rtde_payload_inertia_{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  //*************** Motion primitives stuff ***************
  std::shared_ptr<urcl::InstructionExecutor> instruction_executor_;

  // Async thread handling
  std::shared_ptr<std::thread> async_moprim_cmd_thread_;
  std::atomic_bool async_moprim_thread_shutdown_;
  realtime_tools::LockFreeSPSCQueue<std::array<double, 25>, 1024> moprim_cmd_queue_;
  std::array<double, 25> current_moprim_command_;

  // Status for communication with controller
  bool motion_primitives_forward_controller_running_;
  using MoprimExecutionState = motion_primitives_controllers::ExecutionState;
  std::atomic<MoprimExecutionState> current_moprim_execution_status_;
  std::atomic_bool ready_for_new_moprim_;

  // Command and state interfaces for the motion primitives
  // 25 Commands: motion_type + 6 joints + 2*7 positions (goal and via) + blend_radius + velocity + acceleration +
  // move_time
  std::array<double, 25> hw_moprim_commands_;
  std::array<double, 25> pending_moprim_command_;
  bool moprim_command_pending_ = false;
  // 2 States: execution_status, ready_for_new_primitive
  std::array<double, 2> hw_moprim_states_;

  // flag to put all following primitives into a motion sequence instead of sending single primitives
  std::atomic_bool build_moprim_sequence_{ false };
  std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> moprim_sequence_;

  void handleMoprimCommands(bool accept_new_command);
  bool resetMoprimCmdInterfaces(size_t start_index = 0);
  void asyncMoprimCmdThread();
  void processMoprimMotionCmd(const std::array<double, 25>& command);
  bool getMoprimTimeOrVelAndAcc(const std::array<double, 25>& command, double& velocity, double& acceleration,
                                double& move_time);
  bool getMoprimVelAndAcc(const std::array<double, 25>& command, double& velocity, double& acceleration,
                          double& move_time);
  void quaternionToRotVec(double qx, double qy, double qz, double qw, double& rx, double& ry, double& rz);

  const std::string HW_IF_MOTION_PRIMITIVES = "motion_primitive";
  //*************** End Motion primitives stuff ***************

  // copy of non double values
  std::array<double, 18> actual_dig_out_bits_copy_;
  std::array<double, 18> actual_dig_in_bits_copy_;
  std::array<double, 4> analog_io_types_copy_;
  double tool_mode_copy_;
  std::array<double, 2> tool_analog_input_types_copy_;
  double tool_output_voltage_copy_;
  double robot_mode_copy_;
  double safety_mode_copy_;
  std::array<double, 4> robot_status_bits_copy_;
  std::array<double, 11> safety_status_bits_copy_;

  std::atomic<bool> robot_program_running_;
  std::atomic<bool> stop_requested_;
  bool non_blocking_read_;
  double robot_program_running_copy_;

  /* Vectors used to store the trajectory received from the passthrough trajectory controller. The whole trajectory is
   * received before it is sent to the robot. */
  std::vector<std::array<double, 6>> trajectory_joint_positions_;
  std::vector<std::array<double, 6>> trajectory_joint_velocities_;
  std::vector<std::array<double, 6>> trajectory_joint_accelerations_;
  std::vector<double> trajectory_times_;
  // One slot per completion. The callback and write() exchange the result and its order together.
  realtime_tools::LockFreeSPSCQueue<int, 16> trajectory_result_queue_;

  PausingState pausing_state_;
  double pausing_ramp_up_increment_;

  // resources switching aux vars
  std::vector<std::vector<uint32_t>> stop_modes_;
  std::vector<std::set<std::string>> start_modes_;
  bool position_controller_running_;
  bool velocity_controller_running_;
  bool torque_controller_running_;
  bool force_mode_controller_running_ = false;

  std::shared_ptr<urcl::UrDriver> ur_driver_;  // changed to shared_ptr for instruction_executer
  std::shared_ptr<std::thread> async_thread_;

  std::atomic_bool rtde_comm_has_been_started_ = false;

  urcl::RobotReceiveTimeout receive_timeout_ = urcl::RobotReceiveTimeout::millisec(20);

  const std::string PASSTHROUGH_GPIO = "trajectory_passthrough";
  const std::string FORCE_MODE_GPIO = "force_mode";
  const std::string FREEDRIVE_MODE_GPIO = "freedrive_mode";
  const std::string TOOL_CONTACT_GPIO = "tool_contact";
  const std::string TWIST_GPIO = "twist";

  std::unordered_map<std::string, std::unordered_map<std::string, bool>> mode_compatibility_;

  std::unique_ptr<urcl::rtde_interface::DataPackage> data_package_buffer_;
  std::function<bool()> get_data_package;

  bool use_currents_as_efforts_ = false;

  // --- Interface handles, resolved once when the interfaces are exported ---
  // Resolving throws if the description lacks an interface, so every handle is valid afterwards. Only the joint effort
  // command handles are optional and nullptr when the description does not declare them.

  using StateHandle = hardware_interface::StateInterface::SharedPtr;
  using CommandHandle = hardware_interface::CommandInterface::SharedPtr;

  template <typename T>
  bool set_state_nonblocking(const StateHandle& handle, const T& value)
  {
    return set_state(handle, value, false);
  }
  template <typename T>
  bool get_command_nonblocking(const CommandHandle& handle, T& value)
  {
    return get_command(handle, value, false);
  }
  template <typename T>
  bool set_command_nonblocking(const CommandHandle& handle, const T& value)
  {
    return set_command(handle, value, false);
  }
  // Not real-time safe. Throws if the value cannot be read.
  double get_command_blocking(const CommandHandle& handle)
  {
    double value;
    get_command(handle, value, true);
    return value;
  }

  std::vector<StateHandle> joint_position_state_handles_;
  std::vector<StateHandle> joint_velocity_state_handles_;
  std::vector<StateHandle> joint_effort_state_handles_;
  std::vector<CommandHandle> joint_position_command_handles_;
  std::vector<CommandHandle> joint_velocity_command_handles_;
  std::vector<CommandHandle> joint_effort_command_handles_;

  // Plain RTDE-read state values (gpio digital/analog IO, tool/robot/safety status, tcp pose, ft
  // sensor, payload, get_robot_software_version, system_interface/gpio program_running): handle
  // plus a pointer to the source value, pushed via set_state() in one loop per read() cycle.
  std::vector<std::pair<StateHandle, double*>> state_push_list_;

  std::array<StateHandle, 2> moprim_state_handles_;

  CommandHandle io_async_success_handle_;
  std::array<CommandHandle, 18> standard_dig_out_bits_cmd_handles_;
  std::array<CommandHandle, 2> standard_analog_output_cmd_handles_;
  CommandHandle analog_output_domain_cmd_handle_;
  CommandHandle tool_voltage_cmd_handle_;
  CommandHandle target_speed_fraction_cmd_handle_;
  CommandHandle scaling_async_success_handle_;
  CommandHandle resend_robot_program_cmd_handle_;
  CommandHandle resend_robot_program_async_success_handle_;
  CommandHandle hand_back_control_cmd_handle_;
  CommandHandle hand_back_control_async_success_handle_;
  CommandHandle payload_mass_handle_;
  std::array<CommandHandle, 3> payload_cog_handles_;
  std::array<CommandHandle, 6> payload_inertia_handles_;
  CommandHandle payload_transition_time_handle_;
  CommandHandle payload_async_success_handle_;
  std::array<CommandHandle, 3> gravity_vector_handles_;
  CommandHandle gravity_async_success_handle_;
  std::array<CommandHandle, 6> friction_viscous_handles_;
  std::array<CommandHandle, 6> friction_coulomb_handles_;
  CommandHandle friction_async_success_handle_;
  CommandHandle zero_ftsensor_cmd_handle_;
  CommandHandle zero_ftsensor_async_success_handle_;
  CommandHandle freedrive_enable_handle_;
  CommandHandle freedrive_abort_handle_;
  CommandHandle freedrive_async_success_handle_;

  std::array<CommandHandle, 6> force_mode_task_frame_handles_;
  std::array<CommandHandle, 6> force_mode_selection_vector_handles_;
  std::array<CommandHandle, 6> force_mode_wrench_handles_;
  CommandHandle force_mode_type_handle_;
  std::array<CommandHandle, 6> force_mode_limits_handles_;
  CommandHandle force_mode_async_success_handle_;
  CommandHandle force_mode_disable_cmd_handle_;
  CommandHandle force_mode_damping_handle_;
  CommandHandle force_mode_gain_scaling_handle_;

  CommandHandle passthrough_transfer_state_cmd_handle_;
  CommandHandle passthrough_transfer_sequence_cmd_handle_;
  CommandHandle passthrough_time_from_start_handle_;
  CommandHandle passthrough_abort_cmd_handle_;
  CommandHandle passthrough_size_handle_;
  std::array<CommandHandle, 6> passthrough_positions_handles_;
  std::array<CommandHandle, 6> passthrough_velocities_handles_;
  std::array<CommandHandle, 6> passthrough_accelerations_handles_;
  StateHandle passthrough_transfer_state_state_handle_;
  StateHandle passthrough_transfer_sequence_state_handle_;
  StateHandle passthrough_abort_state_handle_;

  std::array<CommandHandle, 6> twist_command_handles_;

  CommandHandle tool_contact_set_state_handle_;
  StateHandle tool_contact_result_handle_;
  StateHandle tool_contact_state_handle_;

  std::array<CommandHandle, 25> moprim_command_handles_;
};
}  // namespace ur_robot_driver

#endif  // UR_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_
