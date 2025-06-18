// Copyright 2019, FZI Forschungszentrum Informatik
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
 * \date    2020-11-9
 *
 * \author  Mathias Fuhrer mathias.fuhrer@b-robotized.de
 * \date    2025-05-28 – Added support for usage with motion_primitives_forward_controller
 *
 */
//----------------------------------------------------------------------
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ur_client_library/exceptions.h"
#include "ur_client_library/ur/tool_communication.h"
#include "ur_client_library/ur/version_information.h"

#include <rclcpp/logging.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ur_robot_driver/hardware_interface.hpp"
#include "ur_robot_driver/urcl_log_handler.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace rtde = urcl::rtde_interface;

namespace ur_robot_driver
{

URPositionHardwareInterface::~URPositionHardwareInterface()
{
}

hardware_interface::CallbackReturn
URPositionHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = system_info;

  // initialize
  urcl_joint_positions_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_joint_velocities_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_joint_efforts_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_ft_sensor_measurements_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_tcp_pose_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_position_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_position_commands_old_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  position_controller_running_ = false;
  velocity_controller_running_ = false;
  freedrive_mode_controller_running_ = false;
  passthrough_trajectory_controller_running_ = false;
  runtime_state_ = static_cast<uint32_t>(rtde::RUNTIME_STATE::STOPPED);
  pausing_state_ = PausingState::RUNNING;
  pausing_ramp_up_increment_ = 0.01;
  controllers_initialized_ = false;
  first_pass_ = true;
  initialized_ = false;
  async_thread_shutdown_ = false;
  system_interface_initialized_ = 0.0;
  freedrive_mode_abort_ = 0.0;
  passthrough_trajectory_transfer_state_ = 0.0;
  passthrough_trajectory_abort_ = 0.0;
  trajectory_joint_positions_.clear();
  trajectory_joint_velocities_.clear();
  trajectory_joint_accelerations_.clear();

  // Motion primitives stuff
  async_moprim_thread_shutdown_ = false;
  new_moprim_cmd_available_ = false;
  new_moprim_stop_available_ = false;
  new_moprim_reset_available_ = false;
  current_moprim_execution_status_ = ExecutionState::IDLE;
  ready_for_new_moprim_ = false;
  motion_primitives_forward_controller_running_ = false;
  // 2 States: execution_status, ready_for_new_primitive
  hw_moprim_states_.resize(2, std::numeric_limits<double>::quiet_NaN());
  // 25 Commands: // motion_type + 6 joints + 2*7 positions (goal and via) + blend_radius + velocity + acceleration +
  // move_time
  hw_moprim_commands_.resize(25, std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> URPositionHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &urcl_joint_positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &urcl_joint_velocities_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &urcl_joint_efforts_[i]));
  }

  // Obtain the tf_prefix from the urdf so that we can have the general interface multiple times
  // NOTE using the tf_prefix at this point is some kind of workaround. One should actually go through the list of gpio
  // state interface in info_ and match them accordingly
  const std::string tf_prefix = info_.hardware_parameters.at("tf_prefix");
  state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "speed_scaling", "speed_scaling_factor",
                                                                   &speed_scaling_combined_));

  for (auto& sensor : info_.sensors) {
    if (sensor.name == tf_prefix + "tcp_fts_sensor") {
      const std::vector<std::string> fts_names = {
        "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"
      };
      for (uint j = 0; j < 6; ++j) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(sensor.name, fts_names[j], &urcl_ft_sensor_measurements_[j]));
      }
    }
  }

  for (size_t i = 0; i < 18; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "digital_output_" + std::to_string(i), &actual_dig_out_bits_copy_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "digital_input_" + std::to_string(i), &actual_dig_in_bits_copy_[i]));
  }

  for (size_t i = 0; i < 11; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "safety_status_bit_" + std::to_string(i), &safety_status_bits_copy_[i]));
  }

  for (size_t i = 0; i < 4; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "analog_io_type_" + std::to_string(i), &analog_io_types_copy_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "robot_status_bit_" + std::to_string(i), &robot_status_bits_copy_[i]));
  }

  for (size_t i = 0; i < 2; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "tool_analog_input_type_" + std::to_string(i), &tool_analog_input_types_copy_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "tool_analog_input_" + std::to_string(i), &tool_analog_input_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "standard_analog_input_" + std::to_string(i), &standard_analog_input_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "gpio", "standard_analog_output_" + std::to_string(i), &standard_analog_output_[i]));
  }

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_voltage", &tool_output_voltage_copy_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "gpio", "robot_mode", &robot_mode_copy_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "gpio", "safety_mode", &safety_mode_copy_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "gpio", "tool_mode", &tool_mode_copy_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_current", &tool_output_current_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "gpio", "tool_temperature", &tool_temperature_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "system_interface", "initialized",
                                                                   &system_interface_initialized_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "gpio", "program_running", &robot_program_running_copy_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "tcp_pose", "position.x", &urcl_tcp_pose_[0]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "tcp_pose", "position.y", &urcl_tcp_pose_[1]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "tcp_pose", "position.z", &urcl_tcp_pose_[2]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.x", &tcp_rotation_buffer.x));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.y", &tcp_rotation_buffer.y));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.z", &tcp_rotation_buffer.z));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.w", &tcp_rotation_buffer.w));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      tf_prefix + "get_robot_software_version", "get_version_major", &get_robot_software_version_major_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      tf_prefix + "get_robot_software_version", "get_version_minor", &get_robot_software_version_minor_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      tf_prefix + "get_robot_software_version", "get_version_bugfix", &get_robot_software_version_bugfix_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      tf_prefix + "get_robot_software_version", "get_version_build", &get_robot_software_version_build_));

  // Motion primitives stuff
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(HW_IF_MOTION_PRIMITIVES, "execution_status", &hw_moprim_states_[0]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(HW_IF_MOTION_PRIMITIVES, "ready_for_new_primitive", &hw_moprim_states_[1]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> URPositionHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &urcl_position_commands_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &urcl_velocity_commands_[i]));
  }
  // Obtain the tf_prefix from the urdf so that we can have the general interface multiple times
  // NOTE using the tf_prefix at this point is some kind of workaround. One should actually go through the list of gpio
  // command interface in info_ and match them accordingly
  const std::string tf_prefix = info_.hardware_parameters.at("tf_prefix");

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "gpio", "io_async_success", &io_async_success_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "speed_scaling", "target_speed_fraction_cmd", &target_speed_fraction_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "speed_scaling", "target_speed_fraction_async_success", &scaling_async_success_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "resend_robot_program", "resend_robot_program_cmd", &resend_robot_program_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "resend_robot_program", "resend_robot_program_async_success", &resend_robot_program_async_success_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "hand_back_control", "hand_back_control_cmd", &hand_back_control_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "hand_back_control", "hand_back_control_async_success", &hand_back_control_async_success_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + "payload", "mass", &payload_mass_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "payload", "cog.x", &payload_center_of_gravity_[0]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "payload", "cog.y", &payload_center_of_gravity_[1]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "payload", "cog.z", &payload_center_of_gravity_[2]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "payload", "payload_async_success", &payload_async_success_));

  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "task_frame_x", &force_mode_task_frame_[0]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "task_frame_y", &force_mode_task_frame_[1]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "task_frame_z", &force_mode_task_frame_[2]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "task_frame_rx", &force_mode_task_frame_[3]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "task_frame_ry", &force_mode_task_frame_[4]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "task_frame_rz", &force_mode_task_frame_[5]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "selection_vector_x", &force_mode_selection_vector_[0]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "selection_vector_y", &force_mode_selection_vector_[1]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "selection_vector_z", &force_mode_selection_vector_[2]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "selection_vector_rx", &force_mode_selection_vector_[3]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "selection_vector_ry", &force_mode_selection_vector_[4]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "selection_vector_rz", &force_mode_selection_vector_[5]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "wrench_x", &force_mode_wrench_[0]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "wrench_y", &force_mode_wrench_[1]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "wrench_z", &force_mode_wrench_[2]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "wrench_rx", &force_mode_wrench_[3]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "wrench_ry", &force_mode_wrench_[4]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "wrench_rz", &force_mode_wrench_[5]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "type", &force_mode_type_);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "limits_x", &force_mode_limits_[0]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "limits_y", &force_mode_limits_[1]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "limits_z", &force_mode_limits_[2]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "limits_rx", &force_mode_limits_[3]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "limits_ry", &force_mode_limits_[4]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "limits_rz", &force_mode_limits_[5]);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "force_mode_async_success", &force_mode_async_success_);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "disable_cmd", &force_mode_disable_cmd_);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "damping", &force_mode_damping_);
  command_interfaces.emplace_back(tf_prefix + FORCE_MODE_GPIO, "gain_scaling", &force_mode_gain_scaling_);

  for (size_t i = 0; i < 18; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        tf_prefix + "gpio", "standard_digital_output_cmd_" + std::to_string(i), &standard_dig_out_bits_cmd_[i]));
  }

  for (size_t i = 0; i < 2; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        tf_prefix + "gpio", "standard_analog_output_cmd_" + std::to_string(i), &standard_analog_output_cmd_[i]));
  }
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "gpio", "analog_output_domain_cmd", &analog_output_domain_cmd_));

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "gpio", "tool_voltage_cmd", &tool_voltage_cmd_));

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + "zero_ftsensor", "zero_ftsensor_cmd", &zero_ftsensor_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      tf_prefix + "zero_ftsensor", "zero_ftsensor_async_success", &zero_ftsensor_async_success_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + FREEDRIVE_MODE_GPIO, "async_success",
                                                                       &freedrive_mode_async_success_));

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + FREEDRIVE_MODE_GPIO, "enable", &freedrive_mode_enable_));

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + FREEDRIVE_MODE_GPIO, "abort", &freedrive_mode_abort_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + PASSTHROUGH_GPIO, "transfer_state",
                                                                       &passthrough_trajectory_transfer_state_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + PASSTHROUGH_GPIO, "time_from_start",
                                                                       &passthrough_trajectory_time_from_start_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(tf_prefix + PASSTHROUGH_GPIO, "abort", &passthrough_trajectory_abort_));

  for (size_t i = 0; i < 6; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + PASSTHROUGH_GPIO,
                                                                         "setpoint_positions_" + std::to_string(i),
                                                                         &passthrough_trajectory_positions_[i]));
  }

  for (size_t i = 0; i < 6; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + PASSTHROUGH_GPIO,
                                                                         "setpoint_velocities_" + std::to_string(i),
                                                                         &passthrough_trajectory_velocities_[i]));
  }

  for (size_t i = 0; i < 6; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(tf_prefix + PASSTHROUGH_GPIO,
                                                                         "setpoint_accelerations_" + std::to_string(i),
                                                                         &passthrough_trajectory_accelerations_[i]));
  }

  // Motion primitives stuff
  // Command for motion type (motion_type)
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "motion_type", &hw_moprim_commands_[0]));
  // Joint position commands (q1, q2, ..., q6)
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "q1", &hw_moprim_commands_[1]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "q2", &hw_moprim_commands_[2]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "q3", &hw_moprim_commands_[3]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "q4", &hw_moprim_commands_[4]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "q5", &hw_moprim_commands_[5]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "q6", &hw_moprim_commands_[6]));
  // Position commands (pos_x, pos_y, pos_z, pos_qx, pos_qy, pos_qz, pos_qz)
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_x", &hw_moprim_commands_[7]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_y", &hw_moprim_commands_[8]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_z", &hw_moprim_commands_[9]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_qx", &hw_moprim_commands_[10]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_qy", &hw_moprim_commands_[11]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_qz", &hw_moprim_commands_[12]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_qw", &hw_moprim_commands_[13]));
  // Via Position commands for circula motion
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_via_x", &hw_moprim_commands_[14]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_via_y", &hw_moprim_commands_[15]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_via_z", &hw_moprim_commands_[16]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_via_qx", &hw_moprim_commands_[17]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_via_qy", &hw_moprim_commands_[18]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_via_qz", &hw_moprim_commands_[19]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "pos_via_qw", &hw_moprim_commands_[20]));
  // Other command parameters (blend_radius, velocity, acceleration, move_time)
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "blend_radius", &hw_moprim_commands_[21]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "velocity", &hw_moprim_commands_[22]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "acceleration", &hw_moprim_commands_[23]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface(HW_IF_MOTION_PRIMITIVES, "move_time", &hw_moprim_commands_[24]));

  return command_interfaces;
}

hardware_interface::CallbackReturn
URPositionHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Starting ...please wait...");

  // The robot's IP address.
  const std::string robot_ip = info_.hardware_parameters["robot_ip"];
  // Path to the urscript code that will be sent to the robot
  const std::string script_filename = info_.hardware_parameters["script_filename"];
  // Path to the file containing the recipe used for requesting RTDE outputs.
  const std::string output_recipe_filename = info_.hardware_parameters["output_recipe_filename"];
  // Path to the file containing the recipe used for requesting RTDE inputs.
  const std::string input_recipe_filename = info_.hardware_parameters["input_recipe_filename"];
  // Start robot in headless mode. This does not require the 'External Control' URCap to be running
  // on the robot, but this will send the URScript to the robot directly. On e-Series robots this
  // requires the robot to run in 'remote-control' mode.
  const bool headless_mode =
      (info_.hardware_parameters["headless_mode"] == "true") || (info_.hardware_parameters["headless_mode"] == "True");
  // Port that will be opened to communicate between the driver and the robot controller.
  const int reverse_port = stoi(info_.hardware_parameters["reverse_port"]);
  // The driver will offer an interface to receive the program's URScript on this port.
  const int script_sender_port = stoi(info_.hardware_parameters["script_sender_port"]);

  // The ip address of the host the driver runs on
  std::string reverse_ip = info_.hardware_parameters["reverse_ip"];
  if (reverse_ip == "0.0.0.0") {
    reverse_ip = "";
  }

  // Port (on the host pc) of the trajectory interface
  const int trajectory_port = stoi(info_.hardware_parameters["trajectory_port"]);

  // Port (on the host PC) that will be used to forward script commands from the driver to the robot
  const int script_command_port = stoi(info_.hardware_parameters["script_command_port"]);

  // Enables non_blocking_read mode. Should only be used with combined_robot_hw. Disables error generated when read
  // returns without any data, sets the read timeout to zero, and synchronises read/write operations. Enabling this when
  // not used with combined_robot_hw can suppress important errors and affect real-time performance.
  non_blocking_read_ = (info_.hardware_parameters["non_blocking_read"] == "true") ||
                       (info_.hardware_parameters["non_blocking_read"] == "True");

  // Specify gain for servoing to position in joint space.
  // A higher gain can sharpen the trajectory.
  const int servoj_gain = stoi(info_.hardware_parameters["servoj_gain"]);
  // Specify lookahead time for servoing to position in joint space.
  // A longer lookahead time can smooth the trajectory.
  const double servoj_lookahead_time = stod(info_.hardware_parameters["servoj_lookahead_time"]);

  const bool use_tool_communication = (info_.hardware_parameters["use_tool_communication"] == "true") ||
                                      (info_.hardware_parameters["use_tool_communication"] == "True");

  // Hash of the calibration reported by the robot. This is used for validating the robot
  // description is using the correct calibration. If the robot's calibration doesn't match this
  // hash, an error will be printed. You can use the robot as usual, however Cartesian poses of the
  // endeffector might be inaccurate. See the "ur_calibration" package on help how to generate your
  // own hash matching your actual robot.
  const std::string calibration_checksum = info_.hardware_parameters["kinematics/hash"];

  std::unique_ptr<urcl::ToolCommSetup> tool_comm_setup;
  if (use_tool_communication) {
    tool_comm_setup = std::make_unique<urcl::ToolCommSetup>();

    using ToolVoltageT = std::underlying_type<urcl::ToolVoltage>::type;

    // Tool voltage that will be set as soon as the UR-Program on the robot is started. Note: This
    // parameter is only evaluated, when the parameter "use_tool_communication" is set to TRUE.
    // Then, this parameter is required.}
    const ToolVoltageT tool_voltage = std::stoi(info_.hardware_parameters["tool_voltage"]);
    tool_comm_setup->setToolVoltage(static_cast<urcl::ToolVoltage>(tool_voltage));

    using ParityT = std::underlying_type<urcl::Parity>::type;

    // Parity used for tool communication. Will be set as soon as the UR-Program on the robot is
    // started. Can be 0 (None), 1 (odd) and 2 (even).
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    const ParityT parity = std::stoi(info_.hardware_parameters["tool_parity"]);
    tool_comm_setup->setParity(static_cast<urcl::Parity>(parity));

    // Baud rate used for tool communication. Will be set as soon as the UR-Program on the robot is
    // started. See UR documentation for valid baud rates.
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    const int baud_rate = std::stoi(info_.hardware_parameters["tool_baud_rate"]);
    tool_comm_setup->setBaudRate(static_cast<uint32_t>(baud_rate));

    // Number of stop bits used for tool communication. Will be set as soon as the UR-Program on the robot is
    // started. Can be 1 or 2.
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    const int stop_bits = std::stoi(info_.hardware_parameters["tool_stop_bits"]);
    tool_comm_setup->setStopBits(static_cast<uint32_t>(stop_bits));

    // Number of idle chars for the RX unit used for tool communication. Will be set as soon as the UR-Program on the
    // robot is started. Valid values: min=1.0, max=40.0
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    const int rx_idle_chars = std::stoi(info_.hardware_parameters["tool_rx_idle_chars"]);
    tool_comm_setup->setRxIdleChars(rx_idle_chars);

    // Number of idle chars for the TX unit used for tool communication. Will be set as soon as the UR-Program on the
    // robot is started. Valid values: min=0.0, max=40.0
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    const int tx_idle_chars = std::stoi(info_.hardware_parameters["tool_tx_idle_chars"]);
    tool_comm_setup->setTxIdleChars(tx_idle_chars);
  }

  // Obtain the tf_prefix which is needed for the logging handler so that log messages from different arms are
  // distiguishable in the log
  const std::string tf_prefix = info_.hardware_parameters.at("tf_prefix");
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Initializing driver...");
  registerUrclLogHandler(tf_prefix);
  try {
    rtde_comm_has_been_started_ = false;
    urcl::UrDriverConfiguration driver_config;
    driver_config.robot_ip = robot_ip;
    driver_config.script_file = script_filename;
    driver_config.output_recipe_file = output_recipe_filename;
    driver_config.input_recipe_file = input_recipe_filename;
    driver_config.headless_mode = headless_mode;
    driver_config.reverse_port = static_cast<uint32_t>(reverse_port);
    driver_config.script_sender_port = static_cast<uint32_t>(script_sender_port);
    driver_config.trajectory_port = static_cast<uint32_t>(trajectory_port);
    driver_config.script_command_port = static_cast<uint32_t>(script_command_port);
    driver_config.reverse_ip = reverse_ip;
    driver_config.servoj_gain = static_cast<uint32_t>(servoj_gain);
    driver_config.servoj_lookahead_time = servoj_lookahead_time;
    driver_config.non_blocking_read = non_blocking_read_;
    driver_config.tool_comm_setup = std::move(tool_comm_setup);
    driver_config.handle_program_state =
        std::bind(&URPositionHardwareInterface::handleRobotProgramState, this, std::placeholders::_1);
    ur_driver_ = std::make_shared<urcl::UrDriver>(driver_config);
  } catch (urcl::ToolCommNotAvailable& e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("URPositionHardwareInterface"), "See parameter use_tool_communication");

    return hardware_interface::CallbackReturn::ERROR;
  } catch (urcl::UrException& e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("URPositionHardwareInterface"), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Timeout before the reverse interface will be dropped by the robot
  receive_timeout_ = urcl::RobotReceiveTimeout::sec(std::stof(info_.hardware_parameters["robot_receive_timeout"]));

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Calibration checksum: '%s'.",
              calibration_checksum.c_str());
  // check calibration
  // https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/c3378599d5fa73a261328b326392e847f312ab6b/ur_robot_driver/src/hardware_interface.cpp#L296-L309
  if (ur_driver_->checkCalibration(calibration_checksum)) {
    RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Calibration checked successfully.");
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("URPositionHardwareInterface"),

                        "The calibration parameters of the connected robot don't match the ones from the given "
                        "kinematics config file. Please be aware that this can lead to critical inaccuracies of tcp "
                        "positions. Use the ur_calibration tool to extract the correct calibration from the robot and "
                        "pass that into the description. See "
                        "[https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_calibration/"
                        "README.md] for details.");
  }

  // Export version information to state interfaces
  urcl::VersionInformation version_info = ur_driver_->getVersion();
  get_robot_software_version_major_ = version_info.major;
  get_robot_software_version_minor_ = version_info.minor;
  get_robot_software_version_build_ = version_info.build;
  get_robot_software_version_bugfix_ = version_info.bugfix;

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Initializing InstructionExecutor");
  instruction_executor_ = std::make_shared<urcl::InstructionExecutor>(ur_driver_);

  async_thread_ = std::make_shared<std::thread>(&URPositionHardwareInterface::asyncThread, this);

  // Start async thread for sending motion primitives
  async_moprim_cmd_thread_ = std::make_shared<std::thread>(&URPositionHardwareInterface::asyncMoprimCmdThread, this);
  async_moprim_stop_thread_ = std::make_shared<std::thread>(&URPositionHardwareInterface::asyncMoprimStopThread, this);

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully started!");

  ur_driver_->registerTrajectoryDoneCallback(
      std::bind(&URPositionHardwareInterface::trajectory_done_callback, this,
                std::placeholders::_1));  // conflict with instruction_executor_ --> callback function needs to get
                                          // changed when using instruction executor

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
URPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Activating HW interface");

  for (size_t i = 0; i < 6; i++) {
    force_mode_task_frame_[i] = NO_NEW_CMD_;
    force_mode_selection_vector_[i] = static_cast<uint32_t>(NO_NEW_CMD_);
    force_mode_wrench_[i] = NO_NEW_CMD_;
    force_mode_limits_[i] = NO_NEW_CMD_;
  }
  force_mode_type_ = static_cast<unsigned int>(NO_NEW_CMD_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
URPositionHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(rclcpp::get_logger("URPositionHardwareInterface"), "on_cleanup");
  return stop();
}

hardware_interface::CallbackReturn
URPositionHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(rclcpp::get_logger("URPositionHardwareInterface"), "on_shutdown");
  return stop();
}

hardware_interface::CallbackReturn URPositionHardwareInterface::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Stopping ...please wait...");

  if (async_thread_) {
    async_thread_shutdown_ = true;
    async_thread_->join();
    async_thread_.reset();
  }
  if (async_moprim_cmd_thread_) {
    async_moprim_thread_shutdown_ = true;
    async_moprim_cmd_thread_->join();
    async_moprim_cmd_thread_.reset();
  }
  if (async_moprim_stop_thread_) {
    async_moprim_thread_shutdown_ = true;
    async_moprim_stop_thread_->join();
    async_moprim_stop_thread_.reset();
  }

  ur_driver_.reset();

  unregisterUrclLogHandler();

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

template <typename T>
void URPositionHardwareInterface::readData(const std::unique_ptr<rtde::DataPackage>& data_pkg,
                                           const std::string& var_name, T& data)
{
  if (!data_pkg->getData(var_name, data)) {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }
}

template <typename T, size_t N>
void URPositionHardwareInterface::readBitsetData(const std::unique_ptr<rtde::DataPackage>& data_pkg,
                                                 const std::string& var_name, std::bitset<N>& data)
{
  if (!data_pkg->getData<T, N>(var_name, data)) {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }
}

void URPositionHardwareInterface::asyncThread()
{
  async_thread_shutdown_ = false;
  while (!async_thread_shutdown_) {
    if (initialized_) {
      //        RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Initialized in async thread");
      checkAsyncIO();
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(20000000));
  }
}

hardware_interface::return_type URPositionHardwareInterface::read(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period)
{
  // We want to start the rtde comm the latest point possible due to the delay times arising from setting up the
  // communication with multiple arms
  if (!rtde_comm_has_been_started_) {
    ur_driver_->startRTDECommunication();
    rtde_comm_has_been_started_ = true;
  }
  std::unique_ptr<rtde::DataPackage> data_pkg = ur_driver_->getDataPackage();

  if (data_pkg) {
    packet_read_ = true;
    readData(data_pkg, "actual_q", urcl_joint_positions_);
    readData(data_pkg, "actual_qd", urcl_joint_velocities_);
    readData(data_pkg, "actual_current", urcl_joint_efforts_);

    readData(data_pkg, "target_speed_fraction", target_speed_fraction_);
    readData(data_pkg, "speed_scaling", speed_scaling_);
    readData(data_pkg, "runtime_state", runtime_state_);
    readData(data_pkg, "actual_TCP_force", urcl_ft_sensor_measurements_);
    readData(data_pkg, "actual_TCP_pose", urcl_tcp_pose_);
    readData(data_pkg, "target_TCP_pose", urcl_target_tcp_pose_);
    readData(data_pkg, "standard_analog_input0", standard_analog_input_[0]);
    readData(data_pkg, "standard_analog_input1", standard_analog_input_[1]);
    readData(data_pkg, "standard_analog_output0", standard_analog_output_[0]);
    readData(data_pkg, "standard_analog_output1", standard_analog_output_[1]);
    readData(data_pkg, "tool_mode", tool_mode_);
    readData(data_pkg, "tool_analog_input0", tool_analog_input_[0]);
    readData(data_pkg, "tool_analog_input1", tool_analog_input_[1]);
    readData(data_pkg, "tool_output_voltage", tool_output_voltage_);
    readData(data_pkg, "tool_output_current", tool_output_current_);
    readData(data_pkg, "tool_temperature", tool_temperature_);
    readData(data_pkg, "robot_mode", robot_mode_);
    readData(data_pkg, "safety_mode", safety_mode_);
    readBitsetData<uint32_t>(data_pkg, "robot_status_bits", robot_status_bits_);
    readBitsetData<uint32_t>(data_pkg, "safety_status_bits", safety_status_bits_);
    readBitsetData<uint64_t>(data_pkg, "actual_digital_input_bits", actual_dig_in_bits_);
    readBitsetData<uint64_t>(data_pkg, "actual_digital_output_bits", actual_dig_out_bits_);
    readBitsetData<uint32_t>(data_pkg, "analog_io_types", analog_io_types_);
    readBitsetData<uint32_t>(data_pkg, "tool_analog_input_types", tool_analog_input_types_);
    readData(data_pkg, "tcp_offset", tcp_offset_);

    // required transforms
    extractToolPose();
    transformForceTorque();

    // TODO(anyone): logic for sending other stuff to higher level interface

    // pausing state follows runtime state when pausing
    if (runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PAUSED)) {
      pausing_state_ = PausingState::PAUSED;
    } else if (runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PLAYING) &&
               pausing_state_ == PausingState::PAUSED) {
      // When the robot resumed program execution and pausing state was PAUSED, we enter RAMPUP
      speed_scaling_combined_ = 0.0;
      pausing_state_ = PausingState::RAMPUP;
    }

    if (pausing_state_ == PausingState::RAMPUP) {
      double speed_scaling_ramp = speed_scaling_combined_ + pausing_ramp_up_increment_;
      speed_scaling_combined_ = std::min(speed_scaling_ramp, speed_scaling_ * target_speed_fraction_);

      if (speed_scaling_ramp > speed_scaling_ * target_speed_fraction_) {
        pausing_state_ = PausingState::RUNNING;
      }
    } else if (runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::RESUMING)) {
      // We have to keep speed scaling on ROS side at 0 during RESUMING to prevent controllers from
      // continuing to interpolate
      speed_scaling_combined_ = 0.0;
    } else {
      // Normal case
      speed_scaling_combined_ = speed_scaling_ * target_speed_fraction_;
    }

    if (first_pass_ && !initialized_) {
      initAsyncIO();
      // initialize commands
      urcl_position_commands_ = urcl_position_commands_old_ = urcl_joint_positions_;
      urcl_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
      target_speed_fraction_cmd_ = NO_NEW_CMD_;
      resend_robot_program_cmd_ = NO_NEW_CMD_;
      zero_ftsensor_cmd_ = NO_NEW_CMD_;
      hand_back_control_cmd_ = NO_NEW_CMD_;
      force_mode_disable_cmd_ = NO_NEW_CMD_;
      freedrive_mode_abort_ = NO_NEW_CMD_;
      freedrive_mode_enable_ = NO_NEW_CMD_;
      initialized_ = true;
    }

    updateNonDoubleValues();

    // Motion primitives stuff
    hw_moprim_states_[0] = current_moprim_execution_status_;
    hw_moprim_states_[1] = static_cast<double>(ready_for_new_moprim_);

    return hardware_interface::return_type::OK;
  }
  if (!non_blocking_read_)
    RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Unable to read from hardware...");
  // TODO(anyone): could not read from the driver --> return ERROR --> on error will be called
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type URPositionHardwareInterface::write(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
{
  // If there is no interpreting program running on the robot, we do not want to send anything.
  // TODO(anyone): We would still like to disable the controllers requiring a writable interface. In ROS1
  // this was done externally using the controller_stopper.
  if ((runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PLAYING) ||
       runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PAUSING)) &&
      robot_program_running_ && (!non_blocking_read_ || packet_read_)) {
    if (position_controller_running_) {
      ur_driver_->writeJointCommand(urcl_position_commands_, urcl::comm::ControlMode::MODE_SERVOJ, receive_timeout_);

    } else if (velocity_controller_running_) {
      ur_driver_->writeJointCommand(urcl_velocity_commands_, urcl::comm::ControlMode::MODE_SPEEDJ, receive_timeout_);

    } else if (freedrive_mode_controller_running_ && freedrive_activated_) {
      ur_driver_->writeFreedriveControlMessage(urcl::control::FreedriveControlMessage::FREEDRIVE_NOOP);

    } else if (passthrough_trajectory_controller_running_) {
      ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP);
      check_passthrough_trajectory_controller();

    } else if (motion_primitives_forward_controller_running_) {
      handleMoprimCommands();

    } else {
      ur_driver_->writeKeepalive();
    }

    if (!std::isnan(force_mode_task_frame_[0]) && !std::isnan(force_mode_selection_vector_[0]) &&
        !std::isnan(force_mode_wrench_[0]) && !std::isnan(force_mode_type_) && !std::isnan(force_mode_limits_[0]) &&
        !std::isnan(force_mode_damping_) && !std::isnan(force_mode_gain_scaling_) && ur_driver_ != nullptr) {
      start_force_mode();
    } else if (!std::isnan(force_mode_disable_cmd_) && ur_driver_ != nullptr && force_mode_async_success_ == 2.0) {
      stop_force_mode();
    }

    packet_read_ = false;
  }

  return hardware_interface::return_type::OK;
}

void URPositionHardwareInterface::handleRobotProgramState(bool program_running)
{
  robot_program_running_ = program_running;
}

void URPositionHardwareInterface::initAsyncIO()
{
  for (size_t i = 0; i < 18; ++i) {
    standard_dig_out_bits_cmd_[i] = NO_NEW_CMD_;
  }

  for (size_t i = 0; i < 2; ++i) {
    standard_analog_output_cmd_[i] = NO_NEW_CMD_;
  }

  analog_output_domain_cmd_ = NO_NEW_CMD_;

  tool_voltage_cmd_ = NO_NEW_CMD_;

  payload_mass_ = NO_NEW_CMD_;
  payload_center_of_gravity_ = { NO_NEW_CMD_, NO_NEW_CMD_, NO_NEW_CMD_ };
}

void URPositionHardwareInterface::checkAsyncIO()
{
  if (!rtde_comm_has_been_started_) {
    return;
  }
  for (size_t i = 0; i < 18; ++i) {
    if (!std::isnan(standard_dig_out_bits_cmd_[i]) && ur_driver_ != nullptr) {
      if (i <= 7) {
        io_async_success_ =
            ur_driver_->getRTDEWriter().sendStandardDigitalOutput(i, static_cast<bool>(standard_dig_out_bits_cmd_[i]));
      } else if (i <= 15) {
        io_async_success_ = ur_driver_->getRTDEWriter().sendConfigurableDigitalOutput(
            static_cast<uint8_t>(i - 8), static_cast<bool>(standard_dig_out_bits_cmd_[i]));
      } else {
        io_async_success_ = ur_driver_->getRTDEWriter().sendToolDigitalOutput(
            static_cast<uint8_t>(i - 16), static_cast<bool>(standard_dig_out_bits_cmd_[i]));
      }
      standard_dig_out_bits_cmd_[i] = NO_NEW_CMD_;
    }
  }

  for (size_t i = 0; i < 2; ++i) {
    if (!std::isnan(standard_analog_output_cmd_[i]) && ur_driver_ != nullptr) {
      urcl::AnalogOutputType domain = urcl::AnalogOutputType::SET_ON_TEACH_PENDANT;
      if (!std::isnan(analog_output_domain_cmd_) && ur_driver_ != nullptr) {
        domain = static_cast<urcl::AnalogOutputType>(analog_output_domain_cmd_);
        analog_output_domain_cmd_ = NO_NEW_CMD_;
      }
      io_async_success_ =
          ur_driver_->getRTDEWriter().sendStandardAnalogOutput(i, standard_analog_output_cmd_[i], domain);
      standard_analog_output_cmd_[i] = NO_NEW_CMD_;
    }
  }

  if (!std::isnan(tool_voltage_cmd_) && ur_driver_ != nullptr) {
    io_async_success_ = ur_driver_->setToolVoltage(static_cast<urcl::ToolVoltage>(tool_voltage_cmd_));
    tool_voltage_cmd_ = NO_NEW_CMD_;
  }

  if (!std::isnan(target_speed_fraction_cmd_) && ur_driver_ != nullptr) {
    scaling_async_success_ = ur_driver_->getRTDEWriter().sendSpeedSlider(target_speed_fraction_cmd_);
    target_speed_fraction_cmd_ = NO_NEW_CMD_;
  }

  if (!std::isnan(resend_robot_program_cmd_) && ur_driver_ != nullptr) {
    try {
      resend_robot_program_async_success_ = ur_driver_->sendRobotProgram();
    } catch (const urcl::UrException& e) {
      RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Service Call failed: '%s'", e.what());
    }
    resend_robot_program_cmd_ = NO_NEW_CMD_;
  }

  if (!std::isnan(hand_back_control_cmd_) && ur_driver_ != nullptr) {
    robot_program_running_ = false;
    hand_back_control_async_success_ = true;
    hand_back_control_cmd_ = NO_NEW_CMD_;
  }

  if (!std::isnan(payload_mass_) && !std::isnan(payload_center_of_gravity_[0]) &&
      !std::isnan(payload_center_of_gravity_[1]) && !std::isnan(payload_center_of_gravity_[2]) &&
      ur_driver_ != nullptr) {
    payload_async_success_ = ur_driver_->setPayload(payload_mass_, payload_center_of_gravity_);
    payload_mass_ = NO_NEW_CMD_;
    payload_center_of_gravity_ = { NO_NEW_CMD_, NO_NEW_CMD_, NO_NEW_CMD_ };
  }

  if (!std::isnan(zero_ftsensor_cmd_) && ur_driver_ != nullptr) {
    zero_ftsensor_async_success_ = ur_driver_->zeroFTSensor();
    zero_ftsensor_cmd_ = NO_NEW_CMD_;
  }

  if (!std::isnan(freedrive_mode_enable_) && ur_driver_ != nullptr) {
    RCLCPP_INFO(get_logger(), "Starting freedrive mode.");
    freedrive_mode_async_success_ =
        ur_driver_->writeFreedriveControlMessage(urcl::control::FreedriveControlMessage::FREEDRIVE_START);
    freedrive_mode_enable_ = NO_NEW_CMD_;
    freedrive_activated_ = true;
  }

  if (!std::isnan(freedrive_mode_abort_) && freedrive_mode_abort_ == 1.0 && freedrive_activated_ &&
      ur_driver_ != nullptr) {
    RCLCPP_INFO(get_logger(), "Stopping freedrive mode.");
    freedrive_mode_async_success_ =
        ur_driver_->writeFreedriveControlMessage(urcl::control::FreedriveControlMessage::FREEDRIVE_STOP);
    freedrive_activated_ = false;
    freedrive_mode_abort_ = NO_NEW_CMD_;
  }
}

void URPositionHardwareInterface::updateNonDoubleValues()
{
  for (size_t i = 0; i < 18; ++i) {
    actual_dig_out_bits_copy_[i] = static_cast<double>(actual_dig_out_bits_[i]);
    actual_dig_in_bits_copy_[i] = static_cast<double>(actual_dig_in_bits_[i]);
  }

  for (size_t i = 0; i < 11; ++i) {
    safety_status_bits_copy_[i] = static_cast<double>(safety_status_bits_[i]);
  }

  for (size_t i = 0; i < 4; ++i) {
    analog_io_types_copy_[i] = static_cast<double>(analog_io_types_[i]);
    robot_status_bits_copy_[i] = static_cast<double>(robot_status_bits_[i]);
  }

  for (size_t i = 0; i < 2; ++i) {
    tool_analog_input_types_copy_[i] = static_cast<double>(tool_analog_input_types_[i]);
  }

  tool_output_voltage_copy_ = static_cast<double>(tool_output_voltage_);
  robot_mode_copy_ = static_cast<double>(robot_mode_);
  safety_mode_copy_ = static_cast<double>(safety_mode_);
  tool_mode_copy_ = static_cast<double>(tool_mode_);
  system_interface_initialized_ = initialized_ ? 1.0 : 0.0;
  robot_program_running_copy_ = robot_program_running_ ? 1.0 : 0.0;
}

void URPositionHardwareInterface::transformForceTorque()
{
  KDL::Wrench ft(
      KDL::Vector(urcl_ft_sensor_measurements_[0], urcl_ft_sensor_measurements_[1], urcl_ft_sensor_measurements_[2]),
      KDL::Vector(urcl_ft_sensor_measurements_[3], urcl_ft_sensor_measurements_[4], urcl_ft_sensor_measurements_[5]));
  if (ur_driver_->getVersion().major >= 5)  // e-Series
  {
    // Setup necessary frames
    KDL::Vector vec = KDL::Vector(tcp_offset_[3], tcp_offset_[4], tcp_offset_[5]);
    double angle = vec.Normalize();
    KDL::Rotation rotation = KDL::Rotation::Rot(vec, angle);
    KDL::Frame flange_to_tcp = KDL::Frame(rotation, KDL::Vector(tcp_offset_[0], tcp_offset_[1], tcp_offset_[2]));

    vec = KDL::Vector(urcl_target_tcp_pose_[3], urcl_target_tcp_pose_[4], urcl_target_tcp_pose_[5]);
    angle = vec.Normalize();
    rotation = KDL::Rotation::Rot(vec, angle);
    KDL::Frame base_to_tcp =
        KDL::Frame(rotation, KDL::Vector(urcl_target_tcp_pose_[0], urcl_target_tcp_pose_[1], urcl_target_tcp_pose_[2]));
    // Calculate transformation from base to flange, see calculation details below
    // `base_to_tcp = base_to_flange*flange_to_tcp -> base_to_flange = base_to_tcp * inv(flange_to_tcp)`
    KDL::Frame base_to_flange = base_to_tcp * flange_to_tcp.Inverse();
    // rotate f/t sensor output back to the flange frame
    ft = base_to_flange.M.Inverse() * ft;

    // Transform the wrench to the tcp frame
    ft = flange_to_tcp * ft;
  } else {  // CB3
    KDL::Vector vec = KDL::Vector(urcl_target_tcp_pose_[3], urcl_target_tcp_pose_[4], urcl_target_tcp_pose_[5]);
    double angle = vec.Normalize();
    KDL::Rotation base_to_tcp_rot = KDL::Rotation::Rot(vec, angle);

    // rotate f/t sensor output back to the tcp frame
    ft = base_to_tcp_rot.Inverse() * ft;
  }
  urcl_ft_sensor_measurements_ = { ft[0], ft[1], ft[2], ft[3], ft[4], ft[5] };
}

void URPositionHardwareInterface::extractToolPose()
{
  // imported from ROS1 driver hardware_interface.cpp#L911-L928
  double tcp_angle =
      std::sqrt(std::pow(urcl_tcp_pose_[3], 2) + std::pow(urcl_tcp_pose_[4], 2) + std::pow(urcl_tcp_pose_[5], 2));

  tf2::Vector3 rotation_vec(urcl_tcp_pose_[3], urcl_tcp_pose_[4], urcl_tcp_pose_[5]);
  if (tcp_angle > 1e-16) {
    tcp_rotation_quat_.setRotation(rotation_vec.normalized(), tcp_angle);
  } else {
    tcp_rotation_quat_.setValue(0.0, 0.0, 0.0, 1.0);  // default Quaternion is 0,0,0,0 which is invalid
  }
  tcp_rotation_buffer.set(tcp_rotation_quat_);
}

hardware_interface::return_type URPositionHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  start_modes_ = std::vector<std::vector<std::string>>(info_.joints.size());
  stop_modes_ = std::vector<std::vector<uint>>(info_.joints.size());
  std::vector<std::vector<std::string>> control_modes(info_.joints.size());
  const std::string tf_prefix = info_.hardware_parameters.at("tf_prefix");

  // Assess current state
  for (auto i = 0u; i < info_.joints.size(); i++) {
    if (position_controller_running_) {
      control_modes[i] = { hardware_interface::HW_IF_POSITION };
    }
    if (velocity_controller_running_) {
      control_modes[i] = { hardware_interface::HW_IF_VELOCITY };
    }
    if (force_mode_controller_running_) {
      control_modes[i].push_back(FORCE_MODE_GPIO);
    }
    if (passthrough_trajectory_controller_running_) {
      control_modes[i].push_back(PASSTHROUGH_GPIO);
    }
    if (freedrive_mode_controller_running_) {
      control_modes[i].push_back(FREEDRIVE_MODE_GPIO);
    }
    if (motion_primitives_forward_controller_running_) {
      control_modes[i].push_back(HW_IF_MOTION_PRIMITIVES);
    }
  }

  if (!std::all_of(start_modes_.begin() + 1, start_modes_.end(),
                   [&](const std::vector<std::string>& other) { return other == start_modes_[0]; })) {
    RCLCPP_ERROR(get_logger(), "Start modes of all joints have to be the same.");
    return hardware_interface::return_type::ERROR;
  }

  // Starting interfaces
  // If a joint has been reserved already, raise an error.
  // Modes that are not directly mapped to a single joint such as force_mode reserve all joints.
  for (const auto& key : start_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        if (!start_modes_[i].empty()) {
          RCLCPP_ERROR(get_logger(), "Attempting to start position control while there is another control mode already "
                                     "requested.");
          return hardware_interface::return_type::ERROR;
        }
        start_modes_[i] = { hardware_interface::HW_IF_POSITION };
      } else if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        if (!start_modes_[i].empty()) {
          RCLCPP_ERROR(get_logger(), "Attempting to start velocity control while there is another control mode already "
                                     "requested.");
          return hardware_interface::return_type::ERROR;
        }
        start_modes_[i] = { hardware_interface::HW_IF_VELOCITY };
      } else if (key == tf_prefix + FORCE_MODE_GPIO + "/type") {
        if (std::any_of(start_modes_[i].begin(), start_modes_[i].end(), [&](const std::string& item) {
              return item == hardware_interface::HW_IF_POSITION || item == hardware_interface::HW_IF_VELOCITY;
            })) {
          RCLCPP_ERROR(get_logger(), "Attempting to start force_mode control while there is either position or "
                                     "velocity mode already requested by another controller.");
          return hardware_interface::return_type::ERROR;
        }
        start_modes_[i].push_back(FORCE_MODE_GPIO);
      } else if (key == tf_prefix + PASSTHROUGH_GPIO + "/setpoint_positions_" + std::to_string(i)) {
        if (std::any_of(start_modes_[i].begin(), start_modes_[i].end(), [&](const std::string& item) {
              return item == hardware_interface::HW_IF_POSITION || item == hardware_interface::HW_IF_VELOCITY;
            })) {
          RCLCPP_ERROR(get_logger(), "Attempting to start trajectory passthrough control while there is either "
                                     "position or velocity mode already requested by another controller.");
          return hardware_interface::return_type::ERROR;
        }
        start_modes_[i].push_back(PASSTHROUGH_GPIO);
      } else if (key == tf_prefix + FREEDRIVE_MODE_GPIO + "/async_success") {
        if (std::any_of(start_modes_[i].begin(), start_modes_[i].end(), [&](const std::string& item) {
              return item == hardware_interface::HW_IF_POSITION || item == hardware_interface::HW_IF_VELOCITY ||
                     item == PASSTHROUGH_GPIO || item == FORCE_MODE_GPIO;
            })) {
          return hardware_interface::return_type::ERROR;
        }
        start_modes_[i].push_back(FREEDRIVE_MODE_GPIO);

      } else if (key == tf_prefix + HW_IF_MOTION_PRIMITIVES + "/motion_type") {
        if (std::any_of(start_modes_[i].begin(), start_modes_[i].end(), [&](const std::string& item) {
              return item == hardware_interface::HW_IF_POSITION || item == hardware_interface::HW_IF_VELOCITY ||
                     item == PASSTHROUGH_GPIO || item == FORCE_MODE_GPIO;
            })) {
          return hardware_interface::return_type::ERROR;
        }
        start_modes_[i].push_back(HW_IF_MOTION_PRIMITIVES);
      }
    }
  }

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto& key : stop_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        stop_modes_[i].push_back(StoppingInterface::STOP_POSITION);
        control_modes[i].erase(
            std::remove_if(control_modes[i].begin(), control_modes[i].end(),
                           [](const std::string& item) { return item == hardware_interface::HW_IF_POSITION; }),
            control_modes[i].end());
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        stop_modes_[i].push_back(StoppingInterface::STOP_VELOCITY);
        control_modes[i].erase(
            std::remove_if(control_modes[i].begin(), control_modes[i].end(),
                           [](const std::string& item) { return item == hardware_interface::HW_IF_VELOCITY; }),
            control_modes[i].end());
      }
      if (key == tf_prefix + FORCE_MODE_GPIO + "/disable_cmd") {
        stop_modes_[i].push_back(StoppingInterface::STOP_FORCE_MODE);
        control_modes[i].erase(std::remove_if(control_modes[i].begin(), control_modes[i].end(),
                                              [&](const std::string& item) { return item == FORCE_MODE_GPIO; }),
                               control_modes[i].end());
      }
      if (key == tf_prefix + PASSTHROUGH_GPIO + "/setpoint_positions_" + std::to_string(i)) {
        stop_modes_[i].push_back(StoppingInterface::STOP_PASSTHROUGH);
        control_modes[i].erase(std::remove_if(control_modes[i].begin(), control_modes[i].end(),
                                              [&](const std::string& item) { return item == PASSTHROUGH_GPIO; }),
                               control_modes[i].end());
      }
      if (key == tf_prefix + FREEDRIVE_MODE_GPIO + "/async_success") {
        stop_modes_[i].push_back(StoppingInterface::STOP_FREEDRIVE);
        control_modes[i].erase(std::remove_if(control_modes[i].begin(), control_modes[i].end(),
                                              [&](const std::string& item) { return item == FREEDRIVE_MODE_GPIO; }),
                               control_modes[i].end());
      }
      if (key == tf_prefix + HW_IF_MOTION_PRIMITIVES + "/motion_type") {
        stop_modes_[i].push_back(StoppingInterface::STOP_MOTION_PRIMITIVES);
        control_modes[i].erase(std::remove_if(control_modes[i].begin(), control_modes[i].end(),
                                              [&](const std::string& item) { return item == HW_IF_MOTION_PRIMITIVES; }),
                               control_modes[i].end());
      }
    }
  }

  // Do not start conflicting controllers
  // Passthrough controller requested to start
  if (std::any_of(start_modes_[0].begin(), start_modes_[0].end(),
                  [this](auto& item) { return (item == PASSTHROUGH_GPIO); }) &&
      (std::any_of(start_modes_[0].begin(), start_modes_[0].end(),
                   [this](auto& item) {
                     return (item == hardware_interface::HW_IF_VELOCITY || item == hardware_interface::HW_IF_POSITION ||
                             item == FREEDRIVE_MODE_GPIO || item == HW_IF_MOTION_PRIMITIVES);
                   }) ||
       std::any_of(control_modes[0].begin(), control_modes[0].end(), [this](auto& item) {
         return (item == hardware_interface::HW_IF_VELOCITY || item == hardware_interface::HW_IF_POSITION ||
                 item == FREEDRIVE_MODE_GPIO || item == HW_IF_MOTION_PRIMITIVES);
       }))) {
    RCLCPP_ERROR(get_logger(), "Attempting to start passthrough_trajectory control while there is either position or "
                               "velocity or motion primitives or freedrive mode running.");
    ret_val = hardware_interface::return_type::ERROR;
  }

  // Force mode requested to start
  if (std::any_of(start_modes_[0].begin(), start_modes_[0].end(),
                  [this](auto& item) { return (item == FORCE_MODE_GPIO); }) &&
      (std::any_of(start_modes_[0].begin(), start_modes_[0].end(),
                   [this](auto& item) {
                     return (item == hardware_interface::HW_IF_VELOCITY || item == hardware_interface::HW_IF_POSITION ||
                             item == FREEDRIVE_MODE_GPIO || item == HW_IF_MOTION_PRIMITIVES);
                   }) ||
       std::any_of(control_modes[0].begin(), control_modes[0].end(), [this](auto& item) {
         return (item == hardware_interface::HW_IF_VELOCITY || item == hardware_interface::HW_IF_POSITION ||
                 item == FORCE_MODE_GPIO || item == FREEDRIVE_MODE_GPIO || item == HW_IF_MOTION_PRIMITIVES);
       }))) {
    RCLCPP_ERROR(get_logger(), "Attempting to start force mode control while there is either position or "
                               "velocity or motion primitives mode running.");
    ret_val = hardware_interface::return_type::ERROR;
  }

  // Freedrive mode requested to start
  if (std::any_of(start_modes_[0].begin(), start_modes_[0].end(),
                  [this](auto& item) { return (item == FREEDRIVE_MODE_GPIO); }) &&
      (std::any_of(start_modes_[0].begin(), start_modes_[0].end(),
                   [this](auto& item) {
                     return (item == hardware_interface::HW_IF_VELOCITY || item == hardware_interface::HW_IF_POSITION ||
                             item == PASSTHROUGH_GPIO || item == FORCE_MODE_GPIO || item == HW_IF_MOTION_PRIMITIVES);
                   }) ||
       std::any_of(control_modes[0].begin(), control_modes[0].end(), [this](auto& item) {
         return (item == hardware_interface::HW_IF_VELOCITY || item == hardware_interface::HW_IF_POSITION ||
                 item == PASSTHROUGH_GPIO || item == FORCE_MODE_GPIO || item == HW_IF_MOTION_PRIMITIVES);
       }))) {
    RCLCPP_ERROR(get_logger(), "Attempting to start force mode control while there is either position or "
                               "velocity or motion primitives mode running.");
    ret_val = hardware_interface::return_type::ERROR;
  }

  // Position mode requested to start
  if (std::any_of(start_modes_[0].begin(), start_modes_[0].end(),
                  [](auto& item) { return (item == hardware_interface::HW_IF_POSITION); }) &&
      (std::any_of(start_modes_[0].begin(), start_modes_[0].end(),
                   [this](auto& item) {
                     return (item == hardware_interface::HW_IF_VELOCITY || item == PASSTHROUGH_GPIO ||
                             item == FORCE_MODE_GPIO || item == FREEDRIVE_MODE_GPIO || item == HW_IF_MOTION_PRIMITIVES);
                   }) ||
       std::any_of(control_modes[0].begin(), control_modes[0].end(), [this](auto& item) {
         return (item == hardware_interface::HW_IF_VELOCITY || item == hardware_interface::HW_IF_POSITION ||
                 item == PASSTHROUGH_GPIO || item == FORCE_MODE_GPIO || item == FREEDRIVE_MODE_GPIO ||
                 item == HW_IF_MOTION_PRIMITIVES);
       }))) {
    RCLCPP_ERROR(get_logger(), "Attempting to start position control while there is either trajectory passthrough or "
                               "velocity mode or motion primitives moder or force_mode or freedrive mode running.");
    ret_val = hardware_interface::return_type::ERROR;
  }

  // Velocity mode requested to start
  if (std::any_of(start_modes_[0].begin(), start_modes_[0].end(),
                  [](auto& item) { return (item == hardware_interface::HW_IF_VELOCITY); }) &&
      (std::any_of(start_modes_[0].begin(), start_modes_[0].end(),
                   [this](auto& item) {
                     return (item == hardware_interface::HW_IF_POSITION || item == PASSTHROUGH_GPIO ||
                             item == FORCE_MODE_GPIO || item == FREEDRIVE_MODE_GPIO || item == HW_IF_MOTION_PRIMITIVES);
                   }) ||
       std::any_of(control_modes[0].begin(), control_modes[0].end(), [this](auto& item) {
         return (item == hardware_interface::HW_IF_VELOCITY || item == hardware_interface::HW_IF_POSITION ||
                 item == PASSTHROUGH_GPIO || item == FORCE_MODE_GPIO || item == FREEDRIVE_MODE_GPIO ||
                 item == HW_IF_MOTION_PRIMITIVES);
       }))) {
    RCLCPP_ERROR(get_logger(), "Attempting to start velocity control while there is either trajectory passthrough or "
                               "position mode or motion primitives mode or force_mode or freedrive mode running.");
    ret_val = hardware_interface::return_type::ERROR;
  }

  // Motion primitives mode requested to start
  if (std::any_of(start_modes_[0].begin(), start_modes_[0].end(),
                  [this](auto& item) { return (item == HW_IF_MOTION_PRIMITIVES); }) &&
      (std::any_of(start_modes_[0].begin(), start_modes_[0].end(),
                   [this](auto& item) {
                     return (item == hardware_interface::HW_IF_POSITION || item == hardware_interface::HW_IF_VELOCITY ||
                             item == PASSTHROUGH_GPIO || item == FORCE_MODE_GPIO || item == FREEDRIVE_MODE_GPIO);
                   }) ||
       std::any_of(control_modes[0].begin(), control_modes[0].end(), [this](auto& item) {
         return (item == hardware_interface::HW_IF_VELOCITY || item == hardware_interface::HW_IF_POSITION ||
                 item == PASSTHROUGH_GPIO || item == FORCE_MODE_GPIO || item == FREEDRIVE_MODE_GPIO ||
                 item == HW_IF_MOTION_PRIMITIVES);
       }))) {
    RCLCPP_ERROR(get_logger(), "Attempting to start motion primitives control while there is either trajectory "
                               "passthrough or "
                               "position mode or velocity mode or force_mode or freedrive mode running.");
    ret_val = hardware_interface::return_type::ERROR;
  }

  controllers_initialized_ = true;
  return ret_val;
}

hardware_interface::return_type URPositionHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (stop_modes_[0].size() != 0 && std::find(stop_modes_[0].begin(), stop_modes_[0].end(),
                                              StoppingInterface::STOP_POSITION) != stop_modes_[0].end()) {
    position_controller_running_ = false;
    urcl_position_commands_ = urcl_position_commands_old_ = urcl_joint_positions_;
  } else if (stop_modes_[0].size() != 0 && std::find(stop_modes_[0].begin(), stop_modes_[0].end(),
                                                     StoppingInterface::STOP_VELOCITY) != stop_modes_[0].end()) {
    velocity_controller_running_ = false;
    urcl_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  } else if (stop_modes_[0].size() != 0 && std::find(stop_modes_[0].begin(), stop_modes_[0].end(),
                                                     StoppingInterface::STOP_FORCE_MODE) != stop_modes_[0].end()) {
    force_mode_controller_running_ = false;
    stop_force_mode();
  } else if (stop_modes_[0].size() != 0 && std::find(stop_modes_[0].begin(), stop_modes_[0].end(),
                                                     StoppingInterface::STOP_PASSTHROUGH) != stop_modes_[0].end()) {
    passthrough_trajectory_controller_running_ = false;
    passthrough_trajectory_abort_ = 1.0;
    trajectory_joint_positions_.clear();
    trajectory_joint_accelerations_.clear();
    trajectory_joint_velocities_.clear();
  } else if (stop_modes_.size() != 0 && std::find(stop_modes_[0].begin(), stop_modes_[0].end(),
                                                  StoppingInterface::STOP_FREEDRIVE) != stop_modes_[0].end()) {
    freedrive_mode_controller_running_ = false;
    freedrive_activated_ = false;
    freedrive_mode_abort_ = 1.0;
  } else if (stop_modes_.size() != 0 && std::find(stop_modes_[0].begin(), stop_modes_[0].end(),
                                                  StoppingInterface::STOP_MOTION_PRIMITIVES) != stop_modes_[0].end()) {
    motion_primitives_forward_controller_running_ = false;
    resetMoprimCmdInterfaces();
    current_moprim_execution_status_ = ExecutionState::IDLE;
    ready_for_new_moprim_ = false;

    // use the callback from the hardware_interface implementation (not the one of the instruction_executor_)
    ur_driver_->registerTrajectoryDoneCallback(
        std::bind(&URPositionHardwareInterface::trajectory_done_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Motion primitives mode stopped.");
  }

  if (start_modes_.size() != 0 && std::find(start_modes_[0].begin(), start_modes_[0].end(),
                                            hardware_interface::HW_IF_POSITION) != start_modes_[0].end()) {
    velocity_controller_running_ = false;
    passthrough_trajectory_controller_running_ = false;
    motion_primitives_forward_controller_running_ = false;
    urcl_position_commands_ = urcl_position_commands_old_ = urcl_joint_positions_;
    position_controller_running_ = true;

  } else if (start_modes_[0].size() != 0 && std::find(start_modes_[0].begin(), start_modes_[0].end(),
                                                      hardware_interface::HW_IF_VELOCITY) != start_modes_[0].end()) {
    position_controller_running_ = false;
    passthrough_trajectory_controller_running_ = false;
    motion_primitives_forward_controller_running_ = false;
    urcl_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    velocity_controller_running_ = true;
  } else if (start_modes_[0].size() != 0 &&
             std::find(start_modes_[0].begin(), start_modes_[0].end(), FORCE_MODE_GPIO) != start_modes_[0].end()) {
    motion_primitives_forward_controller_running_ = false;
    force_mode_controller_running_ = true;
  } else if (start_modes_[0].size() != 0 &&
             std::find(start_modes_[0].begin(), start_modes_[0].end(), PASSTHROUGH_GPIO) != start_modes_[0].end()) {
    velocity_controller_running_ = false;
    position_controller_running_ = false;
    motion_primitives_forward_controller_running_ = false;
    passthrough_trajectory_controller_running_ = true;
    passthrough_trajectory_abort_ = 0.0;
  } else if (start_modes_[0].size() != 0 &&
             std::find(start_modes_[0].begin(), start_modes_[0].end(), FREEDRIVE_MODE_GPIO) != start_modes_[0].end()) {
    velocity_controller_running_ = false;
    position_controller_running_ = false;
    motion_primitives_forward_controller_running_ = false;
    freedrive_mode_controller_running_ = true;
    freedrive_activated_ = false;
  } else if (start_modes_[0].size() != 0 && std::find(start_modes_[0].begin(), start_modes_[0].end(),
                                                      HW_IF_MOTION_PRIMITIVES) != start_modes_[0].end()) {
    velocity_controller_running_ = false;
    position_controller_running_ = false;
    freedrive_mode_controller_running_ = false;
    passthrough_trajectory_controller_running_ = false;
    force_mode_controller_running_ = false;

    // use the callback from the instruction_executor_ (not the one of the hardware_interface implementation)
    instruction_executor_->registerTrajDoneCallback();
    resetMoprimCmdInterfaces();
    current_moprim_execution_status_ = ExecutionState::IDLE;
    ready_for_new_moprim_ = true;
    motion_primitives_forward_controller_running_ = true;

    RCLCPP_INFO(get_logger(), "Motion primitives mode started.");
  }

  start_modes_.clear();
  stop_modes_.clear();

  return ret_val;
}

void URPositionHardwareInterface::start_force_mode()
{
  for (size_t i = 0; i < force_mode_selection_vector_.size(); i++) {
    force_mode_selection_vector_copy_[i] = force_mode_selection_vector_[i];
  }
  /* Check version of robot to ensure that the correct startForceMode is called. */
  if (ur_driver_->getVersion().major < 5) {
    force_mode_async_success_ =
        ur_driver_->startForceMode(force_mode_task_frame_, force_mode_selection_vector_copy_, force_mode_wrench_,
                                   force_mode_type_, force_mode_limits_, force_mode_damping_);
    if (force_mode_gain_scaling_ != 0.5) {
      RCLCPP_WARN(rclcpp::get_logger("URPositionHardwareInterface"), "Force mode gain scaling cannot be used on "
                                                                     "CB3 "
                                                                     "robots. Starting force mode, but "
                                                                     "disregarding "
                                                                     "gain scaling.");
    }
  } else {
    force_mode_async_success_ =
        ur_driver_->startForceMode(force_mode_task_frame_, force_mode_selection_vector_copy_, force_mode_wrench_,
                                   force_mode_type_, force_mode_limits_, force_mode_damping_, force_mode_gain_scaling_);
  }

  for (size_t i = 0; i < 6; i++) {
    force_mode_task_frame_[i] = NO_NEW_CMD_;
    force_mode_selection_vector_[i] = static_cast<uint32_t>(NO_NEW_CMD_);
    force_mode_wrench_[i] = NO_NEW_CMD_;
    force_mode_limits_[i] = NO_NEW_CMD_;
  }
  force_mode_type_ = static_cast<unsigned int>(NO_NEW_CMD_);
  force_mode_damping_ = NO_NEW_CMD_;
  force_mode_gain_scaling_ = NO_NEW_CMD_;
}

void URPositionHardwareInterface::stop_force_mode()
{
  force_mode_async_success_ = ur_driver_->endForceMode();
  force_mode_disable_cmd_ = NO_NEW_CMD_;
}

void URPositionHardwareInterface::check_passthrough_trajectory_controller()
{
  static double last_time = 0.0;
  // See passthrough_trajectory_controller.hpp for an explanation of the passthrough_trajectory_transfer_state_ values.

  // We should abort and are not in state IDLE
  if (passthrough_trajectory_abort_ == 1.0 && passthrough_trajectory_transfer_state_ != 0.0) {
    ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL);
  } else if (passthrough_trajectory_transfer_state_ == 2.0) {
    passthrough_trajectory_abort_ = 0.0;
    trajectory_joint_positions_.push_back(passthrough_trajectory_positions_);

    trajectory_times_.push_back(passthrough_trajectory_time_from_start_ - last_time);
    last_time = passthrough_trajectory_time_from_start_;

    if (!std::isnan(passthrough_trajectory_velocities_[0])) {
      trajectory_joint_velocities_.push_back(passthrough_trajectory_velocities_);
    }
    if (!std::isnan(passthrough_trajectory_accelerations_[0])) {
      trajectory_joint_accelerations_.push_back(passthrough_trajectory_accelerations_);
    }
    passthrough_trajectory_transfer_state_ = 1.0;
    /* When all points have been read, write them to the physical robot controller.*/
  } else if (passthrough_trajectory_transfer_state_ == 3.0) {
    /* Tell robot controller how many points are in the trajectory. */
    ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START,
                                              trajectory_joint_positions_.size());
    /* Write the appropriate type of point depending on the combination of positions, velocities and accelerations. */
    if (!has_velocities(trajectory_joint_velocities_) && !has_accelerations(trajectory_joint_accelerations_)) {
      for (size_t i = 0; i < trajectory_joint_positions_.size(); i++) {
        ur_driver_->writeTrajectorySplinePoint(trajectory_joint_positions_[i], urcl::vector6d_t{ 0, 0, 0, 0, 0, 0 },
                                               trajectory_times_[i]);
      }
    } else if (has_velocities(trajectory_joint_velocities_) && !has_accelerations(trajectory_joint_accelerations_)) {
      for (size_t i = 0; i < trajectory_joint_positions_.size(); i++) {
        ur_driver_->writeTrajectorySplinePoint(trajectory_joint_positions_[i], trajectory_joint_velocities_[i],
                                               trajectory_times_[i]);
      }
    } else if (!has_velocities(trajectory_joint_velocities_) && has_accelerations(trajectory_joint_accelerations_)) {
      for (size_t i = 0; i < trajectory_joint_positions_.size(); i++) {
        ur_driver_->writeTrajectorySplinePoint(trajectory_joint_positions_[i], trajectory_joint_accelerations_[i],
                                               trajectory_times_[i]);
      }
    } else if (has_velocities(trajectory_joint_velocities_) && has_accelerations(trajectory_joint_accelerations_)) {
      for (size_t i = 0; i < trajectory_joint_positions_.size(); i++) {
        ur_driver_->writeTrajectorySplinePoint(trajectory_joint_positions_[i], trajectory_joint_velocities_[i],
                                               trajectory_joint_accelerations_[i], trajectory_times_[i]);
      }
    }
    trajectory_joint_positions_.clear();
    trajectory_joint_accelerations_.clear();
    trajectory_joint_velocities_.clear();
    trajectory_times_.clear();
    last_time = 0.0;
    passthrough_trajectory_abort_ = 0.0;
    passthrough_trajectory_transfer_state_ = 4.0;
  }
}

void URPositionHardwareInterface::trajectory_done_callback(urcl::control::TrajectoryResult result)
{
  if (result == urcl::control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE) {
    passthrough_trajectory_abort_ = 1.0;
  } else {
    passthrough_trajectory_abort_ = 0.0;
  }
  passthrough_trajectory_transfer_state_ = 5.0;
  return;
}

bool URPositionHardwareInterface::has_velocities(std::vector<std::array<double, 6>> velocities)
{
  return (velocities.size() > 0);
}

bool URPositionHardwareInterface::has_accelerations(std::vector<std::array<double, 6>> accelerations)
{
  return (accelerations.size() > 0);
}

void URPositionHardwareInterface::handleMoprimCommands()
{
  // Check if we have a new command
  if (!std::isnan(hw_moprim_commands_[0])) {
    ready_for_new_moprim_ = false;  // set to false to indicate that the driver is busy handling a command
    // set state interface immediately
    // --> if waiting for next read() cycle it happens sometimes that a command is overwritten
    hw_moprim_states_[1] = static_cast<double>(ready_for_new_moprim_);
    if (hw_moprim_commands_[0] == MotionType::STOP_MOTION) {
      std::lock_guard<std::mutex> guard(moprim_stop_mutex_);
      if (!new_moprim_stop_available_) {
        new_moprim_stop_available_ = true;
        resetMoprimCmdInterfaces();
      }
    } else if (hw_moprim_commands_[0] == MotionType::RESET_STOP) {
      std::lock_guard<std::mutex> guard(moprim_stop_mutex_);
      if (!new_moprim_reset_available_) {
        new_moprim_reset_available_ = true;
        resetMoprimCmdInterfaces();
      }
    } else {
      std::lock_guard<std::mutex> guard(moprim_cmd_mutex_);
      if (!new_moprim_cmd_available_) {
        // Copy command to thread-safe buffer
        pending_moprim_cmd_ = hw_moprim_commands_;
        new_moprim_cmd_available_ = true;
        resetMoprimCmdInterfaces();
      }
    }
  }

  // Send keepalive if current_moprim_execution_status_ is not EXECUTING
  if (ur_driver_ && current_moprim_execution_status_ != ExecutionState::EXECUTING) {
    ur_driver_->writeKeepalive();
  }
}

void URPositionHardwareInterface::resetMoprimCmdInterfaces()
{
  std::fill(hw_moprim_commands_.begin(), hw_moprim_commands_.end(), std::numeric_limits<double>::quiet_NaN());
}

void URPositionHardwareInterface::asyncMoprimStopThread()
{
  while (!async_moprim_thread_shutdown_) {
    if (new_moprim_stop_available_) {
      std::lock_guard<std::mutex> guard(moprim_stop_mutex_);
      new_moprim_stop_available_ = false;
      processMoprimStopCmd();
    } else if (new_moprim_reset_available_) {
      std::lock_guard<std::mutex> guard(moprim_stop_mutex_);
      new_moprim_reset_available_ = false;
      processMoprimResetCmd();
    }
    // Small sleep to prevent busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "[asyncMoprimStopThread] Exiting");
}

void URPositionHardwareInterface::processMoprimStopCmd()
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Received Motion Primitives STOP command");
  // delete motion sequence
  build_moprim_sequence_ = false;
  moprim_sequence_.clear();

  if (instruction_executor_->isTrajectoryRunning()) {
    RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Stopping motion ...");
    if (!instruction_executor_->cancelMotion()) {
      RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Failed to stop motion");
      current_moprim_execution_status_ = ExecutionState::ERROR;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Motion stopped successfully");
      current_moprim_execution_status_ = ExecutionState::STOPPED;
      ready_for_new_moprim_ = false;
    }
  } else {
    RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "No motion to stop");
    current_moprim_execution_status_ = ExecutionState::STOPPED;
    ready_for_new_moprim_ = false;
  }
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
              " [processMoprimStopCmd] After executing stop: current_moprim_execution_status_ = %d",
              current_moprim_execution_status_.load());
}

void URPositionHardwareInterface::processMoprimResetCmd()
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Received RESET_STOP command");
  current_moprim_execution_status_ = ExecutionState::IDLE;
  ready_for_new_moprim_ = true;  // set to true to allow sending new commands
}

void URPositionHardwareInterface::asyncMoprimCmdThread()
{
  while (!async_moprim_thread_shutdown_) {
    // Check for new commands
    if (new_moprim_cmd_available_) {
      // RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "[asyncMoprimCmdThread] New command available");
      std::vector<double> current_command;
      {
        std::lock_guard<std::mutex> guard(moprim_cmd_mutex_);
        current_command = pending_moprim_cmd_;
        new_moprim_cmd_available_ = false;
      }

      // Process the command
      processMoprimMotionCmd(current_command);
    }

    // Small sleep to prevent busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "[asyncMoprimCmdThread] Exiting");
}

void URPositionHardwareInterface::processMoprimMotionCmd(const std::vector<double>& command)
{
  if (command.empty() || std::isnan(command[0])) {
    return;
  }
  double velocity, acceleration, move_time;
  double motion_type = command[0];
  double blend_radius = command[21];

  try {
    switch (static_cast<uint8_t>(motion_type)) {
      case MotionType::MOTION_SEQUENCE_START:
      {
        RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Received MOTION_SEQUENCE_START: add all "
                                                                       "following "
                                                                       "commands to the motion sequence.");
        build_moprim_sequence_ = true;  // set flag to put all following commands into the motion sequence
        moprim_sequence_.clear();
        ready_for_new_moprim_ = true;  // set to true to allow sending new commands
        return;
      }

      case MotionType::MOTION_SEQUENCE_END:
      {
        RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
                    "Received MOTION_SEQUENCE_END: executing motion sequence with %zu motion primitives",
                    moprim_sequence_.size());
        build_moprim_sequence_ = false;
        current_moprim_execution_status_ = ExecutionState::EXECUTING;
        bool success = instruction_executor_->executeMotion(moprim_sequence_);
        current_moprim_execution_status_ = success ? ExecutionState::SUCCESS : ExecutionState::ERROR;
        RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
                    " [processMoprimMotionCmd] After executing motion sequence: current_moprim_execution_status_ = %d",
                    current_moprim_execution_status_.load());
        moprim_sequence_.clear();
        if (success) {
          ready_for_new_moprim_ = true;  // set to true to allow sending new commands
        }
        return;
      }

      case MotionType::LINEAR_JOINT:
      {  // moveJ
        // Check if joint positions are valid
        for (int i = 1; i <= 6; ++i) {
          if (std::isnan(command[i])) {
            RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Invalid motion command: joint positions "
                                                                            "contain NaN values");
            current_moprim_execution_status_ = ExecutionState::ERROR;
            return;
          }
        }
        urcl::vector6d_t joint_positions = { command[1], command[2], command[3], command[4], command[5], command[6] };

        // Get move_time OR (velocity AND acceleration)
        if (!getMoprimTimeOrVelAndAcc(command, velocity, acceleration, move_time)) {
          RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Invalid move_time, velocity or acceleration "
                                                                          "values");
          current_moprim_execution_status_ = ExecutionState::ERROR;
          return;
        }

        // Check if the command is part of a motion sequence or a single command
        if (build_moprim_sequence_) {  // Add command to motion sequence
          moprim_sequence_.emplace_back(std::make_shared<urcl::control::MoveJPrimitive>(
              joint_positions, blend_radius, std::chrono::milliseconds(static_cast<int>(move_time * 1000)),
              acceleration, velocity));
          RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
                      "Added moveJ to motion sequence with joint positions: [%f, %f, %f, %f, %f, %f], "
                      "velocity: %f, acceleration: %f, move_time: %f, blend_radius: %f",
                      joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3],
                      joint_positions[4], joint_positions[5], velocity, acceleration, move_time, blend_radius);
          ready_for_new_moprim_ = true;  // set to true to allow sending new commands
          return;
        } else {  // execute single primitive directly
          current_moprim_execution_status_ = ExecutionState::EXECUTING;
          RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
                      "Executing moveJ with joint positions: [%f, %f, %f, %f, %f, %f], "
                      "velocity: %f, acceleration: %f, move_time: %f, blend_radius: %f",
                      joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3],
                      joint_positions[4], joint_positions[5], velocity, acceleration, move_time, blend_radius);
          bool success = instruction_executor_->moveJ(joint_positions, acceleration, velocity, move_time, blend_radius);
          current_moprim_execution_status_ = success ? ExecutionState::SUCCESS : ExecutionState::ERROR;
          RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
                      " [processMoprimMotionCmd] After executing moveJ: current_moprim_execution_status_ = %d",
                      current_moprim_execution_status_.load());
          if (success) {
            ready_for_new_moprim_ = true;  // set to true to allow sending new commands
          }
          return;
        }
        break;
      }

      case MotionType::LINEAR_CARTESIAN:
      {  // moveL
        // Check if pose values (position and quaternion) are valid
        for (int i = 7; i <= 13; ++i) {
          if (std::isnan(command[i])) {
            RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Invalid motion command: pose contains NaN "
                                                                            "values");
            current_moprim_execution_status_ = ExecutionState::ERROR;
            return;
          }
        }
        double rx, ry, rz;
        quaternionToEuler(command[10], command[11], command[12], command[13], rx, ry, rz);
        urcl::Pose pose = { command[7], command[8], command[9], rx, ry, rz };

        // Get move_time OR (velocity AND acceleration)
        if (!getMoprimTimeOrVelAndAcc(command, velocity, acceleration, move_time)) {
          RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Invalid move_time, velocity or acceleration "
                                                                          "values");
          current_moprim_execution_status_ = ExecutionState::ERROR;
          return;
        }

        // Check if the command is part of a motion sequence or a single command
        if (build_moprim_sequence_) {  // Add command to motion sequence
          moprim_sequence_.emplace_back(std::make_shared<urcl::control::MoveLPrimitive>(
              pose, blend_radius, std::chrono::milliseconds(static_cast<int>(move_time * 1000)), acceleration,
              velocity));
          RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
                      "Added  moveL to motion sequence with pose: [%f, %f, %f, %f, %f, %f], "
                      "velocity: %f, acceleration: %f, move_time: %f, blend_radius: %f",
                      pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz, velocity, acceleration, move_time,
                      blend_radius);
          ready_for_new_moprim_ = true;  // set to true to allow sending new commands
          return;
        } else {  // execute single primitive directly
          current_moprim_execution_status_ = ExecutionState::EXECUTING;
          RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
                      "Executing moveL with pose: [%f, %f, %f, %f, %f, %f], "
                      "velocity: %f, acceleration: %f, move_time: %f, blend_radius: %f",
                      pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz, velocity, acceleration, move_time,
                      blend_radius);
          bool success = instruction_executor_->moveL(pose, acceleration, velocity, move_time, blend_radius);
          current_moprim_execution_status_ = success ? ExecutionState::SUCCESS : ExecutionState::ERROR;
          RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
                      " [processMoprimMotionCmd] After executing moveL: current_moprim_execution_status_ = %d",
                      current_moprim_execution_status_.load());
          if (success) {
            ready_for_new_moprim_ = true;  // set to true to allow sending new commands
          }
          return;
        }
        break;
      }

      case MotionType::CIRCULAR_CARTESIAN:
      {  // CIRC
        // Check if pose values (position and quaternion) are valid
        for (int i = 7; i <= 20; ++i) {
          if (std::isnan(command[i])) {
            RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Invalid motion command: pose contains NaN "
                                                                            "values");
            current_moprim_execution_status_ = ExecutionState::ERROR;
            return;
          }
        }

        // 0: Unconstrained mode, 1: Fixed mode
        // (https://tools.pages.cba.mit.edu/Universal_Robotics_UR10_Robot_Arm/scriptManual-3.5.4.pdf)
        int32_t mode = 0;

        // Get velocity and acceleration)
        if (!getMoprimVelAndAcc(command, velocity, acceleration, move_time)) {
          RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Invalid velocity or acceleration values");
          current_moprim_execution_status_ = ExecutionState::ERROR;
          return;
        }

        double via_rx, via_ry, via_rz;
        quaternionToEuler(command[17], command[18], command[19], command[20], via_rx, via_ry, via_rz);
        urcl::Pose via_pose = { command[14], command[15], command[16], via_rx, via_ry, via_rz };

        double goal_rx, goal_ry, goal_rz;
        quaternionToEuler(command[10], command[11], command[12], command[13], goal_rx, goal_ry, goal_rz);
        urcl::Pose goal_pose = { command[7], command[8], command[9], goal_rx, goal_ry, goal_rz };

        // Check if the command is part of a motion sequence or a single command
        if (build_moprim_sequence_) {  // Add command to motion sequence
          moprim_sequence_.emplace_back(std::make_shared<urcl::control::MoveCPrimitive>(
              via_pose, goal_pose, blend_radius, acceleration, velocity, mode));
          RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
                      "Added  moveC to motion sequence with via_pose: [%f, %f, %f, %f, %f, %f], "
                      "goal_pose: [%f, %f, %f, %f, %f, %f], velocity: %f,"
                      "acceleration: %f, blend_radius: %f, mode: %d",
                      via_pose.x, via_pose.y, via_pose.z, via_pose.rx, via_pose.ry, via_pose.rz, goal_pose.x,
                      goal_pose.y, goal_pose.z, goal_pose.rx, goal_pose.ry, goal_pose.rz, velocity, acceleration,
                      blend_radius, mode);
          ready_for_new_moprim_ = true;  // set to true to allow sending new commands
          return;
        } else {  // execute single primitive directly
          current_moprim_execution_status_ = ExecutionState::EXECUTING;
          RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
                      "Executing moveC with via_pose: [%f, %f, %f, %f, %f, %f], "
                      "goal_pose: [%f, %f, %f, %f, %f, %f], velocity: %f,"
                      "acceleration: %f, blend_radius: %f, mode: %d",
                      via_pose.x, via_pose.y, via_pose.z, via_pose.rx, via_pose.ry, via_pose.rz, goal_pose.x,
                      goal_pose.y, goal_pose.z, goal_pose.rx, goal_pose.ry, goal_pose.rz, velocity, acceleration,
                      blend_radius, mode);
          bool success = instruction_executor_->moveC(via_pose, goal_pose, acceleration, velocity, blend_radius, mode);
          current_moprim_execution_status_ = success ? ExecutionState::SUCCESS : ExecutionState::ERROR;
          RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
                      " [processMoprimMotionCmd] After executing moveC: current_moprim_execution_status_ = %d",
                      current_moprim_execution_status_.load());
          if (success) {
            ready_for_new_moprim_ = true;  // set to true to allow sending new commands
          }
          return;
        }
        break;
      }

      default:
      {
        RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"),
                     "Invalid motion command: motion type %f is not supported", motion_type);
        current_moprim_execution_status_ = ExecutionState::ERROR;
        return;
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Failed to execute motion command: %s", e.what());
    current_moprim_execution_status_ = ExecutionState::ERROR;
  }
}

// Convert quaternion to Euler angles (roll, pitch, yaw)
void URPositionHardwareInterface::quaternionToEuler(double qx, double qy, double qz, double qw, double& rx, double& ry, double& rz)
{
  tf2::Quaternion quat_tf(qx, qy, qz, qw);
  tf2::Matrix3x3 rot_mat(quat_tf);
  rot_mat.getRPY(rx, ry, rz);

  // RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"),
  //     "Converted quaternion [%f, %f, %f, %f] to Euler angles: [%f, %f, %f]",
  //     qx, qy, qz, qw, rx, ry, rz);
}

bool URPositionHardwareInterface::getMoprimTimeOrVelAndAcc(const std::vector<double>& command, double& velocity,
                                                           double& acceleration, double& move_time)
{
  // Check if move_time is valid
  if (!std::isnan(command[24]) && command[24] > 0.0) {
    move_time = command[24];
    // If move_time is valid, velocity and acceleration are ignored in moveJ and moveL, but must be > 0.0
    velocity = 1.0;
    acceleration = 1.0;
    return true;
  } else if (!std::isnan(command[22]) && command[22] > 0.0 && !std::isnan(command[23]) && command[23] > 0.0) {
    // If no valid move_time, check if velocity and acceleration are valid
    velocity = command[22];
    acceleration = command[23];
    move_time = 0.0;
    return true;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "move_time, velocity and acceleration are all "
                                                                    "invalid");
    return false;
  }
}

bool URPositionHardwareInterface::getMoprimVelAndAcc(const std::vector<double>& command, double& velocity,
                                                     double& acceleration, double& move_time)
{
  // Check if velocity and acceleration are valid
  if (!std::isnan(command[22]) && command[22] > 0.0 && !std::isnan(command[23]) && command[23] > 0.0) {
    velocity = command[22];
    acceleration = command[23];
    move_time = 0.0;
    return true;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "velocity or acceleration is invalid");
    return false;
  }
}

}  // namespace ur_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_robot_driver::URPositionHardwareInterface, hardware_interface::SystemInterface)
