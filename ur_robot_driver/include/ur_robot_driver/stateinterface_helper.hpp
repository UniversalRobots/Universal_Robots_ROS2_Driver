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
 * \author  Mathias Fuhrer
 * \date    2025-04-11
 *
 */
//----------------------------------------------------------------------
#pragma once

#include <bitset>
#include <vector>
#include <array>
#include <algorithm>
#include <memory>
#include <string>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rtde = urcl::rtde_interface;

namespace ur_robot_driver
{

enum class PausingState
{
  PAUSED,
  RUNNING,
  RAMPUP
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

class URStateHelper
{
public:
  URStateHelper() = default;

  void initialize()
  {
    urcl_joint_positions_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    urcl_joint_velocities_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    urcl_joint_efforts_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    urcl_ft_sensor_measurements_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    urcl_tcp_pose_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    runtime_state_ = static_cast<uint32_t>(rtde::RUNTIME_STATE::STOPPED);
    pausing_state_ = PausingState::RUNNING;
    pausing_ramp_up_increment_ = 0.01;
    system_interface_initialized_ = 0.0;
  }

  void setRobotSoftwareVersion(const urcl::VersionInformation& version_info)
  {
    get_robot_software_version_major_ = version_info.major;
    get_robot_software_version_minor_ = version_info.minor;
    get_robot_software_version_build_ = version_info.build;
    get_robot_software_version_bugfix_ = version_info.bugfix;
  }

  std::vector<hardware_interface::StateInterface> generate_state_interfaces(std::vector<std::string>& joint_names,
                                                                            std::string& tf_prefix,
                                                                            std::vector<std::string>& sensor_names)
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Joints
    for (size_t i = 0; i < joint_names.size(); ++i) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint_names[i], hardware_interface::HW_IF_POSITION, &urcl_joint_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint_names[i], hardware_interface::HW_IF_VELOCITY, &urcl_joint_velocities_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names[i], hardware_interface::HW_IF_EFFORT,
                                                                       &urcl_joint_efforts_[i]));
    }

    // Speed scaling
    state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "speed_scaling",
                                                                     "speed_scaling_factor", &speed_scaling_combined_));

    // Force-torque sensor
    for (const auto& sensor : sensor_names) {
      if (sensor == tf_prefix + "tcp_fts_sensor") {
        const std::vector<std::string> fts_names = {
          "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"
        };
        for (uint j = 0; j < 6; ++j) {
          state_interfaces.emplace_back(
              hardware_interface::StateInterface(sensor, fts_names[j], &urcl_ft_sensor_measurements_[j]));
        }
      }
    }

    // GPIO
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

    // Other states
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_voltage", &tool_output_voltage_copy_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "gpio", "robot_mode", &robot_mode_copy_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "gpio", "safety_mode", &safety_mode_copy_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "gpio", "tool_mode", &tool_mode_copy_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "gpio", "tool_output_current", &tool_output_current_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "gpio", "tool_temperature", &tool_temperature_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(tf_prefix + "system_interface", "initialized",
                                                                     &system_interface_initialized_));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "gpio", "program_running", &robot_program_running_copy_));

    // TCP pose
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "tcp_pose", "position.x", &urcl_tcp_pose_[0]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "tcp_pose", "position.y", &urcl_tcp_pose_[1]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "tcp_pose", "position.z", &urcl_tcp_pose_[2]));

    // TCP rotation
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.x", &tcp_rotation_buffer_.x));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.y", &tcp_rotation_buffer_.y));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.z", &tcp_rotation_buffer_.z));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(tf_prefix + "tcp_pose", "orientation.w", &tcp_rotation_buffer_.w));

    // Software version
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "get_robot_software_version", "get_version_major", &get_robot_software_version_major_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "get_robot_software_version", "get_version_minor", &get_robot_software_version_minor_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "get_robot_software_version", "get_version_bugfix", &get_robot_software_version_bugfix_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        tf_prefix + "get_robot_software_version", "get_version_build", &get_robot_software_version_build_));

    return state_interfaces;
  }

  void processStateData(std::unique_ptr<rtde::DataPackage>& data_pkg, bool& initialized, bool& robot_program_running)
  {
    if (!data_pkg) {
      return;
    }

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
    readData(data_pkg, "tcp_offset", tcp_offset_);

    readBitsetData<uint32_t>(data_pkg, "robot_status_bits", robot_status_bits_);
    readBitsetData<uint32_t>(data_pkg, "safety_status_bits", safety_status_bits_);
    readBitsetData<uint64_t>(data_pkg, "actual_digital_input_bits", actual_dig_in_bits_);
    readBitsetData<uint64_t>(data_pkg, "actual_digital_output_bits", actual_dig_out_bits_);
    readBitsetData<uint32_t>(data_pkg, "analog_io_types", analog_io_types_);
    readBitsetData<uint32_t>(data_pkg, "tool_analog_input_types", tool_analog_input_types_);

    extractToolPose();
    transformForceTorque();

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

    updateNonDoubleValues(initialized, robot_program_running);
  }
  // Public member variables
  urcl::vector6d_t urcl_joint_positions_;
  uint32_t runtime_state_;

private:
  void updateNonDoubleValues(bool initialized, bool robot_program_running)
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
    system_interface_initialized_ = initialized ? 1.0 : 0.0;
    robot_program_running_copy_ = robot_program_running ? 1.0 : 0.0;
  }

  template <typename T>
  void readData(const std::unique_ptr<rtde::DataPackage>& data_pkg, const std::string& var_name, T& data)
  {
    if (!data_pkg->getData(var_name, data)) {
      // This throwing should never happen unless misconfigured
      std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }
  }

  template <typename T, size_t N>
  void readBitsetData(const std::unique_ptr<rtde::DataPackage>& data_pkg, const std::string& var_name,
                      std::bitset<N>& data)
  {
    if (!data_pkg->getData<T, N>(var_name, data)) {
      // This throwing should never happen unless misconfigured
      std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }
  }

  void extractToolPose()
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
    tcp_rotation_buffer_.set(tcp_rotation_quat_);
  }

  void transformForceTorque()
  {
    KDL::Wrench ft(
        KDL::Vector(urcl_ft_sensor_measurements_[0], urcl_ft_sensor_measurements_[1], urcl_ft_sensor_measurements_[2]),
        KDL::Vector(urcl_ft_sensor_measurements_[3], urcl_ft_sensor_measurements_[4], urcl_ft_sensor_measurements_[5]));
    if (get_robot_software_version_major_ >= 5)  // e-Series
    {
      // Setup necessary frames
      KDL::Vector vec = KDL::Vector(tcp_offset_[3], tcp_offset_[4], tcp_offset_[5]);
      double angle = vec.Normalize();
      KDL::Rotation rotation = KDL::Rotation::Rot(vec, angle);
      KDL::Frame flange_to_tcp = KDL::Frame(rotation, KDL::Vector(tcp_offset_[0], tcp_offset_[1], tcp_offset_[2]));

      vec = KDL::Vector(urcl_target_tcp_pose_[3], urcl_target_tcp_pose_[4], urcl_target_tcp_pose_[5]);
      angle = vec.Normalize();
      rotation = KDL::Rotation::Rot(vec, angle);
      KDL::Frame base_to_tcp = KDL::Frame(
          rotation, KDL::Vector(urcl_target_tcp_pose_[0], urcl_target_tcp_pose_[1], urcl_target_tcp_pose_[2]));
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

  // Private member variables
  urcl::vector6d_t urcl_joint_velocities_;
  urcl::vector6d_t urcl_joint_efforts_;
  double target_speed_fraction_;
  double speed_scaling_;
  urcl::vector6d_t urcl_ft_sensor_measurements_;
  urcl::vector6d_t urcl_tcp_pose_;
  std::array<double, 2> standard_analog_input_;
  std::array<double, 2> standard_analog_output_;
  std::array<double, 2> tool_analog_input_;
  double tool_output_current_;
  double tool_temperature_;
  std::bitset<4> robot_status_bits_;
  std::array<double, 4> robot_status_bits_copy_;
  std::bitset<11> safety_status_bits_;
  std::array<double, 11> safety_status_bits_copy_;
  std::bitset<18> actual_dig_in_bits_;
  std::array<double, 18> actual_dig_in_bits_copy_;
  std::bitset<4> analog_io_types_;
  std::array<double, 4> analog_io_types_copy_;
  std::bitset<2> tool_analog_input_types_;
  std::array<double, 2> tool_analog_input_types_copy_;
  tf2::Quaternion tcp_rotation_quat_;
  ur_robot_driver::Quaternion tcp_rotation_buffer_;
  // tf2::Vector3 tcp_force_;
  // tf2::Vector3 tcp_torque_;
  std::bitset<18> actual_dig_out_bits_;
  std::array<double, 18> actual_dig_out_bits_copy_;
  int32_t tool_output_voltage_;
  double tool_output_voltage_copy_;
  int32_t robot_mode_;
  double robot_mode_copy_;
  int32_t safety_mode_;
  double safety_mode_copy_;
  uint32_t tool_mode_;
  double tool_mode_copy_;
  double system_interface_initialized_;
  double robot_program_running_copy_;
  double speed_scaling_combined_;
  double get_robot_software_version_major_;
  double get_robot_software_version_minor_;
  double get_robot_software_version_bugfix_;
  double get_robot_software_version_build_;
  PausingState pausing_state_;
  double pausing_ramp_up_increment_;
  urcl::vector6d_t urcl_target_tcp_pose_;
  urcl::vector6d_t tcp_offset_;
};

}  // namespace ur_robot_driver
