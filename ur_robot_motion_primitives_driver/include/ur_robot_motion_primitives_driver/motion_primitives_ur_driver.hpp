// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UR_ROBOT_MOTION_PRIMITIVES_DRIVER__MOTION_PRIMITIVES_UR_DRIVER_HPP_
#define UR_ROBOT_MOTION_PRIMITIVES_DRIVER__MOTION_PRIMITIVES_UR_DRIVER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "ur_robot_motion_primitives_driver/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/ur/instruction_executor.h>

#include "ur_robot_driver/stateinterface_helper.hpp"

#include "motion_primitives_forward_controller/execution_state.hpp"

namespace ur_robot_motion_primitives_driver
{
class MotionPrimitivesUrDriver : public hardware_interface::SystemInterface
{
public:
  virtual ~MotionPrimitivesUrDriver();

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /*!
   * \brief Callback to handle a change in the current state of the URCaps program running on the
   * robot. Executed only on the state change.
   *
   * \param program_running True when the URCap program is running on the robot.
   */
  void handleRobotProgramState(bool program_running);
  void quaternionToEuler(double qx, double qy, double qz, double qw, double& rx, double& ry, double& rz);


private:
  void reset_command_interfaces(); // Function to reset all command interfaces to NaN
  void asyncCommandThread();
  void asyncStopMotionThread();
  void processMotionCommand(const std::vector<double>& command);
  void processStopCommand();
  bool getMovetimeOrVelocityAndAcceleration(const std::vector<double>& command, double& velocity, double& acceleration, double& move_time);
  bool getVelocityAndAcceleration(const std::vector<double>& command, double& velocity, double& acceleration, double& move_time);
  
  // Async thread handling
  std::unique_ptr<std::thread> async_command_thread_;
  std::unique_ptr<std::thread> async_stop_motion_thread_;
  std::atomic_bool async_thread_shutdown_{false};
  std::mutex command_mutex_;
  std::mutex stop_mutex_;
  
  // Command buffer for thread-safe communication
  std::vector<double> pending_command_;
  std::atomic_bool new_command_available_{false};
  std::atomic_bool new_stop_available_{false};

  
  // Execution status
  std::atomic<int8_t> current_execution_status_{ExecutionState::IDLE};


  std::vector<double> hw_mo_prim_commands_;   // Command interfaces for the motion primitives
  std::vector<double> hw_mo_prim_states_;    // State interfaces for the motion primitives

  std::shared_ptr<urcl::UrDriver> ur_driver_;
  std::shared_ptr<urcl::InstructionExecutor> instruction_executor_; // Shared pointer to the InstructionExecutor, which is responsible for executing motion commands.

  ur_robot_driver::URStateHelper state_helper_; // Helper class to handle state interface stuff from the URPositionHardwareInterface
  std::atomic_bool rtde_comm_has_been_started_ = false;

  urcl::RobotReceiveTimeout receive_timeout_ = urcl::RobotReceiveTimeout::millisec(20);

  bool initialized_;
  bool robot_program_running_;

  bool non_blocking_read_;

  std::atomic_bool build_motion_sequence_{false};   // flag to put all following primitives into a motion sequence instead of sending single primitives
  std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> motion_sequence_;
};

}  // namespace ur_robot_motion_primitives_driver

#endif  // UR_ROBOT_MOTION_PRIMITIVES_DRIVER__MOTION_PRIMITIVES_UR_DRIVER_HPP_
