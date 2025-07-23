// Copyright 2025, Universal Robots A/S
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

#pragma once

#include <memory>

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "realtime_tools/realtime_thread_safe_box.hpp"

#include "ur_controllers/pd_torque_controller_parameters.hpp"

namespace ur_controllers
{
class PDTorqueController : public controller_interface::ControllerInterface
{
public:
  using CmdType = std_msgs::msg::Float64MultiArray;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Change the input for the update function
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

private:
  enum class ControlType
  {
    NONE = 0,
    JOINT_SPACE = 9,
    TASK_SPACE = 10,
  };

  void command_callback(const CmdType::SharedPtr msg, const ControlType control_type);

  std::shared_ptr<pd_torque_controller::ParamListener> param_listener_;
  pd_torque_controller::Params params_;

  rclcpp::Subscription<CmdType>::SharedPtr joints_command_sub_;
  rclcpp::Subscription<CmdType>::SharedPtr task_space_command_sub_;

  // the realtime container to exchange the reference from subscriber
  realtime_tools::RealtimeThreadSafeBox<CmdType> rt_command_;
  // save the last reference in case of unable to get value from box
  CmdType joint_commands_;

  std::atomic<ControlType> control_type_{ ControlType::NONE };
};
}  // namespace ur_controllers
